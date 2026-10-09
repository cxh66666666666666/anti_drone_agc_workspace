#!/usr/bin/env python3
"""audit_interfaces.py —— 协同层接口现场核查（需要 ROS master 正在运行）。

用法：
    python3 audit_interfaces.py [--seconds 5]

逐项核查并打印 PASS/FAIL 表格：
- 8 个协同话题存在、类型与契约表一致；
- 唯一实际发布者且与契约表一致（rosgraph Master.getSystemState）；
- 窗口内实际频率与契约要求比对（订阅采样）；
- 各话题样本 header.frame_id 与 odom child_frame_id；
- TF 闭合（world / air_local / drone/base_link / car/base_link）且无裸 base_link；
- /use_sim_time=true、无 ground_gazebo、唯一 gazebo/gzserver 节点；
- 车 odom 与 Gazebo 实际车体一致性（同一世界系位置差 < 0.05 m）；
- M2 项：空中 odom 与 Gazebo iris 一致性、同拍模型齐全、同拍新鲜度、
  消息时间戳取自 /clock、/agc/map 与场景文件一致。

结果写入 <root>/audit_interfaces.json；任一 FAIL 时退出码为 1。
"""

import argparse
import json
import math
import os
import sys
import time

import rosgraph
import rosnode
import rospy
import yaml
from roslib.message import get_message_class

DEFAULT_ROOT = "/home/ubuntu/anti_drone_agc_workspace/experiments/agc_runs"
GZ_TOPIC = "/gazebo/model_states"
GZ_TYPE = "gazebo_msgs/ModelStates"
CAR_MODEL = "agc_car"
CAR_ODOM_TOPIC = "/agc/ground/odom"

# M2 专用核查常量
AIR_MODEL = "iris"
AIR_ODOM_TOPIC = "/agc/air/odom"
GROUND_ODOM_TOPIC = "/agc/ground/odom"
TARGETS_TOPIC = "/agc/targets"
MAP_TOPIC = "/agc/map"
CLOCK_TOPIC = "/clock"
CLOCK_TYPE = "rosgraph_msgs/Clock"
# 同拍 world 必须同时存在的模型名
REQUIRED_MODELS = ["iris", "agc_car", "intruder_uav", "pole_1", "pole_2"]
# 新鲜度阈值（秒）：空中/地面 odom 与目标观测
ODOM_AGE_MAX_S = 0.5
TARGET_AGE_MAX_S = 0.5

# 协同话题契约表：类型 / 唯一发布者 / 频率要求
# hard=True 表示契约写“≥”的硬下限；hard=False 表示标称频率，允许 20% 容差。
TOPICS = [
    {"name": "/agc/air/odom", "type": "nav_msgs/Odometry",
     "pub": "air_odom_adapter", "hz": 20.0, "hard": True},
    {"name": "/agc/ground/odom", "type": "nav_msgs/Odometry",
     "pub": "ground_odom_adapter", "hz": 20.0, "hard": True},
    {"name": "/agc/targets", "type": "air_ground_core/TargetTrackArray",
     "pub": "gt_adapter", "hz": 10.0, "hard": False},
    {"name": "/agc/map", "type": "nav_msgs/OccupancyGrid",
     "pub": "map_adapter", "hz": 1.0, "hard": True},
    {"name": "/agc/ground/goal", "type": "geometry_msgs/PoseStamped",
     "pub": "task_manager", "hz": 5.0, "hard": True},
    {"name": "/agc/ground/path", "type": "nav_msgs/Path",
     "pub": "ground_planner", "hz": 1.0, "hard": True},
    {"name": "/agc/ground/cmd_vel", "type": "geometry_msgs/TwistStamped",
     "pub": "ground_commander", "hz": 20.0, "hard": False},
    {"name": "/agc/task/state", "type": "air_ground_core/AgcTaskState",
     "pub": "task_manager", "hz": 2.0, "hard": False},
]

# header.frame_id 期望（其余话题均为 world）
FRAME_EXPECT = {"/agc/ground/cmd_vel": "car/base_link"}
FRAME_DEFAULT = "world"
# odom 的 child_frame_id 期望
CHILD_FRAME_EXPECT = {
    "/agc/air/odom": "drone/base_link",
    "/agc/ground/odom": "car/base_link",
}

TF_FRAMES = ["world", "air_local", "drone/base_link", "car/base_link"]

# 保持 TransformListener 等对象引用，避免被 GC
_KEEPALIVE = []


def _basename(node):
    return node.strip("/").split("/")[-1]


def _resolve_msg_class(type_str):
    """安全解析消息类：空串或未知类型返回 None（get_message_class 对空串会抛异常）。"""
    if not type_str:
        return None
    try:
        return get_message_class(type_str)
    except Exception:
        return None


def _node_names():
    try:
        return rosnode.get_node_names()
    except Exception:
        return []


def _dist3(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _model_pos(gz_msg, name):
    """从 ModelStates 取指定模型世界坐标 (x, y, z)；缺失返回 None。"""
    try:
        for nm, pose in zip(gz_msg.name, gz_msg.pose):
            if nm == name:
                return (pose.position.x, pose.position.y, pose.position.z)
    except Exception:
        return None
    return None


def _make_cb(col):
    def _cb(msg):
        col["count"] += 1
        if col.get("first") is None:
            col["first"] = time.monotonic()
        col["sample"] = msg
    return _cb


def main():
    parser = argparse.ArgumentParser(description="协同层接口现场核查")
    parser.add_argument("--seconds", type=float, default=5.0, help="采样窗口秒数（默认 5）")
    parser.add_argument("--fault-mode", action="store_true",
                        help="M4 注入模式：允许目标观测新鲜度超限（延迟/丢失注入时不算 FAIL）")
    args = parser.parse_args()
    seconds = max(1.0, float(args.seconds))

    rospy.init_node("agc_audit_interfaces", anonymous=True)
    root = rospy.get_param("/agc/log/root", DEFAULT_ROOT) or DEFAULT_ROOT

    # M4：按 run 前固定的 source 组织期望接口表（契约 v1.2）
    target_source = rospy.get_param("/agc/target/source", "gt_adapter")
    obstacle_source = rospy.get_param("/agc/obstacles/source", "scenario")
    topics = []
    for spec in TOPICS:
        s = dict(spec)
        if s["name"] == TARGETS_TOPIC:
            s["pub"] = target_source  # /agc/targets 期望发布者随 source 切换
        topics.append(s)
    if target_source == "air_fusion" or obstacle_source == "air_fusion":
        topics.append({"name": "/agc/air/obs", "type": "air_ground_core/AirObservationArray",
                       "pub": "air_perception", "hz": 10.0, "hard": True})
    if obstacle_source == "air_fusion":
        topics.append({"name": "/agc/obstacles", "type": "air_ground_core/ObstacleTrackArray",
                       "pub": "air_fusion", "hz": 8.0, "hard": False})

    checks = []

    def add(name, ok, detail=""):
        checks.append({"name": name, "status": "PASS" if ok else "FAIL", "detail": str(detail)})

    try:
        master = rosgraph.Master("/agc_audit")
        pubs, _subs, _srvs = master.getSystemState()
        type_map = dict(master.getTopicTypes())
    except Exception as exc:
        print("无法连接 ROS master：%s" % exc)
        sys.exit(1)

    pub_map = {}
    for name, nodes in pubs:
        pub_map[name] = list(nodes)

    # ---------------------------------------------------------- 采样订阅
    collectors = {}
    for spec in topics:
        name = spec["name"]
        actual_type = type_map.get(name, "")
        spec["_actual_type"] = actual_type
        cls = _resolve_msg_class(actual_type) or _resolve_msg_class(spec["type"])
        if cls is None:
            add("话题存在/类型 %s" % name, False,
                "消息类不可用 (actual=%s)" % (actual_type or "<none>"))
            continue
        col = {"count": 0, "first": None, "sample": None}
        collectors[name] = col
        rospy.Subscriber(name, cls, _make_cb(col), queue_size=200)

    # Gazebo 实际车体（用于 odom 一致性）
    gz_col = {"count": 0, "first": None, "sample": None}
    gz_actual = type_map.get(GZ_TOPIC, "")
    gz_cls = _resolve_msg_class(gz_actual) or _resolve_msg_class(GZ_TYPE)
    if gz_cls is not None:
        rospy.Subscriber(GZ_TOPIC, gz_cls, _make_cb(gz_col), queue_size=200)

    # /clock 仿真时刻（用于新鲜度与时间戳核查）
    clock_col = {"count": 0, "first": None, "sample": None}
    clock_cls = _resolve_msg_class(CLOCK_TYPE)
    if clock_cls is not None:
        rospy.Subscriber(CLOCK_TOPIC, clock_cls, _make_cb(clock_col), queue_size=200)

    # 裸 base_link 采样（/tf 的 child_frame_id）
    tf_children = set()
    tf_parents = set()
    tf_samples = {"n": 0}
    try:
        from tf2_msgs.msg import TFMessage
    except Exception:
        TFMessage = None
    if TFMessage is not None:
        def _tf_cb(msg):
            tf_samples["n"] += 1
            for tr in msg.transforms:
                tf_children.add(tr.child_frame_id)
                tf_parents.add(tr.header.frame_id)
        rospy.Subscriber("/tf", TFMessage, _tf_cb, queue_size=200)

    # ---------------------------------------------------------- 采样窗口
    t0 = time.monotonic()
    while not rospy.is_shutdown() and (time.monotonic() - t0) < seconds:
        rospy.sleep(0.05)
    window = time.monotonic() - t0

    # ---------------------------------------------------------- 存在/类型/发布者/频率
    for spec in topics:
        name = spec["name"]
        nodes = pub_map.get(name, [])
        actual_type = spec.get("_actual_type", "")
        exists = bool(nodes) or bool(actual_type)
        add("话题存在 %s" % name, exists, "发布者=%s" % (nodes or "无"))
        add("消息类型 %s" % name, actual_type == spec["type"],
            "actual=%s expected=%s" % (actual_type or "<none>", spec["type"]))

        uniq = len(nodes) == 1
        match = uniq and _basename(nodes[0]) == spec["pub"]
        add("唯一发布者 %s" % name, bool(match),
            "nodes=%s expected=%s" % (nodes or "无", spec["pub"]))

        col = collectors.get(name)
        hz = (col["count"] / window) if (col and window > 0) else 0.0
        min_hz = spec["hz"] if spec["hard"] else spec["hz"] * 0.8
        ok_hz = bool(col) and hz >= min_hz
        add("频率 %s" % name, ok_hz,
            "%.2f Hz (要求 %s%.1f Hz)" % (hz, "≥" if spec["hard"] else "标称 ", spec["hz"]))

        # 采样样本用于 frame 核查
        if col and col["sample"] is not None:
            sample = col["sample"]
            want_frame = FRAME_EXPECT.get(name, FRAME_DEFAULT)
            hdr = getattr(sample, "header", None)
            got_frame = getattr(hdr, "frame_id", None) if hdr is not None else None
            if hdr is None:
                add("frame_id %s" % name, False, "消息无 header")
            else:
                add("frame_id %s" % name, got_frame == want_frame,
                    "actual=%s expected=%s" % (got_frame, want_frame))
            if name in CHILD_FRAME_EXPECT:
                got_child = getattr(sample, "child_frame_id", None)
                add("child_frame_id %s" % name, got_child == CHILD_FRAME_EXPECT[name],
                    "actual=%s expected=%s" % (got_child, CHILD_FRAME_EXPECT[name]))
        else:
            add("样本 %s" % name, False, "窗口内无消息，无法核查 frame")

    # ---------------------------------------------------------- TF 核查
    try:
        import tf2_ros
        buffer = tf2_ros.Buffer()
        _KEEPALIVE.append(tf2_ros.TransformListener(buffer))
        rospy.sleep(1.0)
    except Exception as exc:
        buffer = None
        add("TF 监听初始化", False, str(exc))

    if buffer is not None:
        for frame in TF_FRAMES:
            if frame == "world":
                add("TF frame world", True, "根帧（恒等）")
                continue
            try:
                buffer.lookup_transform("world", frame, rospy.Time(0), rospy.Duration(1.0))
                add("TF world->%s" % frame, True, "lookup_transform OK")
            except Exception as exc:
                add("TF world->%s" % frame, False,
                    "lookup 失败: %s" % type(exc).__name__)

    # 裸 base_link 检查
    if TFMessage is None:
        add("无裸 base_link", False, "tf2_msgs 不可用，无法采样 /tf")
    elif tf_samples["n"] == 0:
        add("无裸 base_link", False, "未采样到 /tf 消息")
    else:
        # 判定口径：协同层不得把任何变换发布到裸 base_link。
        # PX4 SITL / MAVROS 会一次性发布 map/odom/base_link_frd 等历史帧，
        # 这里把它们单独列出提示，不与协同层帧混淆。
        collab = {"world", "air_local", "drone/base_link", "car/base_link"}
        legacy = sorted((tf_children | tf_parents) - collab)
        add("无裸 base_link", "base_link" not in tf_children,
            "采样 %d 帧消息；child 含裸 base_link=%s；协同层外历史帧=%s"
            % (tf_samples["n"], "base_link" in tf_children, legacy or "无"))

    # ---------------------------------------------------------- 基础设施
    use_sim = rospy.get_param("/use_sim_time", False)
    add("/use_sim_time=true", use_sim is True, "actual=%s" % use_sim)

    nodes = _node_names()
    has_ground_gazebo = any(_basename(n) == "ground_gazebo" for n in nodes)
    add("无 ground_gazebo 节点", not has_ground_gazebo,
        "ground_gazebo 存在" if has_ground_gazebo else "未发现")

    gz_nodes = [n for n in nodes
                if any(k in _basename(n).lower() for k in ("gazebo", "gzserver", "gzclient"))]
    # 契约要求「一个 agc_world / gzserver」；gzclient(/gazebo_gui) 只是查看器，
    # 与 headless 无关，不计入「多世界」。故只需唯一 gzserver 侧节点。
    gz_servers = [n for n in gz_nodes
                  if _basename(n) == "gazebo" or "gzserver" in _basename(n).lower()]
    gz_clients = [n for n in gz_nodes if n not in gz_servers]
    add("唯一 gazebo/gzserver 节点", len(gz_servers) == 1,
        "gzserver=%s；查看器=%s" % (gz_servers, gz_clients or "无"))
    clock_pubs = pub_map.get(CLOCK_TOPIC, [])
    add("/clock 唯一发布者", len(clock_pubs) == 1, "publishers=%s" % (clock_pubs or "无"))

    # ---------------------------------------------------------- 车 odom 与 Gazebo 一致性
    gz_ok = gz_cls is not None
    if not gz_ok:
        add("车 odom 与 Gazebo 一致", False,
            "gazebo_msgs 不可用 (actual=%s)" % (gz_actual or "<none>"))
    else:
        odom_col = collectors.get(CAR_ODOM_TOPIC)
        gz_msg = gz_col["sample"]
        odom_msg = odom_col["sample"] if odom_col else None
        if gz_msg is None or odom_msg is None:
            add("车 odom 与 Gazebo 一致", False,
                "样本不足 (model_states=%s, ground_odom=%s)"
                % (gz_msg is not None, odom_msg is not None))
        else:
            car_pos = None
            try:
                for nm, pose in zip(gz_msg.name, gz_msg.pose):
                    if nm == CAR_MODEL:
                        car_pos = (pose.position.x, pose.position.y, pose.position.z)
                        break
            except Exception:
                car_pos = None
            odom_pos = (odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        odom_msg.pose.pose.position.z)
            if car_pos is None:
                add("车 odom 与 Gazebo 一致", False, "model_states 中无 %s" % CAR_MODEL)
            else:
                d = _dist3(car_pos, odom_pos)
                add("车 odom 与 Gazebo 一致", d < 0.05,
                    "距离差 %.4f m (阈值 0.05)；gazebo=%s odom=%s"
                    % (d, tuple(round(v, 3) for v in car_pos),
                       tuple(round(v, 3) for v in odom_pos)))

    # ---------------------------------------------------------- 空中 odom 与 Gazebo 一致性
    if not gz_ok:
        add("空中 odom 与 Gazebo 一致", False,
            "gazebo_msgs 不可用 (actual=%s)" % (gz_actual or "<none>"))
    else:
        gz_msg = gz_col["sample"]
        air_col = collectors.get(AIR_ODOM_TOPIC)
        air_msg = air_col["sample"] if air_col else None
        if gz_msg is None or air_msg is None:
            add("空中 odom 与 Gazebo 一致", False,
                "样本不足 (model_states=%s, air_odom=%s)"
                % (gz_msg is not None, air_msg is not None))
        else:
            air_pos = _model_pos(gz_msg, AIR_MODEL)
            air_odom_pos = (air_msg.pose.pose.position.x,
                            air_msg.pose.pose.position.y,
                            air_msg.pose.pose.position.z)
            if air_pos is None:
                add("空中 odom 与 Gazebo 一致", False, "model_states 中无 %s" % AIR_MODEL)
            else:
                d = _dist3(air_pos, air_odom_pos)
                add("空中 odom 与 Gazebo 一致", d < 0.20,
                    "距离差 %.4f m (阈值 0.20)；gazebo=%s odom=%s"
                    % (d, tuple(round(v, 3) for v in air_pos),
                       tuple(round(v, 3) for v in air_odom_pos)))

    # ---------------------------------------------------------- 同拍模型齐全
    gz_msg = gz_col["sample"]
    if not gz_ok or gz_msg is None:
        add("同拍模型齐全", False,
            "model_states 样本不足 (gazebo_msgs=%s)" % gz_ok)
    else:
        try:
            names = set(gz_msg.name)
        except Exception:
            names = set()
        missing = [m for m in REQUIRED_MODELS if m not in names]
        add("同拍模型齐全", not missing,
            "缺少=%s；实际=%s" % (missing or "无", sorted(names)))

    # ---------------------------------------------------------- 仿真时钟
    clock_msg = clock_col["sample"]
    if clock_msg is not None:
        now_sec = clock_msg.clock.to_sec()
        clock_src = CLOCK_TOPIC
    else:
        now_sec = rospy.Time.now().to_sec()
        clock_src = "rospy.Time.now（无 /clock 样本，回退）"

    # 同拍新鲜度：同一窗口内三条链路样本皆有，且年龄不超阈值
    fresh_ok = True
    fresh_parts = []
    for label, topic, limit in [("空中 odom", AIR_ODOM_TOPIC, ODOM_AGE_MAX_S),
                                ("地面 odom", GROUND_ODOM_TOPIC, ODOM_AGE_MAX_S),
                                ("目标观测", TARGETS_TOPIC, TARGET_AGE_MAX_S)]:
        col = collectors.get(topic)
        msg = col["sample"] if col else None
        if msg is None:
            fresh_ok = False
            fresh_parts.append("%s=无样本" % label)
            continue
        hdr = getattr(msg, "header", None)
        if hdr is None:
            fresh_ok = False
            fresh_parts.append("%s=无 header" % label)
            continue
        age = now_sec - hdr.stamp.to_sec()
        relaxed = args.fault_mode and topic == TARGETS_TOPIC
        if age > limit and not relaxed:
            fresh_ok = False
        fresh_parts.append("%s年龄=%.3fs(≤%.1f%s)" % (
            label, age, limit, "，注入模式放宽" if relaxed else ""))
    add("同拍新鲜度", fresh_ok,
        "%s；时钟源=%s" % ("；".join(fresh_parts), clock_src))

    # 时间戳属仿真时钟：|header.stamp - /clock| ≤ 1.0 s（排除墙钟 epoch 取值）
    ts_ok = True
    ts_parts = []
    for label, topic in [("空中 odom", AIR_ODOM_TOPIC),
                         ("地面 odom", GROUND_ODOM_TOPIC),
                         ("目标观测", TARGETS_TOPIC),
                         ("地图", MAP_TOPIC)]:
        col = collectors.get(topic)
        msg = col["sample"] if col else None
        if msg is None:
            ts_ok = False
            ts_parts.append("%s=无样本" % label)
            continue
        hdr = getattr(msg, "header", None)
        if hdr is None:
            ts_ok = False
            ts_parts.append("%s=无 header" % label)
            continue
        delta = abs(hdr.stamp.to_sec() - now_sec)
        if delta > 1.0:
            ts_ok = False
        ts_parts.append("%sΔ=%.3fs(≤1.0)" % (label, delta))
    add("时间戳属仿真时钟", ts_ok,
        "%s；参考时钟=%s" % ("；".join(ts_parts), clock_src))

    # ---------------------------------------------------------- 地图与场景一致
    map_col = collectors.get(MAP_TOPIC)
    map_msg = map_col["sample"] if map_col else None
    scenario_file = rospy.get_param("/agc/scenario_file", "")
    version_hash = rospy.get_param("/agc/map/version_hash", "")

    if map_msg is None:
        add("地图与场景一致", False, "无 /agc/map 样本")
    elif not scenario_file or not os.path.isfile(scenario_file):
        add("地图与场景一致", False,
            "场景文件不可用: %s" % (scenario_file or "<none>"))
    else:
        scenario = None
        try:
            with open(scenario_file, "r") as f:
                scenario = yaml.safe_load(f) or {}
        except Exception as exc:
            add("地图与场景一致", False, "读取场景 YAML 失败: %s" % exc)
        if scenario is not None:
            mism = []
            info = map_msg.info
            data = list(map_msg.data)
            occupied = sum(1 for v in data if v == 100)
            try:
                mp = scenario.get("map") or {}
                res = float(mp["resolution_m"])
                sx = float(mp["size_x_m"])
                sy = float(mp["size_y_m"])
                ox = float(mp["origin_x"])
                oy = float(mp["origin_y"])
            except Exception as exc:
                res = None
                mism.append("场景 map 段缺字段: %s" % exc)

            if res is not None:
                # 分辨率/尺寸/origin：width=size_x/res、height=size_y/res，
                # 与 map_adapter 的 std::lround 口径一致。
                if abs(info.resolution - res) > 1e-6:
                    mism.append("resolution 场景=%g 地图=%g" % (res, info.resolution))
                exp_w = int(round(sx / res))
                exp_h = int(round(sy / res))
                if int(info.width) != exp_w:
                    mism.append("width 期望=%d 地图=%d" % (exp_w, int(info.width)))
                if int(info.height) != exp_h:
                    mism.append("height 期望=%d 地图=%d" % (exp_h, int(info.height)))
                if abs(info.origin.position.x - ox) > 1e-6:
                    mism.append("origin.x 场景=%g 地图=%g" % (ox, info.origin.position.x))
                if abs(info.origin.position.y - oy) > 1e-6:
                    mism.append("origin.y 场景=%g 地图=%g" % (oy, info.origin.position.y))

                # 每个障碍中心世界坐标落到的栅格必须为占用（100）
                width = int(info.width)
                height = int(info.height)
                for ob in (scenario.get("obstacles") or []):
                    ob_name = (ob or {}).get("name", "?")
                    try:
                        pose = (ob or {}).get("pose") or []
                        bx = float(pose[0])
                        by = float(pose[1])
                    except Exception:
                        mism.append("%s pose 缺字段" % ob_name)
                        continue
                    i = int(math.floor((bx - ox) / res))
                    j = int(math.floor((by - oy) / res))
                    if not (0 <= i < width and 0 <= j < height):
                        mism.append("%s 中心图外 (i=%d,j=%d)" % (ob_name, i, j))
                        continue
                    idx = j * width + i
                    val = data[idx] if idx < len(data) else None
                    if val != 100:
                        mism.append("%s 中心栅格非占用(idx=%d,val=%s)" % (ob_name, idx, val))

                if occupied <= 0:
                    mism.append("占用格数=%d" % occupied)

            if not version_hash:
                mism.append("/agc/map/version_hash 为空")

            if mism:
                add("地图与场景一致", False, "；".join(mism))
            else:
                add("地图与场景一致", True,
                    "resolution=%g %dx%d origin=(%g,%g) 占用格=%d 障碍=%s hash=%s"
                    % (res, int(info.width), int(info.height), ox, oy, occupied,
                       [o.get("name") for o in (scenario.get("obstacles") or [])],
                       version_hash))

    # ---------------------------------------------------------- 输出
    passed = all(c["status"] == "PASS" for c in checks)
    result = {
        "root": root,
        "seconds": seconds,
        "window_s": window,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "expected_publishers": {s["name"]: s["pub"] for s in topics},
        "checks": checks,
        "passed": passed,
    }

    print("=== AGC 接口核查 (窗口 %.1f s) ===" % window)
    print("%-6s %-42s %s" % ("结果", "核查项", "详情"))
    print("-" * 100)
    for c in checks:
        print("%-6s %-42s %s" % (c["status"], c["name"], c["detail"]))
    print("-" * 100)
    n_fail = sum(1 for c in checks if c["status"] == "FAIL")
    print("汇总: PASS=%d FAIL=%d" % (len(checks) - n_fail, n_fail))
    print("AUDIT: %s" % ("PASS" if passed else "FAIL"))

    try:
        os.makedirs(root, exist_ok=True)
        out_path = os.path.join(root, "audit_interfaces.json")
        with open(out_path, "w") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print("audit_interfaces.json 已写入: %s" % out_path)
    except OSError as exc:
        print("写 audit_interfaces.json 失败: %s" % exc)

    if not passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
