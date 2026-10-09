#!/usr/bin/env python3
"""m5_conflict_inject.py —— M5 地面写入权冲突注入（离线实验工具，非实时链路）。

在协同 run 进行期间持续以「外部写入者」身份调用 Gazebo /gazebo/set_model_state
改写本方车 agc_car 位姿，用来验证 ground_drive_adapter 的写入权核查能检出
外部 set_model_state 写入（/agc/drive/pose_authority_ok=false ->
task_manager 结算 INFRA_FAILURE/CONTROL_CONFLICT）。

用法：
    python3 m5_conflict_inject.py --model agc_car --target-x -4.0 --target-y -3.0 \
        --rate 10 --duration 8 --record <batch_dir> --tag cycle1_attempt1
（--target-x/--target-y 指定固定世界坐标遥测点（默认场景车起点，净空充裕，避免碰撞
  优先级遮蔽 CONTROL_CONFLICT）；也可用 --offset 相对当前位置偏移。）
"""
import argparse
import json
import os
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion


def main():
    ap = argparse.ArgumentParser(description="地面写入权冲突注入（外部 set_model_state 写入者）")
    ap.add_argument("--model", default="agc_car")
    ap.add_argument("--target-x", type=float, default=-4.0, help="固定遥测目标点 x(m)")
    ap.add_argument("--target-y", type=float, default=-3.0, help="固定遥测目标点 y(m)")
    ap.add_argument("--offset", type=float, default=None, help="相对当前位的横向偏移(m)；指定则覆盖固定点")
    ap.add_argument("--rate", type=float, default=10.0, help="写入频率(Hz)")
    ap.add_argument("--duration", type=float, default=8.0, help="持续秒数")
    ap.add_argument("--record", default="", help="记录注入 JSONL 的目录（batch 目录）")
    ap.add_argument("--tag", default="", help="本次注入标签")
    args = ap.parse_args()

    rospy.init_node("m5_conflict_inject", anonymous=True)
    ns = rospy.get_param("/agc/gazebo/ns", "/gazebo").rstrip("/")
    svc_name = ns + "/set_model_state"
    rospy.wait_for_service(svc_name, timeout=10.0)
    set_state = rospy.ServiceProxy(svc_name, SetModelState)

    base = {"x": 0.0, "y": 0.0, "z": 0.11}
    state_msg = None

    def on_states(msg):
        nonlocal state_msg
        state_msg = msg

    rospy.Subscriber(ns + "/model_states", ModelStates, on_states, queue_size=1)
    t0 = time.time()
    while state_msg is None and (time.time() - t0) < 5.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    if state_msg is not None:
        for i, nm in enumerate(state_msg.name):
            if nm == args.model:
                base["x"] = state_msg.pose[i].position.x
                base["y"] = state_msg.pose[i].position.y
                base["z"] = state_msg.pose[i].position.z
                break

    # 遥测点：固定世界坐标（默认车起点，净空充裕）或相对当前位置偏移
    tx = args.target_x if args.offset is None else (base["x"] + args.offset)
    ty = args.target_y if args.offset is None else base["y"]
    tz = base["z"]

    n_written = 0
    t_start = time.time()
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown() and (time.time() - t_start) < args.duration:
        st = SetModelStateRequest()
        st.model_state.model_name = args.model
        st.model_state.pose = Pose(
            position=Point(x=tx, y=ty, z=tz),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        st.model_state.reference_frame = "world"
        try:
            set_state(st)
            n_written += 1
        except rospy.ServiceException:
            pass
        rate.sleep()

    if args.record:
        try:
            os.makedirs(args.record, exist_ok=True)
            with open(os.path.join(args.record, "injections.jsonl"), "a") as f:
                f.write(json.dumps({
                    "tool": "m5_conflict_inject", "tag": args.tag, "model": args.model,
                    "target_x": tx, "target_y": ty, "offset": args.offset,
                    "rate": args.rate, "duration": args.duration,
                    "writes": n_written, "wall": time.time()}, ensure_ascii=False) + "\n")
        except OSError:
            pass
    rospy.loginfo("[m5_conflict_inject] 外部写入完成：writes=%d target=(%.2f,%.2f)", n_written, tx, ty)


if __name__ == "__main__":
    main()