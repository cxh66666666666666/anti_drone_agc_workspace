// task_manager —— 协同任务状态机与门禁（M0 接口契约 v1.1）
//
// C++ 等价实现，行为基准为 scripts/task_manager.py：话题/服务/频率/参数默认值、
// 状态转移与原因码、同拍结算优先级、成功窗口四条件均逐条对齐。
// - 发布 /agc/ground/goal（PoseStamped，world，z=0）与 /agc/task/state（AgcTaskState）。
// - 服务 /agc/task/start、/agc/task/abort。
// - 抵近跟踪成功窗口：四项条件在同一连续 5.0 s 仿真时间内成立才结算 TRACKING_SUCCESS。
// - 同拍优先级：OPERATOR_ABORT > GROUND_COLLISION > 基础设施失败 > TRACKING_SUCCESS > 任务超时。
// - start 门禁执行顺序：场景复位（第 7 项）排在 odom 复核（第 3 项）之前。复位会触发
//   /agc/air/recalibrate，重标定无条件清空 odom_valid 并在下一帧 MAVROS 才恢复，
//   先判 odom 再复位会让门禁自己把 odom 打无效（自伤竞态，M1 修复）。
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/master.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <air_ground_core/AgcTaskState.h>
#include <air_ground_core/TargetTrack.h>
#include <air_ground_core/TargetTrackArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "agc/agc_common.hpp"

namespace {

// 可选值：替代 Python 的 None（仿真时刻可能合法为 0，故不用 NaN 哨兵）
struct Opt {
  bool has = false;
  double v = 0.0;
  void set(double x) { has = true; v = x; }
  void clear() { has = false; }
};

bool endsWith(const std::string& s, const std::string& suf) {
  return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

// 节点名最后一段：对齐 Python n.strip("/").split("/")[-1]
std::string lastSegment(const std::string& full) {
  size_t b = 0, e = full.size();
  while (b < e && full[b] == '/') ++b;
  while (e > b && full[e - 1] == '/') --e;
  std::string s = full.substr(b, e - b);
  size_t p = s.find_last_of('/');
  return p == std::string::npos ? s : s.substr(p + 1);
}

struct CarPose {
  bool has = false;
  double x = 0, y = 0, yaw = 0;
};
struct TargetXY {
  bool has = false;
  double x = 0, y = 0;
};

// start 门禁 7 项结果（任一不过 -> INFRA_FAILURE / STARTUP_FAILURE）
struct Gate {
  bool single_world = false, clock = false, odom = false, targets_map = false;
  bool drive_backend = false, air_handover = false, scenario_reset = false;
  double odom_wait_s = 0.0;       // 复位后复核 odom 的等待时长（0 表示无需等待）
  std::string odom_probe = "{}";  // 复位前/后的 odom 现场，作为自伤竞态的证据
  bool all() const {
    return single_world && clock && odom && targets_map && drive_backend &&
           air_handover && scenario_reset;
  }
  std::string json() const {
    agc::JsonObj inner;
    inner.bv("single_world", single_world).bv("clock", clock).bv("odom", odom)
        .bv("targets_map", targets_map).bv("drive_backend", drive_backend)
        .bv("air_handover", air_handover).bv("scenario_reset", scenario_reset);
    agc::JsonObj o;
    o.raw("gate_failures", inner.dump());
    o.n("odom_wait_s", odom_wait_s);
    o.raw("odom_probe", odom_probe);
    return o.dump();
  }
  std::string text() const {
    return "single_world=" + std::string(single_world ? "1" : "0") +
           ", clock=" + (clock ? "1" : "0") + ", odom=" + (odom ? "1" : "0") +
           ", targets_map=" + (targets_map ? "1" : "0") +
           ", drive_backend=" + (drive_backend ? "1" : "0") +
           ", air_handover=" + (air_handover ? "1" : "0") +
           ", scenario_reset=" + (scenario_reset ? "1" : "0");
  }
};

// 同拍候选结局：prio 越小越优先，sub 为同类子序（对应 Python 的 (rank, subrank)）
struct Cand {
  int prio;
  int sub;
  int32_t state;
  std::string reason;
  std::string detail;
  bool operator<(const Cand& o) const {
    return prio != o.prio ? prio < o.prio : sub < o.sub;
  }
};

}  // namespace

class TaskManager {
 public:
  TaskManager() : nh_() {
    // 阈值（参数名与默认值对齐 Python）
    acquire_streak_s_ = agc::pgetD("/agc/success/acquire_streak_s", 1.0);
    approach_radius_ = agc::pgetD("/agc/success/approach_radius_m", 3.0);
    hold_window_s_ = agc::pgetD("/agc/success/hold_window_s", 5.0);
    confidence_min_ = agc::pgetD("/agc/success/confidence_min", 0.5);
    target_max_age_ = agc::pgetD("/agc/freshness/target_max_age_s", 0.5);
    odom_max_age_ = agc::pgetD("/agc/freshness/odom_max_age_s", 0.5);
    path_max_age_ = agc::pgetD("/agc/freshness/path_max_age_s", 2.0);
    goal_max_age_ = agc::pgetD("/agc/freshness/goal_max_age_s", 1.0);
    wall_stall_s_ = agc::pgetD("/agc/freshness/wall_clock_stall_s", 2.0);
    protection_m_ = agc::pgetD("/agc/clearance/protection_m", 0.80);
    emergency_m_ = agc::pgetD("/agc/clearance/emergency_m", 0.35);
    car_radius_ = agc::pgetD("/agc/clearance/car_radius_m", 0.30);
    search_timeout_ = agc::pgetD("/agc/limits/search_timeout_s", 60.0);
    approach_timeout_ = agc::pgetD("/agc/limits/approach_timeout_s", 60.0);
    mission_timeout_ = agc::pgetD("/agc/limits/mission_timeout_s", 120.0);
    lost_tol_ = agc::pgetD("/agc/limits/target_lost_tolerance_s", 2.0);
    plan_retry_max_ = agc::pgetI("/agc/limits/plan_retry_max", 3);
    replan_period_ = agc::pgetD("/agc/planner/replan_period_s", 2.0);
    goal_hz_ = agc::pgetD("/agc/rates/goal_hz", 10.0);
    state_hz_ = agc::pgetD("/agc/rates/state_hz", 2.0);
    // 复位会触发空中重标定并清空 odom_valid，门禁在复位后复核 odom 的等待上限
    odom_recheck_timeout_ = agc::pgetD("/agc/start/odom_recheck_timeout_s", 3.0);

    car_model_ = agc::pgetStr("/agc/gazebo/car_model", "agc_car");
    drone_model_ = agc::pgetStr("/agc/gazebo/drone_model", "iris");
    target_id_ = agc::pgetStr("/agc/target/id", "intruder_uav");
    scenario_file_ = agc::cfgS("scenario_file", "");
    obstacles_ = agc::loadScenario(scenario_file_).obstacles;

    // 输入缓存
    targets_seen_ = false;
    has_target_ = false;
    map_seen_ = false;
    ground_odom_.reset();
    air_odom_.reset();
    path_msg_.reset();
    model_states_seen_ = false;

    // 订阅 / 发布 / 服务
    sub_targets_ = nh_.subscribe("/agc/targets", 1, &TaskManager::targetsCb, this);
    sub_map_ = nh_.subscribe("/agc/map", 1, &TaskManager::mapCb, this);
    sub_ground_odom_ = nh_.subscribe("/agc/ground/odom", 1, &TaskManager::groundOdomCb, this);
    sub_air_odom_ = nh_.subscribe("/agc/air/odom", 1, &TaskManager::airOdomCb, this);
    sub_path_ = nh_.subscribe("/agc/ground/path", 1, &TaskManager::pathCb, this);
    sub_models_ = nh_.subscribe(gzNs() + "/model_states", 1, &TaskManager::modelsCb, this);

    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/agc/ground/goal", 1);
    pub_state_ = nh_.advertise<air_ground_core::AgcTaskState>("/agc/task/state", 1);
    srv_start_ = nh_.advertiseService("/agc/task/start", &TaskManager::handleStart, this);
    srv_abort_ = nh_.advertiseService("/agc/task/abort", &TaskManager::handleAbort, this);
    reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/agc/scenario/reset");
    nh_.setParam("/agc/commander/force_stop", false);

    timer_ = nh_.createTimer(ros::Duration(0.05), &TaskManager::tick, this);  // 20 Hz 内部评估
    ROS_INFO("task_manager 就绪：状态=IDLE，等待 /agc/task/start");
  }

 private:
  // ------------------------------------------------------------ 工具
  static std::string gzNs() {
    std::string ns = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
    if (ns.empty()) return "";
    while (!ns.empty() && ns.back() == '/') ns.pop_back();
    return (!ns.empty() && ns.front() == '/') ? ns : "/" + ns;
  }

  // 当前 ROS 节点名；失败返回 false（对齐 Python node_names 返回 None）
  bool nodeNames(std::vector<std::string>& out) {
    out.clear();
    // master 查询在 master 不可达时会抛 XmlRpc 异常，必须兜住（否则节点会被 abort）
    try {
      return ros::master::getNodes(out);
    } catch (const std::exception& e) {
      ROS_DEBUG("nodeNames: master 查询异常: %s", e.what());
      out.clear();
      return false;
    } catch (...) {
      out.clear();
      return false;
    }
  }

  // 话题发布者节点名；ok=false 表示查询失败（对齐 Python 返回 None）
  std::vector<std::string> publishersOf(const std::string& topic, bool& ok) {
    std::vector<std::string> out;
    ok = false;
    try {
      XmlRpc::XmlRpcValue args, response, payload;
      args[0] = ros::this_node::getName();
      if (!ros::master::execute("getSystemState", args, response, payload, false)) return out;
      if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1) return out;
      XmlRpc::XmlRpcValue pubs = payload[0];
      if (pubs.getType() != XmlRpc::XmlRpcValue::TypeArray) return out;
      ok = true;
      for (int i = 0; i < pubs.size(); ++i) {
        XmlRpc::XmlRpcValue entry = pubs[i];
        if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) continue;
        if (entry[0].getType() != XmlRpc::XmlRpcValue::TypeString) continue;
        if (std::string(entry[0]) != topic) continue;
        XmlRpc::XmlRpcValue nodes = entry[1];
        if (nodes.getType() != XmlRpc::XmlRpcValue::TypeArray) break;
        for (int j = 0; j < nodes.size(); ++j) {
          if (nodes[j].getType() == XmlRpc::XmlRpcValue::TypeString)
            out.push_back(std::string(nodes[j]));
        }
        break;
      }
    } catch (const std::exception& e) {
      ROS_DEBUG("publishersOf: master 查询异常(%s): %s", topic.c_str(), e.what());
      out.clear();
      ok = false;
    } catch (...) {
      out.clear();
      ok = false;
    }
    return out;
  }

  // ------------------------------------------------------------ 回调
  void targetsCb(const air_ground_core::TargetTrackArray::ConstPtr& msg) {
    targets_seen_ = true;
    has_target_ = false;
    for (const auto& tr : msg->tracks) {
      if (tr.target_id == target_id_) {
        target_ = tr;
        has_target_ = true;
        break;
      }
    }
    if (!has_target_ && !msg->tracks.empty()) {
      target_ = msg->tracks[0];
      has_target_ = true;
    }
  }
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr&) { map_seen_ = true; }
  void groundOdomCb(const nav_msgs::Odometry::ConstPtr& msg) { ground_odom_ = msg; }
  void airOdomCb(const nav_msgs::Odometry::ConstPtr& msg) { air_odom_ = msg; }
  void pathCb(const nav_msgs::Path::ConstPtr& msg) { path_msg_ = msg; }
  void modelsCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    model_states_seen_ = true;
    model_names_ = msg->name;
  }

  // ------------------------------------------------------------ 测量
  CarPose carPose() const {
    CarPose c;
    if (!ground_odom_) return c;
    const geometry_msgs::Pose& p = ground_odom_->pose.pose;
    c.has = true;
    c.x = p.position.x;
    c.y = p.position.y;
    c.yaw = agc::yawFromQuat(p.orientation);
    return c;
  }
  TargetXY targetXY() const {
    TargetXY t;
    if (!has_target_) return t;
    const geometry_msgs::Point& p = target_.pose.pose.position;
    t.has = true;
    t.x = p.x;
    t.y = p.y;
    return t;
  }
  double targetObsAge() const {
    if (!has_target_) return agc::INF;
    return agc::stampAge(target_.header.stamp);
  }
  // 有效观测：存在且年龄达标且置信度达标
  bool targetValid() const {
    if (!has_target_) return false;
    return targetObsAge() <= target_max_age_ &&
           static_cast<double>(target_.confidence) >= confidence_min_;
  }
  double carClearance(double x, double y) const { return agc::minClearance(x, y, obstacles_); }

  void clockWatch(double t) {
    const double wall = agc::wallNow();
    if (has_last_wall_ && has_last_sim_) {
      const double dwall = wall - last_wall_;
      const double dsim = t - last_sim_;
      if (dwall > 0.05 && dsim > 1e-6) {
        clock_advancing_ = true;
        clock_stall_ = false;
      } else if (dwall > wall_stall_s_ && dsim <= 1e-6) {
        clock_stall_ = true;
      }
    }
    last_wall_ = wall;
    last_sim_ = t;
    has_last_wall_ = has_last_sim_ = true;
  }

  // 低频核查地面控制权（M5）：/agc/ground/cmd_vel 唯一发布者必须是 ground_commander；
  // 且任务运行期唯一写车位姿者为 ground_drive_adapter（其 /agc/drive/pose_authority_ok
  // 反映是否检测到外部 set_model_state 写入）。任一不满足 -> CONTROL_CONFLICT（基础设施失败）。
  void conflictCheck(double t) {
    if (has_last_conflict_ && (t - last_conflict_check_) < 1.0) return;
    last_conflict_check_ = t;
    has_last_conflict_ = true;
    bool ok = false;
    const std::vector<std::string> pubs = publishersOf("/agc/ground/cmd_vel", ok);
    const bool cmd_unique = ok && pubs.size() == 1;
    bool cmd_conflict = false;
    if (ok) {
      cmd_conflict = !(cmd_unique && endsWith(pubs[0], "ground_commander"));
    }
    const bool pose_auth = agc::pgetB("/agc/drive/pose_authority_ok", true);
    const int ext_writes = agc::pgetI("/agc/drive/external_write_count", 0);

    control_conflict_ = cmd_conflict || !pose_auth;
    conflict_reason_.clear();
    if (cmd_conflict) conflict_reason_ = "cmd_vel_publishers=" + std::to_string(pubs.size());
    if (!pose_auth) {
      if (!conflict_reason_.empty()) conflict_reason_ += ";";
      conflict_reason_ += "external_pose_write(count=" + std::to_string(ext_writes) + ")";
    }
    ros::param::set("/agc/ground/control_conflict", control_conflict_);
    ros::param::set("/agc/ground/control_conflict_reason", conflict_reason_);

    // 控制权台账：每次核查一条，记录当时状态、各执行口实际发布者与冲突判定
    agc::JsonObj o;
    o.n("t_sim", t).n("t_wall", agc::wallNow())
        .iv("task_state", state_).v("task_state_name", agc::stateName(state_))
        .bv("cmd_vel_unique", cmd_unique).raw("cmd_vel_publishers", agc::JsonObj::arrStr(pubs))
        .bv("pose_authority_ok", pose_auth).iv("external_write_count", ext_writes)
        .bv("control_conflict", control_conflict_).v("conflict_reason", conflict_reason_)
        .v("ground_controller", groundControllerName())
        .v("air_controller", agc::pgetStr("/agc/air/active_controller", "none"));
    agc::appendJsonl("control_authority.jsonl", o);
    if (control_conflict_)
      ROS_WARN_THROTTLE(2.0, "task_manager: 地面控制冲突: %s", conflict_reason_.c_str());
  }

  // 当前阶段的名义地面控制者（唯一执行口为 ground_commander）
  std::string groundControllerName() const {
    return (state_ == agc::ST_GROUND_APPROACH || state_ == agc::ST_TRACKING_SUCCESS)
               ? "ground_commander"
               : "none";
  }

  // ------------------------------------------------------------ start 门禁
  // odom 实效判定：valid 标志 + 年龄（契约「≥20 Hz，年龄 ≤0.5 s」）
  bool odomOk() const {
    const bool g_valid = agc::pgetB("/agc/ground/odom_valid", false);
    const bool a_valid = agc::pgetB("/agc/air/odom_valid", false);
    const double g_age = ground_odom_ ? agc::stampAge(ground_odom_->header.stamp) : agc::INF;
    const double a_age = air_odom_ ? agc::stampAge(air_odom_->header.stamp) : agc::INF;
    return g_valid && a_valid && g_age <= odom_max_age_ && a_age <= odom_max_age_;
  }

  // odom 现场快照（证据）：无消息时年龄记 -1
  std::string odomProbeJson() const {
    agc::JsonObj o;
    o.bv("ground_valid", agc::pgetB("/agc/ground/odom_valid", false))
        .bv("air_valid", agc::pgetB("/agc/air/odom_valid", false))
        .n("ground_age_s", ground_odom_ ? agc::stampAge(ground_odom_->header.stamp) : -1.0)
        .n("air_age_s", air_odom_ ? agc::stampAge(air_odom_->header.stamp) : -1.0);
    return o.dump();
  }

  // 复位后复核 odom：重标定一般已在复位服务内完成，此时立即通过；
  // 尚未恢复则有限等待，超时仍不通过按门禁失败处理（诚实失败，不再产生自伤 run）。
  bool waitOdomRestored(double& waited_s, std::string& first_fail) {
    waited_s = 0.0;
    first_fail.clear();
    if (odomOk()) return true;
    // M1 遗留项 3：首次复核失败的即时快照，用于区分「空中重标定清空 air_valid」
    // 与「地面 odom 瞬态」，而非只看复位前/后两拍。
    first_fail = odomProbeJson();
    const double t0 = agc::wallNow();
    while (ros::ok() && (agc::wallNow() - t0) < odom_recheck_timeout_) {
      ros::WallDuration(0.05).sleep();
      // 必须 spinOnce：阻塞在服务回调里会让本地 odom 缓存停止更新、年龄虚增
      ros::spinOnce();
      if (odomOk()) break;
    }
    waited_s = agc::wallNow() - t0;
    return odomOk();
  }

  Gate runGate() {
    Gate g;
    // 1) 单 world：模型状态已收到且同时有 iris 与 agc_car，且无 ground_gazebo 节点
    std::vector<std::string> names;
    const bool names_ok = nodeNames(names);
    bool has_gw = false;
    for (const auto& n : names)
      if (lastSegment(n) == "ground_gazebo") has_gw = true;
    const bool has_drone =
        std::find(model_names_.begin(), model_names_.end(), drone_model_) != model_names_.end();
    const bool has_car =
        std::find(model_names_.begin(), model_names_.end(), car_model_) != model_names_.end();
    g.single_world = model_states_seen_ && has_drone && has_car && names_ok && !has_gw;
    // 2) 时钟：墙钟推进时仿真时钟也在推进
    const double t = agc::simNow().toSec();
    g.clock = clock_advancing_ && t > 0.0;

    // 复位前留证：此刻门禁尚未被自伤
    agc::JsonObj probe;
    probe.raw("before", odomProbeJson());

    // 7) 场景复位：服务返回 success 且 last_result=SUCCESS
    //    此项必须排在第 3 项 odom 之前：复位内部会调用 /agc/air/recalibrate，
    //    重标定无条件清空 odom_valid 并等下一帧 MAVROS 才恢复；若先判 odom 再复位，
    //    会形成「门禁读到有效 -> 复位清掉 -> 同拍 evaluate 判 ODOM_INVALID」的自伤竞态
    //    （M1 修复前批次 2/30 复现）。
    bool reset_ok = false;
    try {
      if (agc::serviceReady("/agc/scenario/reset", 3.0)) {
        std_srvs::Trigger srv;
        if (reset_client_.call(srv))
          reset_ok = srv.response.success &&
                     agc::pgetStr("/agc/reset/last_result", "") == "SUCCESS";
      } else {
        ROS_WARN("task_manager: 复位服务不可用");
      }
    } catch (const std::exception& e) {
      ROS_WARN("task_manager: 调用复位服务失败：%s", e.what());
    }
    g.scenario_reset = reset_ok;

    // 3) TF/odom：复位后复核，等重标定恢复有效
    std::string odom_first_fail;
    g.odom = waitOdomRestored(g.odom_wait_s, odom_first_fail);
    probe.raw("after", odomProbeJson());
    probe.raw("first_fail", odom_first_fail.empty() ? std::string("null") : odom_first_fail);
    g.odom_probe = probe.dump();
    if (g.odom_wait_s > 0.0)
      ROS_INFO("task_manager: 复位后 odom 等待 %.3f s 恢复，结果=%d", g.odom_wait_s,
               g.odom ? 1 : 0);

    // 4) 目标与地图源
    const std::string mhash = agc::pgetStr("/agc/map/version_hash", "");
    g.targets_map = targets_seen_ && map_seen_ && agc::pgetB("/agc/map/valid", false) &&
                    !mhash.empty();
    // 5) 地面后端
    const std::string backend = agc::pgetStr("/agc/drive/backend", "");
    g.drive_backend = agc::pgetB("/agc/drive/backend_ok", false) && backend == "kinematic_sim";
    // 6) 空中起飞/交接
    g.air_handover = agc::pgetB("/agc/air/handover_ok", false);
    return g;
  }

  // ------------------------------------------------------------ 服务
  bool handleStart(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (agc::isTerminal(state_)) {
      const int epoch = agc::pgetI("/agc/reset/epoch", 0);
      // 终态只能由下一次 run 的复位退出：epoch 未推进则拒绝
      if (!has_terminal_epoch_ || epoch <= terminal_epoch_) {
        res.success = false;
        res.message = "terminal state requires scenario reset";
        return true;
      }
      // M1 遗留项 1：新 attempt 起点显式回到 IDLE 并广播。
      // 门禁失败时上一次终态与本次终态可能同码（如连续 STARTUP_FAILURE），run 日志的
      // 同码终态去重会把本次 attempt 并入上一个 run；先发 IDLE 明确切开两次终态，
      // 使 run 目录与启动快照逐 attempt 一一对应。
      transition(agc::ST_IDLE, "", "{}");
      publishState(agc::simNow().toSec(), true);
    } else if (state_ != agc::ST_IDLE) {
      res.success = false;
      res.message = "task running: " + agc::stateName(state_);
      return true;
    }

    const Gate gate = runGate();
    // 门禁明细落盘：供 agc_run_logger 写每 run 启动状态（无论通过与否都写）
    {
      agc::JsonObj o;
      o.n("t_sim", agc::simNow().toSec()).n("t_wall", agc::wallNow());
      o.bv("passed", gate.all());
      o.raw("gate", gate.json());
      nh_.setParam("/agc/start/last_gate_json", o.dump());
    }
    if (!gate.all()) {
      ROS_ERROR("task_manager: start 门禁未通过 %s", gate.text().c_str());
      transition(agc::ST_INFRA_FAILURE, "STARTUP_FAILURE", gate.json());
      res.success = false;
      res.message = "STARTUP_FAILURE";
      return true;
    }

    const double t = agc::simNow().toSec();
    active_ = true;
    start_sim_.set(t);
    search_start_.set(t);
    acquired_start_.clear();
    approach_start_.clear();
    acquire_start_.clear();
    hold_start_.clear();
    had_target_ = false;
    last_valid_obs_sim_.clear();
    tracking_milestone_met_ = false;
    abort_requested_ = false;
    // M5：清除规划不可达与地面控制冲突的跨阶段残留（M3 记录：IDLE 段重规划一旦置位
    // 该标志，进入 GROUND_APPROACH 会立即结算 PLAN_UNREACHABLE，属跨阶段污染）
    nh_.setParam("/agc/planner/unreachable", false);
    control_conflict_ = false;
    conflict_reason_.clear();
    nh_.setParam("/agc/ground/control_conflict", false);
    nh_.setParam("/agc/ground/control_conflict_reason", "");
    nh_.setParam("/agc/commander/force_stop", false);
    transition(agc::ST_AIR_SEARCH, "", gate.json());
    res.success = true;
    res.message = "AIR_SEARCH";
    return true;
  }

  bool handleAbort(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (agc::isTerminal(state_)) {
      res.success = false;
      res.message = "already terminal";
      return true;
    }
    abort_requested_ = true;  // 同拍以最高优先级结算
    res.success = true;
    res.message = "abort requested";
    return true;
  }

  // ------------------------------------------------------------ 状态转移
  void transition(int32_t new_state, const std::string& reason, const std::string& detail_json) {
    const int32_t old = state_;
    state_ = new_state;
    reason_code_ = reason;
    {
      agc::JsonObj o;
      o.n("t_sim", agc::simNow().toSec())
          .n("t_wall", agc::wallNow())
          .v("from", agc::stateName(old))
          .v("to", agc::stateName(new_state))
          .iv("state", new_state)
          .v("reason", reason_code_)
          .raw("detail", detail_json.empty() ? std::string("{}") : detail_json);
      agc::appendJsonl("state_events.jsonl", o);
    }
    ROS_INFO("task_manager: %s -> %s (%s)", agc::stateName(old).c_str(),
             agc::stateName(new_state).c_str(), reason_code_.c_str());
    if (new_state == agc::ST_TARGET_ACQUIRED) {
      had_target_ = true;
      acquired_start_.set(agc::simNow().toSec());
    } else if (new_state == agc::ST_GROUND_APPROACH) {
      approach_start_.set(agc::simNow().toSec());
      hold_start_.clear();
    } else if (new_state == agc::ST_TRACKING_SUCCESS) {
      tracking_milestone_met_ = true;
    }
    if (agc::isTerminal(new_state)) {
      active_ = false;
      terminal_epoch_ = agc::pgetI("/agc/reset/epoch", 0);
      has_terminal_epoch_ = true;
      nh_.setParam("/agc/commander/force_stop", true);  // 终态立即停车
      publishState(agc::simNow().toSec(), true);
    }
  }

  void logEvent(const std::string& name, const std::string& detail_json) {
    agc::JsonObj o;
    o.n("t_sim", agc::simNow().toSec())
        .n("t_wall", agc::wallNow())
        .v("event", name)
        .iv("state", state_)
        .v("state_name", agc::stateName(state_))
        .v("reason", reason_code_)
        .raw("detail", detail_json.empty() ? std::string("{}") : detail_json);
    agc::appendJsonl("state_events.jsonl", o);
  }

  // ------------------------------------------------------------ 评估
  std::string infraReason() {
    if (clock_stall_) return "CLOCK_STALL";
    const bool g_valid = agc::pgetB("/agc/ground/odom_valid", false);
    const bool a_valid = agc::pgetB("/agc/air/odom_valid", false);
    const double g_age = ground_odom_ ? agc::stampAge(ground_odom_->header.stamp) : agc::INF;
    const double a_age = air_odom_ ? agc::stampAge(air_odom_->header.stamp) : agc::INF;
    if (!g_valid || !a_valid || g_age > odom_max_age_ || a_age > odom_max_age_)
      return "ODOM_INVALID";
    if (!agc::pgetB("/agc/drive/backend_ok", true)) return "DRIVE_BACKEND_FAILURE";
    if (control_conflict_) return "CONTROL_CONFLICT";
    return "";
  }

  bool approachReady() const {
    if (!targetValid()) return false;
    if (!map_seen_ || !agc::pgetB("/agc/map/valid", false)) return false;
    if (!ground_odom_ || agc::stampAge(ground_odom_->header.stamp) > odom_max_age_) return false;
    if (!path_msg_ || path_msg_->poses.empty()) return false;
    if (agc::stampAge(path_msg_->header.stamp) > path_max_age_) return false;
    return true;
  }

  // 返回 true 时填 out（reason/subrank/detail_json）
  bool missionFailure(double t, std::string& reason, int& subrank, std::string& detail) {
    if (!active_) return false;
    if (start_sim_.has && (t - start_sim_.v) > mission_timeout_) {
      detail = agc::JsonObj().n("elapsed_s", t - start_sim_.v).dump();
      reason = "MISSION_TIMEOUT";
      subrank = 4;
      return true;
    }
    if (had_target_ && last_valid_obs_sim_.has &&
        (t - last_valid_obs_sim_.v) > lost_tol_) {
      detail = agc::JsonObj().n("lost_s", t - last_valid_obs_sim_.v).dump();
      reason = "TARGET_LOST";
      subrank = 0;
      return true;
    }
    if (agc::pgetB("/agc/planner/unreachable", false)) {
      detail = "{}";
      reason = "PLAN_UNREACHABLE";
      subrank = 1;
      return true;
    }
    if (state_ == agc::ST_AIR_SEARCH && search_start_.has &&
        (t - search_start_.v) > search_timeout_) {
      detail = agc::JsonObj().n("elapsed_s", t - search_start_.v).dump();
      reason = "SEARCH_TIMEOUT";
      subrank = 3;
      return true;
    }
    if (state_ == agc::ST_TARGET_ACQUIRED && acquired_start_.has) {
      // 有界等待：规划重试上限与抵近超时的较小者，避免无限等待
      const double bound = std::min(static_cast<double>(plan_retry_max_) * replan_period_,
                                    approach_timeout_);
      if ((t - acquired_start_.v) > bound) {
        agc::JsonObj o;
        o.n("waited_s", t - acquired_start_.v).n("bound_s", bound);
        detail = o.dump();
        reason = "PLAN_UNREACHABLE";
        subrank = 1;
        return true;
      }
    }
    if (state_ == agc::ST_GROUND_APPROACH && approach_start_.has &&
        (t - approach_start_.v) > approach_timeout_) {
      detail = agc::JsonObj().n("elapsed_s", t - approach_start_.v).dump();
      reason = "APPROACH_TIMEOUT";
      subrank = 2;
      return true;
    }
    return false;
  }

  void recordTrackingFrame(double t, const CarPose& car, double clearance) {
    if (!car.has || !has_target_ || !ground_odom_) return;
    const geometry_msgs::Twist& tw = ground_odom_->twist.twist;
    const std::array<double, 4> q = {{ground_odom_->pose.pose.orientation.x,
                                      ground_odom_->pose.pose.orientation.y,
                                      ground_odom_->pose.pose.orientation.z,
                                      ground_odom_->pose.pose.orientation.w}};
    const std::array<double, 3> cv =
        agc::rotateVecByQuat(q, {{tw.linear.x, tw.linear.y, tw.linear.z}});
    const geometry_msgs::Twist& tv = target_.twist.twist;
    const TargetXY txy = targetXY();
    const double dist = std::hypot(car.x - txy.x, car.y - txy.y);
    const double rvx = tv.linear.x - cv[0];
    const double rvy = tv.linear.y - cv[1];
    agc::JsonObj o;
    o.n("t_sim", t)
        .raw("car_vel", agc::JsonObj::arrNum({cv[0], cv[1]}))
        .raw("target_vel", agc::JsonObj::arrNum({tv.linear.x, tv.linear.y}))
        .raw("rel_vel", agc::JsonObj::arrNum({rvx, rvy}))
        .n("rel_speed", std::hypot(rvx, rvy))
        .n("distance_m", dist)
        .n("clearance_m", clearance);
    agc::appendJsonl("tracking_frames.jsonl", o);
  }

  void evaluate(double t) {
    clockWatch(t);
    conflictCheck(t);

    const bool valid = targetValid();
    if (valid) {
      last_valid_obs_sim_.set(t);
      if (!acquire_start_.has) acquire_start_.set(t);
    } else {
      acquire_start_.clear();
    }

    const CarPose car = carPose();
    double clearance = agc::INF;
    bool collision = false;
    if (car.has) {
      clearance = carClearance(car.x, car.y);
      // 净空 = 车中心到障碍 OBB 表面的有符号距离（内部为负）；接触/几何相交判据 = clearance ≤ car_radius_m。
      collision = (clearance <= car_radius_);
      // 净空分档事件：只记事件不改变结局（碰撞档另行结算 GROUND_COLLISION）
      const std::pair<bool, bool> tier = {clearance < protection_m_, clearance <= emergency_m_};
      if (!has_last_tier_ || tier.first != last_tier_.first || tier.second != last_tier_.second) {
        if (clearance < protection_m_)
          logEvent("CLEARANCE_VIOLATION",  // 违例不叫碰撞
                   agc::JsonObj().n("clearance_m", clearance).dump());
        if (clearance <= emergency_m_)
          logEvent("EMERGENCY_TRIGGER",
                   agc::JsonObj().n("clearance_m", clearance).dump());
        last_tier_ = tier;
        has_last_tier_ = true;
      }
    }

    if (agc::isTerminal(state_)) return;

    // 收集本拍候选结局，按优先级结算
    std::vector<Cand> cands;
    if (abort_requested_) cands.push_back({1, 0, agc::ST_ABORTED, "OPERATOR_ABORT", "{}"});
    if (active_ && collision)
      cands.push_back({2, 0, agc::ST_MISSION_FAILED, "GROUND_COLLISION",
                       agc::JsonObj().n("clearance_m", clearance).dump()});
    if (active_) {
      const std::string infra = infraReason();
      if (!infra.empty()) {
        std::string detail = "{}";
        if (infra == "CONTROL_CONFLICT")
          detail = agc::JsonObj().v("reason", conflict_reason_).dump();
        cands.push_back({3, 0, agc::ST_INFRA_FAILURE, infra, detail});
      }
    }
    if (active_ && state_ == agc::ST_GROUND_APPROACH) {
      recordTrackingFrame(t, car, clearance);
      const TargetXY txy = targetXY();
      const bool c1 = txy.has && car.has &&
                      std::hypot(car.x - txy.x, car.y - txy.y) <= approach_radius_;
      const bool c2 = valid && ground_odom_ &&
                      agc::stampAge(ground_odom_->header.stamp) <= odom_max_age_;
      const bool c3 = (clearance >= protection_m_) && !collision;
      const bool c4 = true;  // 逐帧记录车/目标速度、相对速度与距离
      const bool conds = c1 && c2 && c3 && c4;
      if (conds) {
        if (!hold_start_.has) hold_start_.set(t);
      } else {
        hold_start_.clear();  // 任一中断即清零
      }
      const double held = hold_start_.has ? (t - hold_start_.v) : 0.0;
      if (conds && held >= hold_window_s_)
        cands.push_back({4, 0, agc::ST_TRACKING_SUCCESS, "",
                         agc::JsonObj().n("held_s", held).dump()});
    }
    {
      std::string mreason, mdetail;
      int msub = 0;
      if (missionFailure(t, mreason, msub, mdetail))
        cands.push_back({5, msub, agc::ST_MISSION_FAILED, mreason, mdetail});
    }

    if (!cands.empty()) {
      std::stable_sort(cands.begin(), cands.end());
      const Cand best = cands[0];
      // 其余同时发生的事件仍完整落盘
      for (size_t i = 1; i < cands.size(); ++i) {
        agc::JsonObj o;
        o.v("reason", cands[i].reason).v("state", agc::stateName(cands[i].state));
        logEvent("COINCIDENT", o.dump());
      }
      transition(best.state, best.reason, best.detail);
      return;
    }

    // 无终局：推进状态机
    if (active_) {
      if (state_ == agc::ST_AIR_SEARCH) {
        if (acquire_start_.has && (t - acquire_start_.v) >= acquire_streak_s_) {
          transition(agc::ST_TARGET_ACQUIRED, "",
                     agc::JsonObj().n("streak_s", t - acquire_start_.v).dump());
        }
      } else if (state_ == agc::ST_TARGET_ACQUIRED) {
        if (approachReady()) transition(agc::ST_GROUND_APPROACH, "", "{}");
      }
      nh_.setParam("/agc/commander/force_stop", false);  // 非终态且需运动
    }
  }

  // ------------------------------------------------------------ 发布
  // 安全接近点：目标地面投影净空不足时，沿「车->目标」方向回退到净空达标处。
  // 这是「规划出的安全接近点」，不是飞行目标的 2 m 高姿态。
  void safeApproachPoint(double tx, double ty, const CarPose& car, double& gx, double& gy) const {
    gx = tx;
    gy = ty;
    const double dx = car.x - tx, dy = car.y - ty;
    const double n = std::hypot(dx, dy);
    if (n < 1e-6) return;
    const double ux = dx / n, uy = dy / n;
    const double step = 0.1, max_back = 20.0;
    for (double tt = 0.0; tt <= max_back; tt += step) {
      const double px = tx + ux * tt, py = ty + uy * tt;
      if (carClearance(px, py) >= protection_m_) {
        gx = px;
        gy = py;
        return;
      }
    }
    gx = tx + ux * max_back;
    gy = ty + uy * max_back;
  }

  void publishGoal(double t) {
    const TargetXY txy = targetXY();
    const CarPose car = carPose();
    if (!txy.has || !car.has) return;  // 无目标不发布，goal 自然过期 -> 地面停车
    double gx = 0, gy = 0;
    safeApproachPoint(txy.x, txy.y, car, gx, gy);
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = agc::FRAME_WORLD;
    msg.header.stamp = agc::simNow();
    msg.pose.position.x = gx;
    msg.pose.position.y = gy;
    msg.pose.position.z = 0.0;
    const std::array<double, 4> q = agc::quatFromYaw(0.0);
    msg.pose.orientation.x = q[0];
    msg.pose.orientation.y = q[1];
    msg.pose.orientation.z = q[2];
    msg.pose.orientation.w = q[3];
    const bool moved =
        !has_goal_msg_ || std::hypot(gx - goal_x_, gy - goal_y_) > 0.05;
    const bool due = !last_goal_pub_.has || (t - last_goal_pub_.v) >= 1.0 / goal_hz_;
    if (moved || due) {
      pub_goal_.publish(msg);
      last_goal_pub_.set(t);
    }
    goal_x_ = gx;
    goal_y_ = gy;
    has_goal_msg_ = true;
  }

  void publishState(double t, bool force = false) {
    air_ground_core::AgcTaskState msg;
    msg.header.frame_id = agc::FRAME_WORLD;
    msg.header.stamp = agc::simNow();
    msg.state = static_cast<uint8_t>(state_);
    msg.state_name = agc::stateName(state_);
    msg.reason_code = reason_code_;
    msg.air_controller = agc::pgetStr("/agc/air/active_controller", "none");
    msg.ground_controller = groundControllerName();
    msg.target_id = target_id_;
    const CarPose car = carPose();
    const TargetXY txy = targetXY();
    if (!txy.has || !car.has) {
      msg.target_xy_distance_m = std::numeric_limits<float>::quiet_NaN();  // 无目标 -> NaN
      msg.target_obs_age_s = std::numeric_limits<float>::quiet_NaN();
    } else {
      msg.target_xy_distance_m =
          static_cast<float>(std::hypot(car.x - txy.x, car.y - txy.y));
      msg.target_obs_age_s = static_cast<float>(targetObsAge());
    }
    msg.tracking_milestone_met = tracking_milestone_met_;
    const bool due = !last_state_pub_sim_.has || (t - last_state_pub_sim_.v) >= 1.0 / state_hz_;
    const bool changed = !has_pub_state_ || state_ != last_pub_state_code_ ||
                         reason_code_ != pub_reason_code_ ||
                         tracking_milestone_met_ != pub_milestone_;
    if (force || changed || due) {
      pub_state_.publish(msg);
      last_pub_state_code_ = state_;
      pub_reason_code_ = reason_code_;
      pub_milestone_ = tracking_milestone_met_;
      has_pub_state_ = true;
      last_state_pub_sim_.set(t);
    }
  }

  // ------------------------------------------------------------ 定时器
  // 轮询 /agc/scenario_file：变化即重载 obstacles_（安全接近点几何随场景更新）
  void checkScenarioReload() {
    const std::string f = agc::cfgS("scenario_file", "");
    if (f == scenario_file_) return;
    scenario_file_ = f;
    obstacles_ = agc::loadScenario(f).obstacles;
    ROS_INFO("task_manager: 场景文件变更 %s，重载 obstacles=%zu", f.c_str(), obstacles_.size());
  }

  void tick(const ros::TimerEvent&) {
    checkScenarioReload();
    const double t = agc::simNow().toSec();
    evaluate(t);
    publishGoal(t);
    publishState(t);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_targets_, sub_map_, sub_ground_odom_, sub_air_odom_, sub_path_, sub_models_;
  ros::Publisher pub_goal_, pub_state_;
  ros::ServiceServer srv_start_, srv_abort_;
  ros::ServiceClient reset_client_;
  ros::Timer timer_;

  // 阈值
  double acquire_streak_s_, approach_radius_, hold_window_s_, confidence_min_;
  double target_max_age_, odom_max_age_, path_max_age_, goal_max_age_, wall_stall_s_;
  double protection_m_, emergency_m_, car_radius_;
  double search_timeout_, approach_timeout_, mission_timeout_, lost_tol_;
  int plan_retry_max_;
  double replan_period_, goal_hz_, state_hz_;
  double odom_recheck_timeout_ = 3.0;
  std::string car_model_, drone_model_, target_id_;
  std::string scenario_file_;  // 场景文件缓存（变化即重载 obstacles_）
  std::vector<agc::Obstacle> obstacles_;

  // 状态
  int32_t state_ = agc::ST_IDLE;
  std::string reason_code_;
  bool active_ = false;
  bool tracking_milestone_met_ = false;
  bool has_terminal_epoch_ = false;
  int terminal_epoch_ = 0;
  bool abort_requested_ = false;
  bool had_target_ = false;
  Opt start_sim_, search_start_, acquired_start_, approach_start_, acquire_start_;
  Opt hold_start_, last_valid_obs_sim_;

  // 时钟监视
  bool has_last_wall_ = false, has_last_sim_ = false;
  double last_wall_ = 0, last_sim_ = 0;
  bool clock_advancing_ = false, clock_stall_ = false;

  // 输入缓存
  bool targets_seen_ = false, has_target_ = false, map_seen_ = false;
  air_ground_core::TargetTrack target_;
  nav_msgs::Odometry::ConstPtr ground_odom_, air_odom_;
  nav_msgs::Path::ConstPtr path_msg_;
  std::vector<std::string> model_names_;
  bool model_states_seen_ = false;

  // 发布去重
  bool has_goal_msg_ = false;
  double goal_x_ = 0, goal_y_ = 0;
  Opt last_goal_pub_, last_state_pub_sim_;
  bool has_pub_state_ = false;
  int32_t last_pub_state_code_ = agc::ST_IDLE;
  std::string pub_reason_code_;
  bool pub_milestone_ = false;

  // 控制冲突 / 净空分档
  bool has_last_conflict_ = false;
  double last_conflict_check_ = 0;
  bool control_conflict_ = false;
  std::string conflict_reason_;  // M5：地面控制冲突明细（cmd_vel 发布者 / 外部写车位姿）
  bool has_last_tier_ = false;
  std::pair<bool, bool> last_tier_{false, false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_manager");
  TaskManager node;
  ros::spin();
  return 0;
}