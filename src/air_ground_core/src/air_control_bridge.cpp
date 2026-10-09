// air_control_bridge —— 空中控制权交接核查与安全 hold 验证（M0 接口契约 v1.1，C++ 实现）。
//
// 职责（Task 6）：只做「核查 + 记录」，绝不发布任何 setpoint。本协同 run 中
// lqr_tracker_node 是 /mavros/setpoint_position/local 的唯一实际发布者，bridge 不接管控制。
//   * 订阅 setpoint，记录每条消息的仿真/墙钟时间与位置，统计相邻消息最大间隔；
//     订阅 /mavros/state 记录 mode/armed/connected 与 mode 变化序列。
//   * 周期性读取 Master 的发布者集合，集合变化即视为控制权实际交接，据此写
//     /agc/air/active_controller、/agc/air/handover_ok。
//   * 安全 hold 验证：/agc/task/state 进入终态(4/5/6/8) 或目标失联后，检查后续 setpoint
//     是否保持同一固定点（发布连续、位置相对悬停点在容差内）→ /agc/air/hold_ok。
//
// 关于「沿旧参考继续飞行」：本 run 采用 trajectory_type=step，参考本身就是固定悬停点，
// 不存在「把偏移归零后继续沿旧参考飞」的风险；bridge 只验证 setpoint 是否稳定保持在
// 悬停点附近，不介入任何控制。
//
// 仿真时间未就绪（ros::Time::now()==0）时，sim 字段记 null（内部以 -1 哨兵表示），
// 靠墙钟继续工作；主循环用墙钟 sleep，避免被 /clock 阻塞。
#include "agc/agc_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

#include <ros/master.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <air_ground_core/AgcTaskState.h>

namespace {

const char* const SETPOINT_TOPIC = "/mavros/setpoint_position/local";
const char* const LQR_NODE = "lqr_tracker_node";
const char* const TAKEOFF_NODE = "drone_launch_controller";

// 墙钟（epoch）用于报告时间戳；间隔统计用单调墙钟，与 Python 的 time.time()/time.monotonic() 分工一致。
double epochNow() { return ros::WallTime::now().toSec(); }
double round6(double v) { return std::round(v * 1e6) / 1e6; }

bool isDir(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}
bool fileExists(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}
void mkdirp(const std::string& p) {
  if (p.empty() || isDir(p)) return;
  std::string cur;
  for (size_t i = 0; i < p.size(); ++i) {
    cur.push_back(p[i]);
    if (p[i] == '/' || i + 1 == p.size()) ::mkdir(cur.c_str(), 0755);
  }
}
std::string join(const std::vector<std::string>& v, const char* sep) {
  std::string s;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) s += sep;
    s += v[i];
  }
  return s;
}

// XmlRpc 值转字符串：用于 getSystemState 布局自检时「原样」打印 first entry。
std::string xvToStr(const XmlRpc::XmlRpcValue& v) {
  switch (v.getType()) {
    case XmlRpc::XmlRpcValue::TypeString:
      return static_cast<std::string>(v);
    case XmlRpc::XmlRpcValue::TypeInt:
      return std::to_string(static_cast<int>(v));
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return static_cast<bool>(v) ? "true" : "false";
    case XmlRpc::XmlRpcValue::TypeDouble: {
      char buf[40];
      std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
      return buf;
    }
    case XmlRpc::XmlRpcValue::TypeArray: {
      std::string s = "[";
      for (int i = 0; i < v.size(); ++i) {
        if (i) s += ",";
        s += xvToStr(v[i]);
      }
      return s + "]";
    }
    case XmlRpc::XmlRpcValue::TypeStruct: {
      std::string s = "{";
      bool first = true;
      for (auto it = v.begin(); it != v.end(); ++it) {
        if (!first) s += ",";
        first = false;
        s += it->first + ":" + xvToStr(it->second);
      }
      return s + "}";
    }
    default:
      return "<invalid>";
  }
}

}  // namespace

class AirControlBridge {
 public:
  AirControlBridge()
      : nh_(),
        hz_(agc::cfgD("rates/bridge_hz", 10.0)),
        gap_max_(agc::cfgD("air/setpoint_gap_max_s", 0.2)),
        hold_tol_(agc::cfgD("air/hold_position_tolerance_m", 0.75)),
        target_lost_tol_(agc::cfgD("limits/target_lost_tolerance_s", 2.0)),
        start_wall_(epochNow()) {
    sub_sp_ = nh_.subscribe(SETPOINT_TOPIC, 50, &AirControlBridge::onSetpoint, this);
    sub_state_ = nh_.subscribe("/mavros/state", 10, &AirControlBridge::onState, this);
    sub_task_ = nh_.subscribe("/agc/task/state", 10, &AirControlBridge::onTaskState, this);
  }

  void run() {
    const double period = 1.0 / std::max(hz_, 1.0);
    ROS_INFO("[air_control_bridge] 启动 bridge_hz=%.1f gap_max=%.2fs hold_tol=%.2fm", hz_,
             gap_max_, hold_tol_);
    while (ros::ok()) {
      pollPublishers();
      publishDiag();
      if (agc::wallNow() - last_report_wall_ >= 5.0) writeReport();
      ros::WallDuration(period).sleep();  // 墙钟 sleep，避免 /clock 停滞时阻塞
    }
  }

  void shutdown() {
    const std::string path = writeReport();
    ROS_INFO("[air_control_bridge] 交接报告已写入: %s", path.c_str());
  }

 private:
  // ---------------------------------------------------------- 工具
  static double simNowOpt() {  // /clock 未就绪（now()==0）返回 -1（等价 Python 的 None）
    const double t = ros::Time::now().toSec();
    return t > 0.0 ? t : -1.0;
  }
  static std::string normNode(const std::string& name) {
    size_t i = 0;
    while (i < name.size() && name[i] == '/') ++i;
    return name.substr(i);
  }

  // ---------------------------------------------------------- 回调
  void onSetpoint(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const double now_wall = agc::wallNow();  // 单调墙钟
    const double stamp = msg->header.stamp.toSec();
    const double sim_t = stamp > 0.0 ? stamp : simNowOpt();

    last_pos_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    has_last_pos_ = true;
    ++sp_count_;

    // 相邻消息间隔（含交接瞬间的空窗）
    if (has_last_sp_wall_) {
      const double gap = now_wall - last_sp_wall_;
      if (gap > max_gap_) max_gap_ = gap;
      if (hold_active_ && gap > hold_max_gap_) hold_max_gap_ = gap;
    }

    // 交接专用空窗：从「检测到 lqr_tracker_node 成为发布者」之前的最后一条 setpoint
    // 到交接后的第一条 setpoint。契约只要求交接期不留空窗，不能拿整段起飞预热期的
    // 间隔去判定交接质量。
    if (handover_gap_pending_ && has_handover_last_wall_) {
      handover_gap_ = now_wall - handover_last_wall_;
      has_handover_gap_ = true;
      handover_gap_pending_ = false;
      ROS_INFO("[air_control_bridge] 交接空窗 = %.3f s (阈值 %.3f s)", handover_gap_, gap_max_);
    }

    if (!has_first_sp_sim_ && sim_t > 0.0) {
      first_sp_sim_ = sim_t;
      has_first_sp_sim_ = true;
    }
    last_sp_sim_ = sim_t;  // 未就绪时为 -1（等价 None）
    last_sp_wall_ = now_wall;
    has_last_sp_wall_ = true;

    if (hold_active_ && has_hold_point_) {
      ++hold_samples_;
      const double dev = agc::dist3(hold_point_, last_pos_);
      if (dev > hold_max_dev_) hold_max_dev_ = dev;
    }
  }

  void onState(const mavros_msgs::State::ConstPtr& msg) {
    if (msg->mode != mavros_mode_) {
      ModeChange c;
      c.sim_time = simNowOpt();
      c.wall_time = epochNow();
      c.mode = msg->mode;
      mode_changes_.push_back(c);
      mavros_mode_ = msg->mode;
    }
    armed_ = msg->armed;
    connected_ = msg->connected;
  }

  void onTaskState(const air_ground_core::AgcTaskState::ConstPtr& msg) {
    const int state = msg->state;
    const bool terminal = agc::isTerminal(state);
    task_state_ = state;
    task_running_ = (!terminal && state != agc::ST_IDLE);  // 任务运行期=非 IDLE 非终态
    // M1 遗留项 4：attempt 起点钉住该 attempt 的 run 目录并复位 hold 验证状态，
    // 让 air_handover.json 落在本 attempt 的目录，而不是被后续 attempt 覆盖。
    if (!terminal) pinAttemptRunDir();
    bool lost = false;
    const double age = msg->target_obs_age_s;
    if (!terminal && !std::isnan(age) && age > target_lost_tol_ && !msg->target_id.empty()) {
      lost = true;
    }
    if (!hold_active_ && (terminal || lost)) {
      triggerHold(terminal ? ("terminal_state_" + std::to_string(state)) : "target_lost");
    }
  }

  // attempt 起点：把交接报告的写入目标钉在当前 run 目录；目录变化即视为新 attempt
  void pinAttemptRunDir() {
    const std::string run_dir = agc::cfgS("log/run_dir", "");
    if (run_dir.empty() || !isDir(run_dir) || run_dir == pinned_run_dir_) return;
    pinned_run_dir_ = run_dir;
    resetHoldForAttempt();  // 新 attempt 的 hold 验证重新计时
    ROS_INFO("[air_control_bridge] 交接报告目标目录切换为 %s", pinned_run_dir_.c_str());
  }

  void resetHoldForAttempt() {
    hold_active_ = false;
    has_hold_point_ = false;
    hold_samples_ = 0;
    hold_max_dev_ = 0.0;
    hold_max_gap_ = 0.0;
    hold_trigger_sim_ = 0.0;
  }

  void triggerHold(const std::string& reason) {
    hold_active_ = true;
    hold_trigger_sim_ = simNowOpt();
    hold_trigger_wall_ = agc::wallNow();
    hold_point_ = last_pos_;
    has_hold_point_ = has_last_pos_;
    hold_samples_ = 0;
    hold_max_dev_ = 0.0;
    hold_max_gap_ = 0.0;
    notes_.push_back("hold 验证触发(" + reason + ")，悬停点=[" +
                     (has_hold_point_ ? (std::to_string(hold_point_[0]) + ", " +
                                         std::to_string(hold_point_[1]) + ", " +
                                         std::to_string(hold_point_[2]))
                                      : "None") +
                     "]");
    ROS_INFO("[air_control_bridge] 开始安全 hold 验证: %s", reason.c_str());
    writeReport();  // 立即把触发快照写入本 attempt 的 run 目录
  }

  // ---------------------------------------------------------- 发布者集合 / 交接判定
  bool fetchSetpointPublishers(std::set<std::string>* nodes) {
    // 注意：Noetic 的 ros/master.h 并未声明 ros::master::getSystemState(V_PV_string&,...)，
    // 因此这里直接走 Master XMLRPC：execute("getSystemState", [caller_id], result, payload)。
    // payload[0] 为发布者表，标准布局每项形如 [topic, [node1, node2, ...]]：
    //   e[0] = topic（字符串），e[1] = 节点名数组（嵌套数组）。
    // 同时也兼容扁平布局 [node..., topic]。master 不可达时 execute 会抛 XmlRpc 异常，必须兜住。
    nodes->clear();
    try {
      XmlRpc::XmlRpcValue args, result, payload;
      args[0] = ros::this_node::getName();
      if (!ros::master::execute("getSystemState", args, result, payload, false)) return false;
      if (!result.valid() || result.getType() != XmlRpc::XmlRpcValue::TypeArray ||
          result.size() < 1 || static_cast<int>(result[0]) != 1) {
        return false;
      }
      if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1) return false;
      XmlRpc::XmlRpcValue& pubs = payload[0];
      if (pubs.getType() != XmlRpc::XmlRpcValue::TypeArray) return false;

      // 运行时自检：第一次拉到系统状态时把 first entry 原样打印一次，
      // 确认 inner [0]=topic、[1]=节点名数组的布局；若不同则走自适应分支。
      if (!sysstate_logged_) {
        sysstate_logged_ = true;
        ROS_INFO("[air_control_bridge] getSystemState 布局自检: payload.size=%d pubs.size=%d%s",
                 payload.size(), pubs.size(),
                 pubs.size() > 0 ? (" first_entry=" + xvToStr(pubs[0])).c_str() : " (pubs 为空)");
      }

      for (int k = 0; k < pubs.size(); ++k) {
        XmlRpc::XmlRpcValue& e = pubs[k];
        if (e.getType() != XmlRpc::XmlRpcValue::TypeArray || e.size() < 1) continue;
        std::string topic;
        int node_idx = -1;
        if (e[0].getType() == XmlRpc::XmlRpcValue::TypeString) {
          topic = static_cast<std::string>(e[0]);  // 标准布局：e[0]=topic
        } else if (e.size() >= 2 && e[1].getType() == XmlRpc::XmlRpcValue::TypeString) {
          topic = static_cast<std::string>(e[1]);  // 自适应扁平布局：[nodes..., topic]
          node_idx = 0;
        } else {
          continue;
        }
        if (topic != SETPOINT_TOPIC) continue;

        // 标准布局：节点名在嵌套数组 e[1] 里
        if (node_idx != 0 && e.size() >= 2 &&
            e[1].getType() == XmlRpc::XmlRpcValue::TypeArray) {
          XmlRpc::XmlRpcValue& ns = e[1];
          for (int j = 0; j < ns.size(); ++j) {
            if (ns[j].getType() == XmlRpc::XmlRpcValue::TypeString)
              nodes->insert(normNode(static_cast<std::string>(ns[j])));
          }
          continue;
        }
        // 扁平布局：节点名平铺在 e[node_idx..]
        for (int i = (node_idx < 0 ? 1 : node_idx); i < e.size(); ++i) {
          if (e[i].getType() != XmlRpc::XmlRpcValue::TypeString) continue;
          nodes->insert(normNode(static_cast<std::string>(e[i])));
        }
      }
    } catch (const std::exception& e) {
      ROS_DEBUG("[air_control_bridge] getSystemState 异常: %s", e.what());
      nodes->clear();
      return false;
    } catch (...) {
      nodes->clear();
      return false;
    }
    return true;
  }

  void pollPublishers() {
    std::set<std::string> nodes;
    if (!fetchSetpointPublishers(&nodes)) {
      ROS_WARN_THROTTLE(5.0, "[air_control_bridge] getSystemState 失败");
      return;
    }
    if (nodes == pub_nodes_) return;

    const std::set<std::string> prev = pub_nodes_;
    pub_nodes_ = nodes;
    ROS_INFO("[air_control_bridge] setpoint 发布者集合变化: [%s] -> [%s]",
             join(sortVec(prev), ", ").c_str(), join(sortVec(nodes), ", ").c_str());

    // 实际交接：必须**先看到**起飞控制器独占该话题，再看到 lqr_tracker_node 新出现。
    // 否则 master 里若残留上一次 run 的注册（或首轮轮询就同时看到两者），
    // 会在启动瞬间把上一 run 的注册误判成本 run 的交接。
    if (!handover_detected_ && nodes.count(LQR_NODE) && !prev.count(LQR_NODE) &&
        prev.count(TAKEOFF_NODE) && has_last_sp_wall_) {
      handover_detected_ = true;
      handover_sim_time_ = simNowOpt();
      handover_wall_time_ = epochNow();
      publishers_before_ = sortVec(prev);
      publishers_after_ = sortVec(nodes);
      handover_last_wall_ = last_sp_wall_;  // 交接前最后一条 setpoint 的墙钟时间
      has_handover_last_wall_ = true;
      handover_gap_pending_ = true;
      notes_.push_back("lqr_tracker_node 出现在 setpoint 发布者集合，判定控制权交接");
      ROS_INFO("[air_control_bridge] 检测到空中控制权交接 -> lqr_tracker_node");
    }
  }

  static std::vector<std::string> sortVec(const std::set<std::string>& s) {
    return std::vector<std::string>(s.begin(), s.end());
  }

  std::string activeController() const {
    // 交接判定后控制权持久归 lqr_tracker_node：起飞节点即便已释放控制权仍会保留
    // setpoint advertiser（无法仅凭 advertiser 列表区分），一旦判定交接即固定，
    // 避免停机/列表抖动把控制者退回起飞节点（M5：唯一控制者可复盘）。
    if (handover_detected_) return LQR_NODE;
    if (pub_nodes_.count(TAKEOFF_NODE)) return "takeoff_controller";
    return "none";
  }

  // 空中控制权冲突（M5）：任务运行期出现 setpoint 空窗（无实际控制者），
  // 或任务已到 TARGET_ACQUIRED 及以上却从未检测到交接。
  bool airConflict() const {
    if (!task_running_) return false;
    const bool no_setpoint =
        has_last_sp_wall_ && (agc::wallNow() - last_sp_wall_) > gap_max_;
    const bool no_handover =
        !handover_detected_ && task_state_ >= agc::ST_TARGET_ACQUIRED;
    return no_setpoint || no_handover;
  }

  // 控制权时间线：控制者/交接/冲突/任务状态变化时写一条
  void logAuthority() {
    const std::string key = activeController() + "|" + (handover_detected_ ? "1" : "0") + "|" +
                            (airConflict() ? "1" : "0") + "|" + std::to_string(task_state_);
    if (has_authority_log_ && key == last_auth_key_) return;
    has_authority_log_ = true;
    last_auth_key_ = key;
    agc::JsonObj o;
    o.n("t_sim", simNowOpt()).n("t_wall", agc::wallNow())
        .v("event", "authority_change").v("active_controller", activeController())
        .v("air_controller_owner", activeController())
        .bv("handover_detected", handover_detected_).bv("control_conflict", airConflict())
        .iv("task_state", task_state_).bv("task_running", task_running_)
        .bv("mavros_offboard", mavros_mode_ == "OFFBOARD")
        .raw("setpoint_publishers", agc::JsonObj::arrStr(sortVec(pub_nodes_)))
        .n("setpoint_gap_s", has_last_sp_wall_ ? (agc::wallNow() - last_sp_wall_) : -1.0);
    agc::appendJsonl("air_control_authority.jsonl", o);
  }

  // 交接已发生 且 交接空窗 < 阈值 且 mode==OFFBOARD
  bool handoverOk() const {
    return handover_detected_ && has_handover_gap_ && handover_gap_ < gap_max_ &&
           mavros_mode_ == "OFFBOARD";
  }

  // hold 已触发、有足够后续采样、位置保持在容差内且发布连续
  bool holdOk() const {
    if (!hold_active_ || !has_hold_point_) return false;
    if (hold_samples_ < 3) return false;
    return hold_max_dev_ <= hold_tol_ && hold_max_gap_ <= gap_max_;
  }

  // ---------------------------------------------------------- 参数写入
  void setIfChanged(const std::string& name, const std::string& value) {
    if (cache_.count(name) && cache_[name] == value) return;
    ros::param::set(name, value);
    cache_[name] = value;
  }
  // 必须写原生 bool：门禁侧按布尔类型读取，写成字符串会导致类型不匹配恒为 false
  void setIfChangedBool(const std::string& name, bool value) {
    auto it = cache_bool_.find(name);
    if (it != cache_bool_.end() && it->second == value) return;
    ros::param::set(name, value);
    cache_bool_[name] = value;
  }

  void publishDiag() {
    setIfChanged("/agc/air/active_controller", activeController());
    setIfChangedBool("/agc/air/handover_ok", handoverOk());
    setIfChangedBool("/agc/air/hold_ok", holdOk());
    setIfChangedBool("/agc/air/control_conflict", airConflict());
    logAuthority();
  }

  // ---------------------------------------------------------- 交接报告
  std::string reportPath() const {
    // 优先本 attempt 钉住的目录（M1 遗留项 4）：终态后 logger 会立刻开新 run 目录，
    // 若仍按实时 log/run_dir 写，报告会落到下一个 attempt 的目录，旧 attempt 副本残缺。
    const std::string run_dir =
        !pinned_run_dir_.empty() ? pinned_run_dir_ : agc::cfgS("log/run_dir", "");
    if (!run_dir.empty() && isDir(run_dir)) return run_dir + "/air_handover.json";
    const std::string root = agc::cfgS("log/root", "/tmp/agc_runs");
    mkdirp(root);
    return root + "/air_handover.json";
  }

  std::string reportJson() const {
    agc::JsonObj o;
    o.bv("handover_detected", handover_detected_);
    if (handover_detected_ && handover_sim_time_ > 0.0) o.n("handover_sim_time", handover_sim_time_);
    else o.nullv("handover_sim_time");
    if (handover_detected_) o.n("handover_wall_time", handover_wall_time_);
    else o.nullv("handover_wall_time");
    o.raw("publishers_before", agc::JsonObj::arrStr(publishers_before_));
    o.raw("publishers_after", agc::JsonObj::arrStr(publishers_after_));
    o.n("max_setpoint_gap_s", round6(max_gap_));
    if (has_handover_gap_) o.n("handover_gap_s", round6(handover_gap_));
    else o.nullv("handover_gap_s");
    o.iv("setpoint_count", sp_count_).v("mavros_mode", mavros_mode_);
    o.bv("hold_verified", holdOk());
    o.v("notes", notes_.empty() ? "no handover/hold event recorded" : join(notes_, "; "));
    o.v("active_controller", activeController());
    o.bv("handover_ok", handoverOk());
    o.bv("hold_ok", holdOk());
    o.bv("control_conflict", airConflict());
    o.iv("task_state_at_report", task_state_);
    o.n("setpoint_gap_max_s_param", gap_max_);
    o.n("hold_position_tolerance_m_param", hold_tol_);
    if (has_first_sp_sim_) o.n("setpoint_first_sim_time", first_sp_sim_);
    else o.nullv("setpoint_first_sim_time");
    if (last_sp_sim_ > 0.0) o.n("setpoint_last_sim_time", last_sp_sim_);
    else o.nullv("setpoint_last_sim_time");
    o.raw("publishers_seen", agc::JsonObj::arrStr(sortVec(pub_nodes_)));
    o.bv("mavros_armed", armed_);
    o.bv("mavros_connected", connected_);
    std::vector<std::string> changes;
    changes.reserve(mode_changes_.size());
    for (const auto& c : mode_changes_) {
      agc::JsonObj e;
      if (c.sim_time > 0.0) e.n("sim_time", c.sim_time);
      else e.nullv("sim_time");
      e.n("wall_time", c.wall_time).v("mode", c.mode);
      changes.push_back(e.dump());
    }
    o.raw("mavros_mode_changes", agc::JsonObj::arr(changes));
    o.bv("hold_triggered", hold_active_);
    if (hold_trigger_sim_ > 0.0) o.n("hold_trigger_sim_time", hold_trigger_sim_);
    else o.nullv("hold_trigger_sim_time");
    o.iv("hold_samples", hold_samples_).n("hold_max_deviation_m", round6(hold_max_dev_));
    o.n("hold_max_gap_s", round6(hold_max_gap_)).n("run_start_wall_time", start_wall_);
    return o.dump();
  }

  std::string writeReport() {
    const std::string path = reportPath();
    const std::string text = reportJson();
    {
      std::ofstream f(path, std::ios::trunc);
      if (f.is_open()) f << text;
      else ROS_WARN("[air_control_bridge] 写交接报告失败 %s", path.c_str());
    }
    // 同一 JSON 字符串写入 /agc/air/handover_json（不占用 calibration_json）
    ros::param::set("/agc/air/handover_json", text);
    last_report_wall_ = agc::wallNow();
    return path;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_sp_, sub_state_, sub_task_;
  double hz_, gap_max_, hold_tol_, target_lost_tol_, start_wall_;

  // setpoint 时序统计
  long long sp_count_ = 0;
  double first_sp_sim_ = 0.0;
  bool has_first_sp_sim_ = false;
  double last_sp_sim_ = -1.0;
  double last_sp_wall_ = 0.0;
  bool has_last_sp_wall_ = false;
  double max_gap_ = 0.0;
  std::array<double, 3> last_pos_{{0.0, 0.0, 0.0}};
  bool has_last_pos_ = false;

  // mavros 状态
  std::string mavros_mode_ = "unknown";
  bool armed_ = false;
  bool connected_ = false;
  struct ModeChange { double sim_time; double wall_time; std::string mode; };
  std::vector<ModeChange> mode_changes_;

  // 发布者集合与交接判定
  std::set<std::string> pub_nodes_;
  std::vector<std::string> publishers_before_, publishers_after_;
  bool handover_detected_ = false;
  double handover_sim_time_ = -1.0;
  double handover_wall_time_ = 0.0;
  double handover_last_wall_ = 0.0;
  bool has_handover_last_wall_ = false;
  double handover_gap_ = 0.0;
  bool has_handover_gap_ = false;
  bool handover_gap_pending_ = false;
  bool sysstate_logged_ = false;

  // 任务状态与 hold 验证
  bool hold_active_ = false;
  std::string pinned_run_dir_;  // 本 attempt 钉住的 run 目录（M1 遗留项 4）
  double hold_trigger_sim_ = -1.0;
  double hold_trigger_wall_ = 0.0;
  std::array<double, 3> hold_point_{{0.0, 0.0, 0.0}};
  bool has_hold_point_ = false;
  long long hold_samples_ = 0;
  double hold_max_dev_ = 0.0;
  double hold_max_gap_ = 0.0;

  std::vector<std::string> notes_;
  std::map<std::string, std::string> cache_;
  std::map<std::string, bool> cache_bool_;
  double last_report_wall_ = 0.0;

  // M5：任务阶段 / 控制权时间线
  int task_state_ = 0;
  bool task_running_ = false;
  bool has_authority_log_ = false;
  std::string last_auth_key_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "air_control_bridge");
  AirControlBridge bridge;
  // 用 AsyncSpinner 在后台线程及时处理回调（等价 rospy 的独立回调线程）；
  // 若在 10 Hz 主循环里 spinOnce，setpoint 会被成批处理，交接空窗量测失真。
  ros::AsyncSpinner spinner(2);
  spinner.start();
  bridge.run();
  bridge.shutdown();
  return 0;
}