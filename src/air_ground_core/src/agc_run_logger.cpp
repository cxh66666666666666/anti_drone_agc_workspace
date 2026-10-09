// agc_run_logger —— 空地协同层运行日志节点（M0 接口契约 v1.1，C++ 实现）。
//
// 设计要点（契约第六节「run 日志与统计口径」）：
// - 每个 attempt 对应独立 run 目录 <root>/<run_id>，run_id=<UTCyyyymmddTHHMMSS>_<6位短哈希>；
// - 启动即建初始 run 目录并写 /agc/log/run_dir、/agc/log/run_id，并立刻落一份占位 summary.json
//   （phase=awaiting_attempt），使被强杀后的空目录不会被离线工具判成「日志损坏」；
// - /agc/task/state 进入终态(4/5/6/8) 先落 summary.json，再立刻开新 run；同码终态重复广播丢弃；
// - 启动快照（门禁/复位/节点/发布者/进程）在 attempt 起点（IDLE -> 非 IDLE）抓取并缓存，
//   finalize 时写缓存值：栈退出后再取会得到已停机的残缺清单（M1 修正）；
//   没有 attempt 的 run 目录（终态后自动新开的那一个）这些字段写 null，由编排层记录代替；
// - 进程退出（ros::spin 返回）补写当前 run 的 summary.json（clean_exit=true）。
// 只订阅与写文件，不发布任何话题。
#include "agc/agc_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <ros/master.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/State.h>
#include <air_ground_core/AgcTaskState.h>
#include <air_ground_core/TargetTrackArray.h>
#include <air_ground_core/AirObservationArray.h>
#include <air_ground_core/ObstacleTrackArray.h>

namespace {

const char* const DEFAULT_ROOT = "/home/ubuntu/anti_drone_agc_workspace/experiments/agc_runs";
const char* const STATE_TOPIC = "/agc/task/state";

// ---------------------------------------------------------------- 文件/路径工具
bool isDir(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}
bool isFile(const std::string& p) {
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
std::string dirName(const std::string& p) {
  const size_t pos = p.find_last_of('/');
  if (pos == std::string::npos) return ".";
  if (pos == 0) return "/";
  return p.substr(0, pos);
}
std::string readFile(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  if (!f.is_open()) return "";
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
void writeText(const std::string& path, const std::string& text) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (f.is_open()) f << text;
}
// 原子写 JSON：先写 .tmp 再 rename（与 Python write_json 的 os.replace 语义一致）。
void writeJsonAtomic(const std::string& path, const std::string& json) {
  const std::string tmp = path + ".tmp";
  writeText(tmp, json);
  std::rename(tmp.c_str(), path.c_str());
}
void copyFile(const std::string& src, const std::string& dst) {
  std::ifstream in(src, std::ios::binary);
  if (!in.is_open()) return;
  std::ofstream out(dst, std::ios::binary | std::ios::trunc);
  if (out.is_open()) out << in.rdbuf();
}

// ---------------------------------------------------------------- 包/工作区路径
// __FILE__ 在 catkin 下是绝对路径（<pkg>/src/<file>.cpp），据此上溯得到包根；
// 兜底时尝试从 /proc/self/exe（devel/lib/air_ground_core/<node>）推导，最后写死默认。
std::string derivePackageRoot() {
  std::vector<std::string> cands;
  const std::string f = __FILE__;
  cands.push_back(dirName(dirName(f)));
  char exe[1024];
  const ssize_t n = ::readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  if (n > 0) {
    exe[n] = '\0';
    const std::string lib_dir = dirName(std::string(exe));  // .../devel/lib/air_ground_core
    cands.push_back(dirName(dirName(dirName(lib_dir))) + "/src/air_ground_core");
  }
  cands.push_back("/home/ubuntu/anti_drone_agc_workspace/src/air_ground_core");
  for (const auto& c : cands) {
    // 只接受绝对路径候选：相对路径会随运行 CWD 变化而失效
    if (c.empty() || c[0] != '/') continue;
    if (isFile(c + "/package.xml") || isFile(c + "/config/agc_params.yaml")) return c;
  }
  return cands.back();
}

const std::string PKG_ROOT = derivePackageRoot();
const std::string WORKSPACE_ROOT = dirName(dirName(PKG_ROOT));
const std::string CONTRACT_DOC =
    WORKSPACE_ROOT + "/docs/项目任务日志/M0/M0_接口契约_v1.2.md";

// ---------------------------------------------------------------- 参数
std::string logRoot() {
  const std::string r = agc::cfgS("log/root", DEFAULT_ROOT);
  return r.empty() ? std::string(DEFAULT_ROOT) : r;
}

// XmlRpc 值 -> JSON（用于 meta.params 与 startup_result）。
std::string xmlrpcToJson(const XmlRpc::XmlRpcValue& v) {
  switch (v.getType()) {
    case XmlRpc::XmlRpcValue::TypeString:
      return agc::JsonObj::quote(static_cast<std::string>(v));
    case XmlRpc::XmlRpcValue::TypeInt:
      return std::to_string(static_cast<int>(v));
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return static_cast<bool>(v) ? "true" : "false";
    case XmlRpc::XmlRpcValue::TypeDouble:
      return agc::JsonObj::num(static_cast<double>(v));
    case XmlRpc::XmlRpcValue::TypeArray: {
      std::string s = "[";
      for (int i = 0; i < v.size(); ++i) {
        if (i) s += ", ";
        s += xmlrpcToJson(v[i]);
      }
      return s + "]";
    }
    case XmlRpc::XmlRpcValue::TypeStruct: {
      std::string s = "{";
      bool first = true;
      for (auto it = v.begin(); it != v.end(); ++it) {
        if (!first) s += ", ";
        first = false;
        s += agc::JsonObj::quote(it->first) + ": " + xmlrpcToJson(it->second);
      }
      return s + "}";
    }
    default:
      return "null";
  }
}

std::string paramRawJson(const std::string& name) {
  XmlRpc::XmlRpcValue v;
  if (!ros::param::get(name, v)) return "null";
  return xmlrpcToJson(v);
}

// Python _as_bool：宽松转 bool；无法判定返回 null。
std::string paramBoolJson(const std::string& name) {
  XmlRpc::XmlRpcValue v;
  if (!ros::param::get(name, v)) return "null";
  if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean) return static_cast<bool>(v) ? "true" : "false";
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v) ? "true" : "false";
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
    std::string s = static_cast<std::string>(v);
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if (s == "true" || s == "1" || s == "yes") return "true";
    if (s == "false" || s == "0" || s == "no") return "false";
  }
  return "null";
}

// 读取参数并尽量按 JSON 原文嵌入：形如 {..} / [..] 视为 JSON，否则当字符串。
// 用于把已落盘 JSON 快照类参数（handover/calibration/gate）原样写入 summary。
std::string paramJsonish(const std::string& name) {
  XmlRpc::XmlRpcValue v;
  if (!ros::param::get(name, v)) return "null";
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
    const std::string s = static_cast<std::string>(v);
    const size_t p = s.find_first_not_of(" \t\r\n");
    if (p != std::string::npos && (s[p] == '{' || s[p] == '[')) return s;  // 已是 JSON 文本
    return agc::JsonObj::quote(s);
  }
  return xmlrpcToJson(v);
}

// 节点名最后一段：/gazebo -> gazebo，对齐 task_manager 的 lastSegment
std::string lastSegment(const std::string& full) {
  size_t b = 0, e = full.size();
  while (b < e && full[b] == '/') ++b;
  while (e > b && full[e - 1] == '/') --e;
  const std::string s = full.substr(b, e - b);
  const size_t p = s.find_last_of('/');
  return p == std::string::npos ? s : s.substr(p + 1);
}

// ---------------------------------------------------------------- 节点 / 发布者 / 进程快照
std::vector<std::string> rosNodes() {
  std::vector<std::string> out;
  try {
    if (!ros::master::getNodes(out)) out.clear();
  } catch (...) {
    out.clear();
  }
  std::sort(out.begin(), out.end());
  return out;
}

// 话题实际发布者节点名（getSystemState）；ok=false 表示查询失败（写 null 而非空数组）
std::vector<std::string> publishersOf(const std::string& topic, bool* ok) {
  std::vector<std::string> out;
  if (ok != nullptr) *ok = false;
  try {
    XmlRpc::XmlRpcValue args, response, payload;
    args[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", args, response, payload, false)) return out;
    if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1) return out;
    XmlRpc::XmlRpcValue pubs = payload[0];
    if (pubs.getType() != XmlRpc::XmlRpcValue::TypeArray) return out;
    if (ok != nullptr) *ok = true;
    for (int i = 0; i < pubs.size(); ++i) {
      XmlRpc::XmlRpcValue entry = pubs[i];
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) continue;
      if (entry[0].getType() != XmlRpc::XmlRpcValue::TypeString) continue;
      if (std::string(entry[0]) != topic) continue;
      XmlRpc::XmlRpcValue nodes = entry[1];
      if (nodes.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int j = 0; j < nodes.size(); ++j) {
          if (nodes[j].getType() == XmlRpc::XmlRpcValue::TypeString)
            out.push_back(std::string(nodes[j]));
        }
      }
      break;
    }
  } catch (...) {
    out.clear();
    if (ok != nullptr) *ok = false;
  }
  std::sort(out.begin(), out.end());
  return out;
}

// 需要核查的进程名（与 start_agc_collab.sh 的清理名单一致）
const std::vector<std::string>& procKeywords() {
  static const std::vector<std::string> k = {
      "px4", "gzserver", "gzclient", "mavros",
      "air_odom_adapter", "ground_odom_adapter", "gt_adapter", "air_perception", "air_fusion",
      "air_follow",
      "map_adapter",
      "task_manager", "ground_planner", "ground_commander", "ground_drive_adapter",
      "intruder_driver", "scenario_reset", "agc_run_logger", "air_control_bridge",
      "lqr_tracker_node", "drone_launch_controller", "rosmaster", "roscore", "rosout"};
  return k;
}

struct ProcInfo {
  int pid = 0;
  std::string comm, cmd;
};

// 扫描 /proc，按关键字匹配 comm 或 cmdline（comm 会被内核截断到 15 字符，故两者都查）
std::vector<ProcInfo> scanProcs() {
  std::vector<ProcInfo> out;
  DIR* d = ::opendir("/proc");
  if (d == nullptr) return out;
  const std::vector<std::string>& kws = procKeywords();
  for (struct dirent* e = ::readdir(d); e != nullptr; e = ::readdir(d)) {
    const std::string nm = e->d_name;
    if (nm.empty() || nm.find_first_not_of("0123456789") != std::string::npos) continue;
    const std::string base = "/proc/" + nm;
    std::string comm = readFile(base + "/comm");
    while (!comm.empty() && (comm.back() == '\n' || comm.back() == '\r')) comm.pop_back();
    std::string cmd = readFile(base + "/cmdline");
    for (char& c : cmd)
      if (c == '\0') c = ' ';
    bool match = false;
    for (const auto& kw : kws) {
      if (comm.find(kw) != std::string::npos || cmd.find(kw) != std::string::npos) {
        match = true;
        break;
      }
    }
    if (match) out.push_back({std::atoi(nm.c_str()), comm, cmd});
  }
  ::closedir(d);
  std::sort(out.begin(), out.end(),
            [](const ProcInfo& a, const ProcInfo& b) { return a.pid < b.pid; });
  return out;
}

// ---------------------------------------------------------------- 哈希与 run_id
std::string scriptsHash() {
  const std::string dir = PKG_ROOT + "/scripts";
  DIR* d = ::opendir(dir.c_str());
  if (d == nullptr) return "missing";
  std::vector<std::string> names;
  for (struct dirent* e = ::readdir(d); e != nullptr; e = ::readdir(d)) {
    const std::string nm = e->d_name;
    if (nm.size() > 3 && nm.substr(nm.size() - 3) == ".py") names.push_back(nm);
  }
  ::closedir(d);
  std::sort(names.begin(), names.end());
  std::string all;
  for (const auto& nm : names) all += readFile(dir + "/" + nm);
  return agc::sha1Text(all);
}

std::string gitCommit() {
  const std::string cmd =
      "timeout 5 git -C \"" + WORKSPACE_ROOT + "\" rev-parse HEAD 2>/dev/null";
  FILE* p = ::popen(cmd.c_str(), "r");
  if (p == nullptr) return "unknown";
  char buf[512];
  std::string out;
  while (std::fgets(buf, sizeof(buf), p) != nullptr) out += buf;
  ::pclose(p);
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' ')) {
    out.pop_back();
  }
  return out.empty() ? "unknown" : out;
}

std::string makeRunId() {
  char ts[32];
  const time_t now = ::time(nullptr);
  struct tm tmv;
  ::gmtime_r(&now, &tmv);
  ::strftime(ts, sizeof(ts), "%Y%m%dT%H%M%S", &tmv);

  timeval tv;
  ::gettimeofday(&tv, nullptr);
  static std::mt19937 rng(static_cast<unsigned>(std::random_device{}()));
  const int r = static_cast<int>(rng() & 0x3FFFFFFF);
  char raw[96];
  std::snprintf(raw, sizeof(raw), "%.6f-%d-%d", tv.tv_sec + tv.tv_usec / 1e6,
                static_cast<int>(::getpid()), r);
  return std::string(ts) + "_" + agc::sha1Text(raw).substr(0, 6);
}

// ---------------------------------------------------------------- 消息序列化
// 与 Python agc_run_logger.msg_to_dict 对齐：header.stamp 是 genpy.Time 的 str()，
// 即「纳秒字符串」（如 "172000000"），而非 secs/nsecs 对象。
std::string nsecStr(const ros::Time& t) {
  const long long ns = static_cast<long long>(t.sec) * 1000000000LL + t.nsec;
  return std::to_string(ns);
}
std::string headerJson(const std_msgs::Header& h) {
  agc::JsonObj o;
  o.iv("seq", h.seq).v("stamp", nsecStr(h.stamp)).v("frame_id", h.frame_id);
  return o.dump();
}
std::string pointJson(double x, double y, double z) {
  agc::JsonObj o;
  o.n("x", x).n("y", y).n("z", z);
  return o.dump();
}
std::string quatJson(double x, double y, double z, double w) {
  agc::JsonObj o;
  o.n("x", x).n("y", y).n("z", z).n("w", w);
  return o.dump();
}
std::string poseJson(const geometry_msgs::Pose& p) {
  agc::JsonObj o;
  o.raw("position", pointJson(p.position.x, p.position.y, p.position.z));
  o.raw("orientation", quatJson(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
  return o.dump();
}
std::string twistJson(const geometry_msgs::Twist& t) {
  agc::JsonObj o;
  o.raw("linear", pointJson(t.linear.x, t.linear.y, t.linear.z));
  o.raw("angular", pointJson(t.angular.x, t.angular.y, t.angular.z));
  return o.dump();
}
template <typename Arr>
std::string covJson(const Arr& c) {
  std::vector<double> v;
  v.reserve(c.size());
  for (const auto& x : c) v.push_back(static_cast<double>(x));
  return agc::JsonObj::arrNum(v);
}

std::string targetTrackArrayJson(const air_ground_core::TargetTrackArray& m) {
  std::vector<std::string> tracks;
  tracks.reserve(m.tracks.size());
  for (const auto& t : m.tracks) {
    agc::JsonObj po;
    po.raw("pose", poseJson(t.pose.pose)).raw("covariance", covJson(t.pose.covariance));
    agc::JsonObj tw;
    tw.raw("twist", twistJson(t.twist.twist)).raw("covariance", covJson(t.twist.covariance));
    agc::JsonObj o;
    o.raw("header", headerJson(t.header)).v("target_id", t.target_id);
    o.raw("pose", po.dump()).raw("twist", tw.dump());
    o.n("confidence", t.confidence).v("source", t.source);
    tracks.push_back(o.dump());
  }
  agc::JsonObj o;
  o.raw("header", headerJson(m.header)).raw("tracks", agc::JsonObj::arr(tracks));
  return o.dump();
}

std::string airObservationArrayJson(const air_ground_core::AirObservationArray& m) {
  std::vector<std::string> sensors;
  sensors.reserve(m.sensors.size());
  for (const auto& so : m.sensors) {
    std::vector<std::string> tgts, obs;
    for (const auto& t : so.targets) {
      agc::JsonObj o;
      o.raw("header", headerJson(t.header)).v("target_id", t.target_id).v("sensor_id", t.sensor_id);
      o.raw("pose", poseJson(t.pose.pose)).raw("covariance", covJson(t.pose.covariance));
      o.n("confidence", t.confidence).iv("obs_seq", t.obs_seq);
      tgts.push_back(o.dump());
    }
    for (const auto& b : so.obstacles) {
      agc::JsonObj o;
      o.raw("header", headerJson(b.header)).v("obstacle_id", b.obstacle_id)
          .v("sensor_id", b.sensor_id);
      o.raw("pose", poseJson(b.pose.pose)).raw("covariance", covJson(b.pose.covariance));
      o.raw("size", pointJson(b.size.x, b.size.y, b.size.z));
      o.n("confidence", b.confidence).iv("obs_seq", b.obs_seq);
      obs.push_back(o.dump());
    }
    agc::JsonObj o;
    o.raw("header", headerJson(so.header)).v("sensor_id", so.sensor_id);
    o.raw("targets", agc::JsonObj::arr(tgts)).raw("obstacles", agc::JsonObj::arr(obs));
    sensors.push_back(o.dump());
  }
  agc::JsonObj o;
  o.raw("header", headerJson(m.header)).raw("sensors", agc::JsonObj::arr(sensors));
  return o.dump();
}

std::string obstacleTrackArrayJson(const air_ground_core::ObstacleTrackArray& m) {
  std::vector<std::string> tracks;
  tracks.reserve(m.tracks.size());
  for (const auto& t : m.tracks) {
    agc::JsonObj o;
    o.raw("header", headerJson(t.header)).v("obstacle_id", t.obstacle_id);
    o.raw("pose", poseJson(t.pose)).raw("size", pointJson(t.size.x, t.size.y, t.size.z));
    o.n("confidence", t.confidence).v("source", t.source);
    tracks.push_back(o.dump());
  }
  agc::JsonObj o;
  o.raw("header", headerJson(m.header)).raw("tracks", agc::JsonObj::arr(tracks));
  return o.dump();
}

std::string agcTaskStateJson(const air_ground_core::AgcTaskState& m) {
  agc::JsonObj o;
  o.raw("header", headerJson(m.header));
  o.iv("state", m.state).v("state_name", m.state_name).v("reason_code", m.reason_code);
  o.v("air_controller", m.air_controller).v("ground_controller", m.ground_controller);
  o.v("target_id", m.target_id);
  o.n("target_xy_distance_m", m.target_xy_distance_m).n("target_obs_age_s", m.target_obs_age_s);
  o.bv("tracking_milestone_met", m.tracking_milestone_met);
  return o.dump();
}

std::string modelStatesJson(const gazebo_msgs::ModelStates& m) {
  std::vector<std::string> poses, twists;
  poses.reserve(m.pose.size());
  twists.reserve(m.twist.size());
  for (const auto& p : m.pose) poses.push_back(poseJson(p));
  for (const auto& t : m.twist) twists.push_back(twistJson(t));
  agc::JsonObj o;
  o.raw("name", agc::JsonObj::arrStr(m.name));
  o.raw("pose", agc::JsonObj::arr(poses));
  o.raw("twist", agc::JsonObj::arr(twists));
  return o.dump();
}

std::string mavrosStateJson(const mavros_msgs::State& m) {
  agc::JsonObj o;
  o.raw("header", headerJson(m.header));
  o.bv("connected", m.connected).bv("armed", m.armed).bv("guided", m.guided);
  o.bv("manual_input", m.manual_input).v("mode", m.mode).iv("system_status", m.system_status);
  return o.dump();
}

// ---------------------------------------------------------------- meta
struct MetaData {
  std::string run_id;
  std::string contract_version_json = "null";
  std::string contract_doc, agc_params_hash, scenario_hash, world_hash;
  std::string car_hash, uav_hash, scripts_hash, git_commit;
  std::string target_source = "gt_adapter";
  std::string obstacle_source = "scenario";
  std::string drive_backend = "kinematic_sim";
  std::string seed = "unseeded";
  std::string params_json = "{}";
  double t_sim_start = 0.0, t_wall_start = 0.0;
  double t_sim_end = 0.0, t_wall_end = 0.0;
  bool has_end = false;
};

std::string metaJson(const MetaData& m) {
  agc::JsonObj models;
  models.v("agc_car", m.car_hash).v("intruder_uav", m.uav_hash);
  agc::JsonObj o;
  o.v("run_id", m.run_id);
  o.raw("contract_version", m.contract_version_json);
  o.v("contract_doc", m.contract_doc).v("agc_params_hash", m.agc_params_hash);
  o.v("scenario_hash", m.scenario_hash).v("world_hash", m.world_hash);
  o.raw("model_hashes", models.dump());
  o.v("scripts_hash", m.scripts_hash).v("git_commit", m.git_commit);
  o.v("target_source", m.target_source).v("obstacle_source", m.obstacle_source)
      .v("drive_backend", m.drive_backend);
  o.v("seed", m.seed).raw("params", m.params_json);
  o.n("t_sim_start", m.t_sim_start).n("t_wall_start", m.t_wall_start);
  if (m.has_end) o.n("t_sim_end", m.t_sim_end).n("t_wall_end", m.t_wall_end);
  else o.nullv("t_sim_end").nullv("t_wall_end");
  return o.dump();
}

// ---------------------------------------------------------------- 辅助文件复制
void materializeAux(const std::string& param_path, const std::string& fallback,
                    const std::string& dst) {
  XmlRpc::XmlRpcValue v;
  if (ros::param::get(param_path, v)) {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct ||
        v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      writeJsonAtomic(dst, xmlrpcToJson(v));
      return;
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
      const std::string s = static_cast<std::string>(v);
      if (s.find_first_not_of(" \t\r\n") != std::string::npos) {
        if (isFile(s)) copyFile(s, dst);
        else writeText(dst, s);  // 参数值本身即 JSON 文本（保证目标为合法 JSON）
        return;
      }
    }
  }
  if (!fallback.empty() && isFile(fallback)) copyFile(fallback, dst);
}

void copyAuxFiles(const std::string& run_dir, const std::string& root) {
  materializeAux("/agc/air/calibration_json", root + "/air_calibration.json",
                 run_dir + "/air_calibration.json");
  materializeAux("/agc/air/handover_json", root + "/air_handover.json",
                 run_dir + "/air_handover.json");
}

// ---------------------------------------------------------------- run 记录器
class RunRecorder {
 public:
  RunRecorder(const std::string& run_id, const std::string& run_dir, const MetaData& meta,
              const std::string& root)
      : run_id_(run_id), run_dir_(run_dir), root_(root), meta_(meta) {
    topics_dir_ = run_dir_ + "/topics";
    mkdirp(topics_dir_);
    writeMeta();
    copyAuxFiles(run_dir_, root_);
  }

  const std::string& runDir() const { return run_dir_; }
  const std::string& runId() const { return run_id_; }
  bool finalized() const { return finalized_; }

  void writeMeta() const { writeJsonAtomic(run_dir_ + "/meta.json", metaJson(meta_)); }

  // attempt 起点用当回合实际参数重建并覆盖写 meta.json（修正场景参数错位）
  void setMeta(const MetaData& m) {
    meta_ = m;
    writeMeta();
  }

  void record(const std::string& topic, const std::string& msg_json) {
    const double t_sim = agc::simNow().toSec();
    const double t_wall = agc::wallNow();
    Stat& st = stats_[topic];
    if (st.count == 0) {
      st.first_sim = t_sim;
      st.has_first = true;
    } else {
      const double gap = t_sim - st.prev;
      if (gap >= 0.0 && gap > st.max_gap) st.max_gap = gap;
    }
    ++st.count;
    st.last_sim = t_sim;
    st.prev = t_sim;
    agc::JsonObj o;
    o.n("t_sim", t_sim).n("t_wall", t_wall).raw("msg", msg_json);
    std::ofstream* f = open(topic);
    if (f != nullptr) *f << o.dump() << "\n";
  }

  void recordState(const std::string& topic, const std::string& msg_json, int state,
                   const std::string& name, const std::string& reason) {
    record(topic, msg_json);
    has_state_ = true;
    last_state_ = state;
    last_state_name_ = name;
    last_state_reason_ = reason;
  }

  void flush() {
    for (auto& kv : files_) {
      if (kv.second != nullptr) kv.second->flush();
    }
  }

  // ---------------------------------------------------------------- 每 run 证据快照
  // 启动状态：门禁 + 交接 + 标定 + 后端 + odom 有效 + 时钟 + 契约/场景版本
  // 单 world 由门禁明细给出（gate.gate_failures.single_world），此处不再重复推导
  std::string startupJson() const {
    agc::JsonObj o;
    o.raw("gate", paramJsonish("/agc/start/last_gate_json"));
    o.raw("handover", paramJsonish("/agc/air/handover_json"));
    o.raw("air_calibration", paramJsonish("/agc/air/calibration_json"));
    o.v("drive_backend", agc::pgetStr("/agc/drive/backend", ""));
    o.raw("drive_backend_ok", paramBoolJson("/agc/drive/backend_ok"));
    o.raw("air_odom_valid", paramBoolJson("/agc/air/odom_valid"));
    o.raw("ground_odom_valid", paramBoolJson("/agc/ground/odom_valid"));
    o.raw("use_sim_time", paramBoolJson("/use_sim_time"));
    o.raw("contract_version", paramRawJson("/agc/contract_version"));
    o.v("scenario_id", agc::loadScenario(agc::cfgS("scenario_file", "")).id);
    return o.dump();
  }

  // 复位结果：本次 run 开始时的 last_result 与 epoch
  std::string resetJson() const {
    agc::JsonObj o;
    o.raw("last_result", paramRawJson("/agc/reset/last_result"));
    o.raw("epoch", paramRawJson("/agc/reset/epoch"));
    return o.dump();
  }

  std::string nodesJson() const { return agc::JsonObj::arrStr(rosNodes()); }

  // 8 个契约话题的实际发布者（唯一发布者核查依据）
  std::string publishersJson() const {
    static const char* const kTopics[] = {
        "/agc/air/odom", "/agc/ground/odom", "/agc/targets", "/agc/map",
        "/agc/ground/goal", "/agc/ground/path", "/agc/ground/cmd_vel", "/agc/task/state",
        "/agc/air/obs", "/agc/obstacles"};
    agc::JsonObj o;
    for (const char* t : kTopics) {
      bool ok = false;
      const std::vector<std::string> p = publishersOf(t, &ok);
      o.raw(t, ok ? agc::JsonObj::arrStr(p) : std::string("null"));
    }
    return o.dump();
  }

  // 关键 OS 进程清单（证明唯一写入者、无重复/遗留进程）
  std::string procsJson() const {
    std::vector<std::string> items;
    for (const ProcInfo& p : scanProcs()) {
      agc::JsonObj o;
      o.iv("pid", p.pid).v("comm", p.comm).v("cmd", p.cmd);
      items.push_back(o.dump());
    }
    return agc::JsonObj::arr(items);
  }

  // attempt 起点抓一次快照并缓存（此后栈会停机，finalize 再取只会得到残缺清单）
  void captureSnapshot() {
    if (has_snapshot_) return;
    snap_startup_ = startupJson();
    snap_reset_ = resetJson();
    snap_nodes_ = nodesJson();
    snap_pubs_ = publishersJson();
    snap_procs_ = procsJson();
    has_snapshot_ = true;
  }
  bool hasSnapshot() const { return has_snapshot_; }

  void writePlaceholderSummary(const std::string& note) const {
    if (finalized_) return;
    agc::JsonObj counting;
    counting.nullv("category").iv("attempted_delta", 0).iv("valid_task_delta", 0);
    counting.iv("infra_failure_delta", 0).iv("aborted_delta", 0);
    counting.iv("tracking_success_delta", 0).iv("mission_failure_delta", 0);
    agc::JsonObj o;
    o.v("run_id", run_id_).n("t_sim_start", meta_.t_sim_start).n("t_wall_start", meta_.t_wall_start);
    o.nullv("final_state").nullv("final_state_name").nullv("final_reason_code");
    o.bv("clean_exit", false).v("phase", note).raw("counting", counting.dump());
    writeJsonAtomic(run_dir_ + "/summary.json", o.dump());
  }

  void finalize(const std::string& reason, bool clean_exit) {
    if (finalized_) return;
    finalized_ = true;
    closeAll();

    const double t_sim_end = agc::simNow().toSec();
    const double t_wall_end = agc::wallNow();
    meta_.has_end = true;
    meta_.t_sim_end = t_sim_end;
    meta_.t_wall_end = t_wall_end;
    writeMeta();
    copyAuxFiles(run_dir_, root_);

    // 归属规则：终态 6→aborted、5→mission_failure、4→tracking_success、8→infra_failure
    std::string category;
    if (has_state_) {
      if (last_state_ == agc::ST_TRACKING_SUCCESS) category = "tracking_success";
      else if (last_state_ == agc::ST_MISSION_FAILED) category = "mission_failure";
      else if (last_state_ == agc::ST_ABORTED) category = "aborted";
      else if (last_state_ == agc::ST_INFRA_FAILURE) category = "infra_failure";
    }
    const bool valid = (category == "tracking_success" || category == "mission_failure");

    agc::JsonObj counting;
    if (category.empty()) counting.nullv("category");
    else counting.v("category", category);
    counting.iv("attempted_delta", category.empty() ? 0 : 1)
        .iv("valid_task_delta", valid ? 1 : 0)
        .iv("infra_failure_delta", category == "infra_failure" ? 1 : 0)
        .iv("aborted_delta", category == "aborted" ? 1 : 0)
        .iv("tracking_success_delta", category == "tracking_success" ? 1 : 0)
        .iv("mission_failure_delta", category == "mission_failure" ? 1 : 0);

    // topics / max_inter_arrival_s
    agc::JsonObj topics;
    agc::JsonObj max_gap;
    for (const auto& kv : stats_) {
      agc::JsonObj t;
      t.iv("count", kv.second.count);
      if (kv.second.has_first) t.n("first_t_sim", kv.second.first_sim);
      else t.nullv("first_t_sim");
      t.n("last_t_sim", kv.second.last_sim);
      topics.raw(kv.first, t.dump());
      max_gap.n(kv.first, kv.second.max_gap);
    }

    agc::JsonObj o;
    o.v("run_id", run_id_).n("t_sim_start", meta_.t_sim_start).n("t_sim_end", t_sim_end);
    o.n("t_wall_start", meta_.t_wall_start).n("t_wall_end", t_wall_end);
    o.raw("topics", topics.dump()).raw("max_inter_arrival_s", max_gap.dump());
    if (has_state_) o.iv("final_state", last_state_);
    else o.nullv("final_state");
    if (has_state_) {
      const std::string nm = last_state_name_.empty() ? agc::stateName(last_state_) : last_state_name_;
      o.v("final_state_name", nm).v("final_reason_code", last_state_reason_);
    } else {
      o.nullv("final_state_name").nullv("final_reason_code");
    }
    // 启动快照取自 attempt 起点；本 run 目录内无 attempt（终态后自动新开的那一个）写 null
    if (has_snapshot_) {
      o.raw("startup", snap_startup_);
      o.raw("reset_result", snap_reset_);
      o.raw("nodes", snap_nodes_);
      o.raw("publishers", snap_pubs_);
      o.raw("processes", snap_procs_);
    } else {
      o.nullv("startup").nullv("reset_result").nullv("nodes").nullv("publishers").nullv("processes");
    }
    o.raw("drive_backend_ok", paramBoolJson("/agc/drive/backend_ok"));
    o.raw("air_handover_ok", paramBoolJson("/agc/air/handover_ok"));
    o.bv("clean_exit", clean_exit).raw("counting", counting.dump());
    o.v("finalize_reason", reason);
    if (category.empty()) o.nullv("finalize_category");
    else o.v("finalize_category", category);
    writeJsonAtomic(run_dir_ + "/summary.json", o.dump());

    ROS_INFO("[agc_run_logger] run %s 结算 reason=%s state=%s category=%s", run_id_.c_str(),
             reason.c_str(), has_state_ ? std::to_string(last_state_).c_str() : "None",
             category.empty() ? "None" : category.c_str());
  }

 private:
  struct Stat {
    long long count = 0;
    double first_sim = 0.0;
    bool has_first = false;
    double last_sim = 0.0;
    double max_gap = 0.0;
    double prev = 0.0;
  };

  std::ofstream* open(const std::string& topic) {
    auto it = files_.find(topic);
    if (it != files_.end()) return it->second;
    std::string safe = topic;
    std::replace(safe.begin(), safe.end(), '/', '_');
    std::ofstream* f = new std::ofstream((topics_dir_ + "/" + safe + ".jsonl").c_str(),
                                         std::ios::out | std::ios::trunc);
    files_[topic] = f->is_open() ? f : nullptr;
    if (!f->is_open()) {
      delete f;
      return nullptr;
    }
    return f;
  }

  void closeAll() {
    for (auto& kv : files_) {
      if (kv.second != nullptr) {
        kv.second->flush();
        kv.second->close();
        delete kv.second;
      }
    }
    files_.clear();
  }

  std::string run_id_, run_dir_, root_, topics_dir_;
  MetaData meta_;
  std::map<std::string, std::ofstream*> files_;
  std::map<std::string, Stat> stats_;
  bool has_state_ = false;
  int last_state_ = -1;
  std::string last_state_name_, last_state_reason_;
  bool has_snapshot_ = false;
  std::string snap_startup_, snap_reset_, snap_nodes_, snap_pubs_, snap_procs_;
  bool finalized_ = false;
};

// ---------------------------------------------------------------- 节点
class AgcRunLogger {
 public:
  AgcRunLogger() : nh_(), root_(logRoot()) {
    mkdirp(root_);
    newRun();
    setupTopics();
    // 周期性 flush，避免高频写盘阻塞回调
    flush_timer_ = nh_.createTimer(ros::Duration(1.0), &AgcRunLogger::onFlush, this);
    ROS_INFO("[agc_run_logger] 已启动，run_dir=%s", recorder_->runDir().c_str());
  }

  // 进程退出时补写当前 run 的 summary.json（clean_exit=true）
  void shutdown() {
    if (recorder_ != nullptr && !recorder_->finalized()) {
      recorder_->finalize("shutdown", true);
    }
  }

 private:
  template <typename M>
  void subscribeTopic(const std::string& topic,
                      void (AgcRunLogger::*fn)(const std::string&, const typename M::ConstPtr&)) {
    subs_.push_back(nh_.subscribe<M>(
        topic, 200, [this, topic, fn](const typename M::ConstPtr& m) { (this->*fn)(topic, m); }));
  }

  void setupTopics() {
    subscribeTopic<nav_msgs::Odometry>("/agc/air/odom", &AgcRunLogger::onOdometry);
    subscribeTopic<nav_msgs::Odometry>("/agc/ground/odom", &AgcRunLogger::onOdometry);
    subscribeTopic<air_ground_core::TargetTrackArray>("/agc/targets", &AgcRunLogger::onTargets);
    subscribeTopic<air_ground_core::AirObservationArray>("/agc/air/obs",
                                                         &AgcRunLogger::onAirObservation);
    subscribeTopic<air_ground_core::ObstacleTrackArray>("/agc/obstacles",
                                                        &AgcRunLogger::onObstacleTracks);
    subscribeTopic<nav_msgs::OccupancyGrid>("/agc/map", &AgcRunLogger::onOccupancyGrid);
    subscribeTopic<geometry_msgs::PoseStamped>("/agc/ground/goal", &AgcRunLogger::onPoseStamped);
    subscribeTopic<nav_msgs::Path>("/agc/ground/path", &AgcRunLogger::onPath);
    subscribeTopic<geometry_msgs::TwistStamped>("/agc/ground/cmd_vel", &AgcRunLogger::onTwistStamped);
    subscribeTopic<air_ground_core::AgcTaskState>("/agc/task/state", &AgcRunLogger::onTaskState);
    subscribeTopic<gazebo_msgs::ModelStates>("/gazebo/model_states", &AgcRunLogger::onModelStates);
    subscribeTopic<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",
                                              &AgcRunLogger::onPoseStamped);
    subscribeTopic<mavros_msgs::State>("/mavros/state", &AgcRunLogger::onMavrosState);
  }

  void onOdometry(const std::string& topic, const nav_msgs::Odometry::ConstPtr& m) {
    if (ok()) recorder_->record(topic, agc::msgJson(*m));
  }
  void onPoseStamped(const std::string& topic, const geometry_msgs::PoseStamped::ConstPtr& m) {
    if (ok()) recorder_->record(topic, agc::msgJson(*m));
  }
  void onPath(const std::string& topic, const nav_msgs::Path::ConstPtr& m) {
    if (ok()) recorder_->record(topic, agc::msgJson(*m));
  }
  void onOccupancyGrid(const std::string& topic, const nav_msgs::OccupancyGrid::ConstPtr& m) {
    if (ok()) recorder_->record(topic, agc::msgJson(*m));
  }
  void onTwistStamped(const std::string& topic, const geometry_msgs::TwistStamped::ConstPtr& m) {
    if (ok()) recorder_->record(topic, agc::msgJson(*m));
  }
  void onTargets(const std::string& topic, const air_ground_core::TargetTrackArray::ConstPtr& m) {
    if (ok()) recorder_->record(topic, targetTrackArrayJson(*m));
  }
  void onAirObservation(const std::string& topic,
                        const air_ground_core::AirObservationArray::ConstPtr& m) {
    if (ok()) recorder_->record(topic, airObservationArrayJson(*m));
  }
  void onObstacleTracks(const std::string& topic,
                        const air_ground_core::ObstacleTrackArray::ConstPtr& m) {
    if (ok()) recorder_->record(topic, obstacleTrackArrayJson(*m));
  }
  void onModelStates(const std::string& topic, const gazebo_msgs::ModelStates::ConstPtr& m) {
    if (ok()) recorder_->record(topic, modelStatesJson(*m));
  }
  void onMavrosState(const std::string& topic, const mavros_msgs::State::ConstPtr& m) {
    if (ok()) recorder_->record(topic, mavrosStateJson(*m));
  }

  void onTaskState(const std::string& topic, const air_ground_core::AgcTaskState::ConstPtr& m) {
    if (!ok()) return;
    const int state = m->state;
    const bool prev_terminal = last_state_ >= 0 && agc::isTerminal(last_state_);
    // 终态重复广播（task_manager 2 Hz 重发）不属于新 run，丢弃；
    // 同码才丢：门禁失败等「终态 -> 另一码终态」是新的 attempt，必须记录
    if (agc::isTerminal(state) && prev_terminal && state == last_state_) return;
    RunRecorder* rec = recorder_;
    // attempt 起点（IDLE -> 非 IDLE）抓启动快照：此刻节点/发布者齐备，且门禁明细已落盘
    if (state != agc::ST_IDLE) {
      rec->captureSnapshot();
      // 用当回合当前参数重建并覆盖 meta.json：newRun() 在终态即建 meta，此刻批跑器
      // 已设置下一场景参数，重建可保证 scenario_hash/world_hash/seed/params 与实际一致
      rec->setMeta(buildMeta(rec->runId()));
    }
    rec->recordState(topic, agcTaskStateJson(*m), state, m->state_name, m->reason_code);
    // 走到这里说明是「新终态」（同码重复已在上面丢弃）：结算本 run，再开下一个
    if (agc::isTerminal(state)) {
      rec->finalize("terminal_state_" + std::to_string(state), false);
      newRun();
    }
    last_state_ = state;
  }

  bool ok() const { return recorder_ != nullptr && !recorder_->finalized(); }

  void onFlush(const ros::TimerEvent&) {
    if (recorder_ != nullptr) recorder_->flush();
  }

  void newRun() {
    std::string run_id = makeRunId();
    std::string run_dir = root_ + "/" + run_id;
    while (isDir(run_dir)) {  // 极低概率重名兜底
      run_id = makeRunId();
      run_dir = root_ + "/" + run_id;
    }
    mkdirp(run_dir);
    MetaData meta = buildMeta(run_id);
    delete recorder_;  // 旧 recorder 已在调用方 finalize
    recorder_ = new RunRecorder(run_id, run_dir, meta, root_);
    recorder_->writePlaceholderSummary("awaiting_attempt");
    ros::param::set("/agc/log/run_dir", run_dir);
    ros::param::set("/agc/log/run_id", run_id);
  }

  MetaData buildMeta(const std::string& run_id) const {
    MetaData m;
    m.run_id = run_id;
    m.contract_version_json = paramRawJson("/agc/contract_version");
    m.contract_doc = agc::sha1File(CONTRACT_DOC);
    m.agc_params_hash = agc::sha1File(PKG_ROOT + "/config/agc_params.yaml");
    const std::string scenario_file = agc::cfgS("scenario_file", "");
    m.scenario_hash = scenario_file.empty() ? "missing" : agc::sha1File(scenario_file);
    // D1：对实际加载的 world 文件取哈希（/agc/world_file 由 launch 设置；缺省回落包内模板）
    m.world_hash = agc::sha1File(agc::cfgS("world_file", PKG_ROOT + "/worlds/agc_world.world"));
    m.car_hash = agc::sha1File(PKG_ROOT + "/models/agc_car/agc_car.sdf");
    m.uav_hash = agc::sha1File(PKG_ROOT + "/models/intruder_uav/intruder_uav.sdf");
    m.scripts_hash = scriptsHash();
    m.git_commit = gitCommit();
    m.target_source = agc::pgetStr("/agc/target/source", "gt_adapter");
    m.obstacle_source = agc::pgetStr("/agc/obstacles/source", "scenario");
    const std::string backend = agc::cfgS("drive/backend", "");
    m.drive_backend = backend.empty() ? "kinematic_sim" : backend;
    const std::string seed = agc::cfgS("log/seed", "");
    m.seed = seed.empty() ? "unseeded" : seed;
    m.params_json = "{}";
    try {
      XmlRpc::XmlRpcValue params;
      if (ros::param::get("/agc", params)) m.params_json = xmlrpcToJson(params);
    } catch (const std::exception& e) {
      ROS_DEBUG("agc_run_logger: 读取 /agc 参数快照失败: %s", e.what());
    } catch (...) {
    }
    m.t_sim_start = agc::simNow().toSec();
    m.t_wall_start = agc::wallNow();
    return m;
  }

  ros::NodeHandle nh_;
  std::string root_;
  std::vector<ros::Subscriber> subs_;
  ros::Timer flush_timer_;
  RunRecorder* recorder_ = nullptr;
  int last_state_ = -1;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "agc_run_logger");
  AgcRunLogger logger;
  ros::spin();
  logger.shutdown();
  return 0;
}