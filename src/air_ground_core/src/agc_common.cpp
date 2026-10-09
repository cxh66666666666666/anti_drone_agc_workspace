// agc_common.cpp —— 共享基础模块实现（与 scripts/agc_common.py 语义一致）
#include "agc/agc_common.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <sstream>

#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include <rosgraph_msgs/Clock.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace agc {

const char* const FRAME_WORLD = "world";
const char* const FRAME_AIR_LOCAL = "air_local";
const char* const FRAME_DRONE = "drone/base_link";
const char* const FRAME_CAR = "car/base_link";
const char* const FRAME_TARGET = "target/base_link";

// ---------------------------------------------------------------- 状态
bool isTerminal(int32_t state) {
  return state == ST_TRACKING_SUCCESS || state == ST_MISSION_FAILED ||
         state == ST_ABORTED || state == ST_INFRA_FAILURE;
}

std::string stateName(int32_t state) {
  switch (state) {
    case ST_IDLE: return "IDLE";
    case ST_AIR_SEARCH: return "AIR_SEARCH";
    case ST_TARGET_ACQUIRED: return "TARGET_ACQUIRED";
    case ST_GROUND_APPROACH: return "GROUND_APPROACH";
    case ST_TRACKING_SUCCESS: return "TRACKING_SUCCESS";
    case ST_MISSION_FAILED: return "MISSION_FAILED";
    case ST_ABORTED: return "ABORTED";
    case ST_INTERCEPT_CONFIRMED: return "INTERCEPT_CONFIRMED";
    case ST_INFRA_FAILURE: return "INFRA_FAILURE";
    default: return "UNKNOWN";
  }
}

// ---------------------------------------------------------------- 参数
double pgetD(const std::string& path, double def) {
  double v = def;
  if (!ros::param::get(path, v)) return def;
  return v;
}
int pgetI(const std::string& path, int def) {
  int v = def;
  if (!ros::param::get(path, v)) return def;
  return v;
}
bool pgetB(const std::string& path, bool def) {
  bool v = def;
  if (!ros::param::get(path, v)) return def;
  return v;
}
std::string pgetStr(const std::string& path, const std::string& def) {
  std::string v;
  if (!ros::param::get(path, v)) return def;
  return v;
}

// ---------------------------------------------------------------- 时间
ros::Time simNow() { return ros::Time::now(); }

double wallNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

double stampAge(const ros::Time& t) {
  if (t.isZero()) return INF;
  return (simNow() - t).toSec();
}

bool serviceReady(const std::string& name, double timeout) {
  // 直接查 master 服务表：不用 wait_for_service 的返回值（/use_sim_time=true
  // 时它在多个场景下立即返回，无法区分就绪与未就绪）。
  // master 查询在 master 不可达时会抛 XmlRpc 异常，必须整体兜住，否则节点会被 abort。
  const double deadline = wallNow() + std::max(timeout, 0.0);
  while (true) {
    bool ok = false;
    try {
      // ros::master::execute 的第二个出参是完整 XML-RPC 响应数组 [code, msg, value]，
      // 其 bool 返回值即 (code == 1)；不能把该数组当成 int 状态码来判断。
      XmlRpc::XmlRpcValue args, response, payload;
      args[0] = ros::this_node::getName();
      args[1] = name;
      ok = ros::master::execute("lookupService", args, response, payload, false) &&
           payload.valid() && payload.getType() == XmlRpc::XmlRpcValue::TypeString;
    } catch (const std::exception& e) {
      ROS_DEBUG("serviceReady: master 查询异常(%s): %s", name.c_str(), e.what());
      ok = false;
    } catch (...) {
      ok = false;
    }
    if (ok) return true;
    if (wallNow() >= deadline) return false;
    ros::WallDuration(0.05).sleep();
  }
}

// ---------------------------------------------------------------- 四元数 / 角度
std::array<double, 4> quatFromYaw(double yaw) {
  return {{0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)}};
}

double yawFromQuat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double yawFromQuat(const geometry_msgs::Quaternion& q) {
  return yawFromQuat(q.x, q.y, q.z, q.w);
}

double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

std::array<double, 3> rotateVecByQuat(const std::array<double, 4>& q,
                                      const std::array<double, 3>& v) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double vx = v[0], vy = v[1], vz = v[2];
  const double tx = 2.0 * (y * vz - z * vy);
  const double ty = 2.0 * (z * vx - x * vz);
  const double tz = 2.0 * (x * vy - y * vx);
  return {{vx + w * tx + (y * tz - z * ty),
           vy + w * ty + (z * tx - x * tz),
           vz + w * tz + (x * ty - y * tx)}};
}

std::array<std::array<double, 3>, 3> quatToMatrix(const std::array<double, 4>& q) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double xx = x * x, yy = y * y, zz = z * z;
  const double xy = x * y, xz = x * z, yz = y * z;
  const double wx = w * x, wy = w * y, wz = w * z;
  return {{{1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)},
           {2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)},
           {2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)}}};
}

std::array<double, 3> matTVec(const std::array<std::array<double, 3>, 3>& m,
                              const std::array<double, 3>& v) {
  return {{m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
           m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
           m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2]}};
}

double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ---------------------------------------------------------------- 哈希
std::string sha1File(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) return "missing";
  std::ostringstream ss;
  ss << f.rdbuf();
  return sha1Text(ss.str());
}

std::string sha1Text(const std::string& text) {
  unsigned char digest[SHA_DIGEST_LENGTH];
  SHA1(reinterpret_cast<const unsigned char*>(text.data()), text.size(), digest);
  static const char* hex = "0123456789abcdef";
  std::string out;
  out.reserve(SHA_DIGEST_LENGTH * 2);
  for (int i = 0; i < SHA_DIGEST_LENGTH; ++i) {
    out.push_back(hex[(digest[i] >> 4) & 0xF]);
    out.push_back(hex[digest[i] & 0xF]);
  }
  return out;
}

// ---------------------------------------------------------------- 场景
namespace {

template <typename T>
void readSeq4(const YAML::Node& n, std::array<T, 4>* out) {
  if (!n || !n.IsSequence() || n.size() < 1) return;
  for (size_t i = 0; i < out->size() && i < n.size(); ++i) {
    (*out)[i] = n[i].as<T>();
  }
}

double seqAt(const YAML::Node& n, size_t i, double def) {
  if (!n || !n.IsSequence() || i >= n.size()) return def;
  try {
    return n[i].as<double>();
  } catch (...) {
    return def;
  }
}

}  // namespace

Scenario loadScenario(const std::string& path) {
  Scenario s;
  if (path.empty()) return s;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    ROS_ERROR("agc_common: 读取场景文件失败 %s: %s", path.c_str(), e.what());
    return s;
  }
  if (!root || !root.IsMap()) return s;

  s.loaded = true;
  s.id = root["scenario_id"] ? root["scenario_id"].as<std::string>() : "unknown";
  readSeq4<double>(root["car_start"], &s.car_start);
  readSeq4<double>(root["target_start"], &s.target_start);

  if (root["target_motion"] && root["target_motion"].IsMap()) {
    const YAML::Node tm = root["target_motion"];
    if (tm["mode"]) s.target_mode = tm["mode"].as<std::string>();
    if (tm["speed_mps"]) s.target_speed = tm["speed_mps"].as<double>();
    // 机动扩展（可选，缺失时用默认值保持原行为）
    if (tm["heading_deg"]) s.target_heading_deg = tm["heading_deg"].as<double>();
    if (tm["amplitude_m"]) s.target_amplitude = tm["amplitude_m"].as<double>();
    if (tm["period_s"]) s.target_period = tm["period_s"].as<double>();
    if (tm["radius_m"]) s.target_radius = tm["radius_m"].as<double>();
    if (tm["waypoints"] && tm["waypoints"].IsSequence()) {
      for (const auto& wp : tm["waypoints"]) {
        s.waypoints.emplace_back(seqAt(wp, 0, 0.0), seqAt(wp, 1, 0.0));
      }
    }
  }

  if (root["obstacles"] && root["obstacles"].IsSequence()) {
    for (const auto& o : root["obstacles"]) {
      Obstacle ob;
      if (o["name"]) ob.name = o["name"].as<std::string>();
      const YAML::Node pose = o["pose"], size = o["size"];
      ob.x = seqAt(pose, 0, 0.0);
      ob.y = seqAt(pose, 1, 0.0);
      ob.z = seqAt(pose, 2, 0.0);
      ob.yaw = seqAt(pose, 5, 0.0);
      ob.sx = seqAt(size, 0, 0.0);
      ob.sy = seqAt(size, 1, 0.0);
      ob.sz = seqAt(size, 2, 0.0);
      s.obstacles.push_back(ob);
    }
  }

  if (root["map"] && root["map"].IsMap()) {
    const YAML::Node m = root["map"];
    if (m["resolution_m"]) s.map_resolution = m["resolution_m"].as<double>();
    if (m["size_x_m"]) s.map_sx = m["size_x_m"].as<double>();
    if (m["size_y_m"]) s.map_sy = m["size_y_m"].as<double>();
    if (m["origin_x"]) s.map_ox = m["origin_x"].as<double>();
    if (m["origin_y"]) s.map_oy = m["origin_y"].as<double>();
    if (m["unknown_is_obstacle"]) s.map_unknown_is_obstacle = m["unknown_is_obstacle"].as<bool>();
  }

  if (root["thresholds"] && root["thresholds"].IsMap()) {
    const YAML::Node t = root["thresholds"];
    if (t["approach_radius_m"]) s.approach_radius = t["approach_radius_m"].as<double>();
    if (t["hold_window_s"]) s.hold_window = t["hold_window_s"].as<double>();
    if (t["confidence_min"]) s.confidence_min = t["confidence_min"].as<double>();
    if (t["search_timeout_s"]) s.search_timeout = t["search_timeout_s"].as<double>();
    if (t["approach_timeout_s"]) s.approach_timeout = t["approach_timeout_s"].as<double>();
    if (t["mission_timeout_s"]) s.mission_timeout = t["mission_timeout_s"].as<double>();
    if (t["target_lost_tolerance_s"]) s.lost_tol = t["target_lost_tolerance_s"].as<double>();
  }
  return s;
}

double pointObbClearance(double px, double py, const Obstacle& o) {
  const double dx = px - o.x, dy = py - o.y;
  const double c = std::cos(-o.yaw), s = std::sin(-o.yaw);
  const double lx = c * dx - s * dy;
  const double ly = s * dx + c * dy;
  const double ex = std::fabs(lx) - o.hx();
  const double ey = std::fabs(ly) - o.hy();
  if (ex > 0.0 && ey > 0.0) return std::sqrt(ex * ex + ey * ey);
  return std::max(ex, ey);
}

double minClearance(double px, double py, const std::vector<Obstacle>& obs) {
  double best = INF;
  for (const auto& o : obs) best = std::min(best, pointObbClearance(px, py, o));
  return best;
}

bool circleIntersectsObstacle(double px, double py, double r, const Obstacle& o) {
  return pointObbClearance(px, py, o) <= r;
}

// ---------------------------------------------------------------- 日志
std::string logRoot() { return cfgS("log/root", "/home/ubuntu/anti_drone_agc_workspace/experiments/agc_runs"); }

std::string runDir() {
  ros::NodeHandle nh;
  std::string d;
  if (nh.getParam("/agc/log/run_dir", d) && !d.empty()) return d;
  return logRoot();
}

void appendJsonl(const std::string& filename, const std::string& json_line) {
  const std::string path = runDir() + "/" + filename;
  std::ofstream f(path, std::ios::app);
  if (!f.is_open()) {
    ROS_WARN_THROTTLE(5.0, "agc_common: 写日志失败 %s", path.c_str());
    return;
  }
  f << json_line << "\n";
}

void appendJsonl(const std::string& filename, const JsonObj& obj) {
  appendJsonl(filename, obj.dump());
}

// ---------------------------------------------------------------- JSON
std::string JsonObj::quote(const std::string& s) {
  std::string out = "\"";
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  out += "\"";
  return out;
}

std::string JsonObj::num(double v) {
  if (std::isnan(v)) return "\"NaN\"";
  if (std::isinf(v)) return v > 0 ? "\"Infinity\"" : "\"-Infinity\"";
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

std::string JsonObj::arr(const std::vector<std::string>& items) {
  std::string out = "[";
  for (size_t i = 0; i < items.size(); ++i) {
    if (i) out += ", ";
    out += items[i];
  }
  out += "]";
  return out;
}

std::string JsonObj::arrStr(const std::vector<std::string>& values) {
  std::vector<std::string> items;
  items.reserve(values.size());
  for (const auto& v : values) items.push_back(quote(v));
  return arr(items);
}

std::string JsonObj::arrNum(const std::vector<double>& values) {
  std::vector<std::string> items;
  items.reserve(values.size());
  for (double v : values) items.push_back(num(v));
  return arr(items);
}

JsonObj& JsonObj::v(const std::string& key, const std::string& value) {
  items_.emplace_back(key, quote(value));
  return *this;
}
JsonObj& JsonObj::n(const std::string& key, double value) {
  items_.emplace_back(key, num(value));
  return *this;
}
JsonObj& JsonObj::iv(const std::string& key, long long value) {
  items_.emplace_back(key, std::to_string(value));
  return *this;
}
JsonObj& JsonObj::bv(const std::string& key, bool value) {
  items_.emplace_back(key, value ? "true" : "false");
  return *this;
}
JsonObj& JsonObj::raw(const std::string& key, const std::string& json) {
  items_.emplace_back(key, json.empty() ? "null" : json);
  return *this;
}
JsonObj& JsonObj::nullv(const std::string& key) {
  items_.emplace_back(key, "null");
  return *this;
}

std::string JsonObj::dump() const {
  std::string out = "{";
  for (size_t i = 0; i < items_.size(); ++i) {
    if (i) out += ", ";
    out += quote(items_[i].first) + ": " + items_[i].second;
  }
  out += "}";
  return out;
}

// ---------------------------------------------------------------- TF
void sendTf(const ros::Time& stamp, const std::string& parent, const std::string& child,
            const std::array<double, 3>& pos, const std::array<double, 4>& quat) {
  static tf2_ros::TransformBroadcaster* broadcaster = nullptr;
  if (broadcaster == nullptr) broadcaster = new tf2_ros::TransformBroadcaster();
  geometry_msgs::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = pos[0];
  t.transform.translation.y = pos[1];
  t.transform.translation.z = pos[2];
  t.transform.rotation.x = quat[0];
  t.transform.rotation.y = quat[1];
  t.transform.rotation.z = quat[2];
  t.transform.rotation.w = quat[3];
  broadcaster->sendTransform(t);
}

void sendStaticTf(const std::string& parent, const std::string& child,
                  const std::array<double, 3>& pos, const std::array<double, 4>& quat) {
  static tf2_ros::StaticTransformBroadcaster* broadcaster = nullptr;
  if (broadcaster == nullptr) broadcaster = new tf2_ros::StaticTransformBroadcaster();
  geometry_msgs::TransformStamped t;
  t.header.stamp = simNow();
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = pos[0];
  t.transform.translation.y = pos[1];
  t.transform.translation.z = pos[2];
  t.transform.rotation.x = quat[0];
  t.transform.rotation.y = quat[1];
  t.transform.rotation.z = quat[2];
  t.transform.rotation.w = quat[3];
  broadcaster->sendTransform(t);
}

// ---------------------------------------------------------------- 消息 -> JSON
namespace {

std::string headerJson(const std_msgs::Header& h) {
  // 与 Python 参考实现一致：Time 序列化为「秒」浮点数（agc_run_logger.py msg_to_dict）
  JsonObj o;
  o.iv("seq", h.seq).n("stamp", h.stamp.toSec()).v("frame_id", h.frame_id);
  return o.dump();
}

std::string pointJson(double x, double y, double z) {
  JsonObj o;
  o.n("x", x).n("y", y).n("z", z);
  return o.dump();
}

std::string quatJson(double x, double y, double z, double w) {
  JsonObj o;
  o.n("x", x).n("y", y).n("z", z).n("w", w);
  return o.dump();
}

std::string poseJson(const geometry_msgs::Pose& p) {
  JsonObj o;
  o.raw("position", pointJson(p.position.x, p.position.y, p.position.z));
  o.raw("orientation", quatJson(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
  return o.dump();
}

std::string twistJson(const geometry_msgs::Twist& t) {
  JsonObj o;
  o.raw("linear", pointJson(t.linear.x, t.linear.y, t.linear.z));
  o.raw("angular", pointJson(t.angular.x, t.angular.y, t.angular.z));
  return o.dump();
}

}  // namespace

std::string msgJson(const nav_msgs::Odometry& m) {
  JsonObj o;
  o.raw("header", headerJson(m.header)).v("child_frame_id", m.child_frame_id);
  JsonObj pose;
  pose.raw("pose", poseJson(m.pose.pose));
  pose.raw("covariance", JsonObj::arrNum(std::vector<double>(m.pose.covariance.begin(),
                                                             m.pose.covariance.end())));
  o.raw("pose", pose.dump());
  JsonObj tw;
  tw.raw("twist", twistJson(m.twist.twist));
  tw.raw("covariance", JsonObj::arrNum(std::vector<double>(m.twist.covariance.begin(),
                                                           m.twist.covariance.end())));
  o.raw("twist", tw.dump());
  return o.dump();
}

std::string msgJson(const geometry_msgs::PoseStamped& m) {
  JsonObj o;
  o.raw("header", headerJson(m.header)).raw("pose", poseJson(m.pose));
  return o.dump();
}

std::string msgJson(const geometry_msgs::TwistStamped& m) {
  JsonObj o;
  o.raw("header", headerJson(m.header)).raw("twist", twistJson(m.twist));
  return o.dump();
}

std::string msgJson(const nav_msgs::Path& m) {
  std::vector<std::string> poses;
  poses.reserve(m.poses.size());
  for (const auto& p : m.poses) {
    JsonObj o;
    o.raw("header", headerJson(p.header)).raw("pose", poseJson(p.pose));
    poses.push_back(o.dump());
  }
  JsonObj o;
  o.raw("header", headerJson(m.header)).raw("poses", JsonObj::arr(poses));
  return o.dump();
}

std::string msgJson(const nav_msgs::OccupancyGrid& m) {
  // 与 Python 参考实现同结构：info{map_load_time, resolution, width, height, origin{position,orientation}}
  JsonObj info;
  info.n("map_load_time", m.info.map_load_time.toSec())
      .n("resolution", m.info.resolution)
      .iv("width", m.info.width)
      .iv("height", m.info.height);
  JsonObj origin;
  origin.raw("position", pointJson(m.info.origin.position.x, m.info.origin.position.y,
                                  m.info.origin.position.z));
  origin.raw("orientation", quatJson(m.info.origin.orientation.x, m.info.origin.orientation.y,
                                     m.info.origin.orientation.z, m.info.origin.orientation.w));
  info.raw("origin", origin.dump());
  std::vector<double> data;
  data.reserve(m.data.size());
  for (int8_t c : m.data) data.push_back(static_cast<double>(c));
  JsonObj o;
  o.raw("header", headerJson(m.header)).raw("info", info.dump());
  o.raw("data", JsonObj::arrNum(data));
  return o.dump();
}

}  // namespace agc