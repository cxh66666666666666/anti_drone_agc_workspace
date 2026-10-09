// agc_common.hpp —— 空地协同层 C++ 共享基础模块（M0 接口契约 v1.1）
//
// 与 Python 参考实现 scripts/agc_common.py 保持同一套语义：
// 帧常量、参数读取、仿真/墙钟时间、四元数与角度、文件哈希、场景 YAML、
// 点到 OBB 净空、run 日志目录与 JSONL、极简 JSON 写入器、TF/服务工具。
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

namespace agc {

const double INF = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------- 帧常量
extern const char* const FRAME_WORLD;      // "world"
extern const char* const FRAME_AIR_LOCAL;  // "air_local"
extern const char* const FRAME_DRONE;      // "drone/base_link"
extern const char* const FRAME_CAR;        // "car/base_link"
extern const char* const FRAME_TARGET;     // "target/base_link"

// ---------------------------------------------------------------- 任务状态
// 与 air_ground_core/AgcTaskState.msg 的常量一致；不能依赖 python 消息包
const int32_t ST_IDLE = 0;
const int32_t ST_AIR_SEARCH = 1;
const int32_t ST_TARGET_ACQUIRED = 2;
const int32_t ST_GROUND_APPROACH = 3;
const int32_t ST_TRACKING_SUCCESS = 4;
const int32_t ST_MISSION_FAILED = 5;
const int32_t ST_ABORTED = 6;
const int32_t ST_INTERCEPT_CONFIRMED = 7;  // 阶段 B 保留，阶段 A 禁用
const int32_t ST_INFRA_FAILURE = 8;

bool isTerminal(int32_t state);
std::string stateName(int32_t state);

// ---------------------------------------------------------------- 参数
double pgetD(const std::string& path, double def);
int pgetI(const std::string& path, int def);
bool pgetB(const std::string& path, bool def);
std::string pgetStr(const std::string& path, const std::string& def);

inline double cfgD(const std::string& rel, double def) { return pgetD("/agc/" + rel, def); }
inline int cfgI(const std::string& rel, int def) { return pgetI("/agc/" + rel, def); }
inline bool cfgB(const std::string& rel, bool def) { return pgetB("/agc/" + rel, def); }
inline std::string cfgS(const std::string& rel, const std::string& def) {
  return pgetStr("/agc/" + rel, def);
}

// ---------------------------------------------------------------- 时间
ros::Time simNow();          // 仿真时钟（依赖 /clock）
double wallNow();            // 单调墙钟，仅用于 watchdog 与日志
double stampAge(const ros::Time& t);  // 年龄秒；t 为 0 时返回 INF
bool serviceReady(const std::string& name, double timeout);  // 查 master，绕开 wait_for_service 返回值坑

// ---------------------------------------------------------------- 四元数 / 角度
std::array<double, 4> quatFromYaw(double yaw);  // (x, y, z, w)
double yawFromQuat(double x, double y, double z, double w);
double yawFromQuat(const geometry_msgs::Quaternion& q);
double normalizeAngle(double a);
std::array<double, 3> rotateVecByQuat(const std::array<double, 4>& q,
                                      const std::array<double, 3>& v);
std::array<std::array<double, 3>, 3> quatToMatrix(const std::array<double, 4>& q);
std::array<double, 3> matTVec(const std::array<std::array<double, 3>, 3>& m,
                              const std::array<double, 3>& v);
double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b);

// ---------------------------------------------------------------- 哈希
std::string sha1File(const std::string& path);  // 读不到返回 "missing"
std::string sha1Text(const std::string& text);

// ---------------------------------------------------------------- 场景
struct Obstacle {
  std::string name;
  double x = 0, y = 0, z = 0, yaw = 0;
  double sx = 0, sy = 0, sz = 0;
  double hx() const { return sx * 0.5; }
  double hy() const { return sy * 0.5; }
};

struct Scenario {
  bool loaded = false;
  std::string id;
  std::array<double, 4> car_start{{0.0, 0.0, 0.11, 0.0}};     // x, y, z, yaw
  std::array<double, 4> target_start{{6.5, 2.5, 2.0, 0.0}};   // x, y, z, yaw
  std::string target_mode = "static";
  double target_speed = 0.0;
  // 机动扩展参数（sinusoid/circle 使用，默认 0 即退化为直线/静止）
  double target_heading_deg = 0.0;  // 机动航向
  double target_amplitude = 0.0;    // 正弦横向幅值
  double target_period = 0.0;       // 正弦周期
  double target_radius = 0.0;       // 圆周半径
  std::vector<std::pair<double, double>> waypoints;
  std::vector<Obstacle> obstacles;
  // map
  double map_resolution = 0.5, map_sx = 40.0, map_sy = 40.0;
  double map_ox = -20.0, map_oy = -20.0;
  bool map_unknown_is_obstacle = true;
  // thresholds（run 前锁定）
  double approach_radius = 3.0, hold_window = 5.0, confidence_min = 0.5;
  double search_timeout = 60.0, approach_timeout = 60.0, mission_timeout = 120.0;
  double lost_tol = 2.0;
};

Scenario loadScenario(const std::string& path);

// 点到障碍 OBB 表面的平面距离；点在内部返回负值
double pointObbClearance(double px, double py, const Obstacle& o);
// 全部障碍中的最小净空；无障碍返回 INF
double minClearance(double px, double py, const std::vector<Obstacle>& obs);
// 车体（半径 r 的圆）与非目标障碍是否几何相交
bool circleIntersectsObstacle(double px, double py, double r, const Obstacle& o);

// ---------------------------------------------------------------- 日志
std::string runDir();     // /agc/log/run_dir，缺省回退 /agc/log/root
std::string logRoot();    // /agc/log/root
void appendJsonl(const std::string& filename, const std::string& json_line);
void appendJsonl(const std::string& filename, const class JsonObj& obj);

// ---------------------------------------------------------------- 极简 JSON 写入器
// 只做「按插入顺序输出对象」的最小实现，够写 run 日志与 meta/summary 用。
class JsonObj {
 public:
  JsonObj& v(const std::string& key, const std::string& value);  // 字符串（自动转义）
  JsonObj& n(const std::string& key, double value);              // 数值（NaN/Inf -> 字符串）
  JsonObj& iv(const std::string& key, long long value);
  JsonObj& bv(const std::string& key, bool value);
  JsonObj& raw(const std::string& key, const std::string& json);  // 已是合法 JSON
  JsonObj& nullv(const std::string& key);
  std::string dump() const;

  static std::string quote(const std::string& s);
  static std::string num(double v);
  static std::string arr(const std::vector<std::string>& pre_rendered_items);
  static std::string arrStr(const std::vector<std::string>& values);
  static std::string arrNum(const std::vector<double>& values);

 private:
  std::vector<std::pair<std::string, std::string>> items_;
};

// ---------------------------------------------------------------- TF
void sendTf(const ros::Time& stamp, const std::string& parent, const std::string& child,
            const std::array<double, 3>& pos, const std::array<double, 4>& quat);
void sendStaticTf(const std::string& parent, const std::string& child,
                  const std::array<double, 3>& pos, const std::array<double, 4>& quat);

// ---------------------------------------------------------------- 消息 -> JSON（供 agc_run_logger 使用）
std::string msgJson(const nav_msgs::Odometry& m);
std::string msgJson(const geometry_msgs::PoseStamped& m);
std::string msgJson(const nav_msgs::Path& m);
std::string msgJson(const nav_msgs::OccupancyGrid& m);
std::string msgJson(const geometry_msgs::TwistStamped& m);

}  // namespace agc