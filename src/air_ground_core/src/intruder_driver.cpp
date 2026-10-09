// intruder_driver —— 目标 intruder_uav 运动驱动（M0 接口契约 v1.1）
// 基准：scripts/intruder_driver.py，C++ 版逐条对齐。
//
// - 按 scenario 的 target_start / target_motion 驱动目标，/agc/rates/drive_hz（20 Hz）
//   用 /gazebo/set_model_state 写位姿（reference_frame="world"，位置 + 四元数）。
// - static：保持 target_start；linear：沿 waypoints 以 speed_mps 匀速往返（累计弧长 pingpong）。
// - sinusoid：沿 heading_deg 匀速前进并叠加横向正弦偏移（amplitude_m/period_s）；
//   circle：以 target_start 为圆心、radius_m 为半径做匀速圆周。
// - 高度用 /agc/target/default_height_m。
// - /agc/target/enabled=false 时冻结（不写位姿）；/agc/target/reset_epoch 变化时相位/索引归零。
// - 每拍写目标轨迹到 <run_dir>/target_trajectory.jsonl；服务失败计数写 /agc/target/service_failures。
#include "agc/agc_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

namespace {

// 折线累计弧长；points=[(x,y),...]，返回长度与累计数组
std::vector<double> cumulativeLengths(const std::vector<std::pair<double, double>>& points) {
  std::vector<double> cum{0.0};
  for (size_t i = 1; i < points.size(); ++i) {
    cum.push_back(cum.back() + std::hypot(points[i].first - points[i - 1].first,
                                          points[i].second - points[i - 1].second));
  }
  return cum;
}

// 弧长 s 处的位置与切线偏航
void pointAtArc(const std::vector<std::pair<double, double>>& points,
                const std::vector<double>& cum, double s, double* x, double* y, double* yaw) {
  const double total = cum.back();
  if (total <= 0.0) {
    *x = points[0].first;
    *y = points[0].second;
    *yaw = 0.0;
    return;
  }
  s = std::min(std::max(s, 0.0), total);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    if (s <= cum[i + 1] || i + 2 == points.size()) {
      const double seg = cum[i + 1] - cum[i];
      const double t = (seg <= 0.0) ? 0.0 : (s - cum[i]) / seg;
      *x = points[i].first + t * (points[i + 1].first - points[i].first);
      *y = points[i].second + t * (points[i + 1].second - points[i].second);
      *yaw = std::atan2(points[i + 1].second - points[i].second,
                        points[i + 1].first - points[i].first);
      return;
    }
  }
  *x = points.back().first;
  *y = points.back().second;
  *yaw = 0.0;
}

// 往返运动：arc 单调增长，映射到折线并反转方向
void pingpongPose(const std::vector<std::pair<double, double>>& points,
                  const std::vector<double>& cum, double arc,
                  double* x, double* y, double* yaw, double* s_out) {
  const double total = cum.back();
  if (total <= 0.0) {
    *x = points[0].first;
    *y = points[0].second;
    *yaw = 0.0;
    *s_out = 0.0;
    return;
  }
  const double period = 2.0 * total;
  const double phase = std::fmod(arc, period);
  double s;
  bool fwd;
  if (phase <= total) {
    s = phase;
    fwd = true;
  } else {
    s = period - phase;
    fwd = false;
  }
  double yaw_fwd;
  pointAtArc(points, cum, s, x, y, &yaw_fwd);
  *yaw = fwd ? yaw_fwd : agc::normalizeAngle(yaw_fwd + M_PI);
  *s_out = s;
}

// Gazebo 服务 namespace：缺省 /gazebo，保证以 / 开头
std::string gzNs() {
  std::string ns = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
  if (ns.empty()) return "";
  while (!ns.empty() && ns.back() == '/') ns.pop_back();
  if (ns.empty() || ns.front() != '/') ns = "/" + ns;
  return ns;
}

void ensureDir(const std::string& path) {
  if (path.empty()) return;
  std::string cur;
  for (size_t i = 0; i < path.size(); ++i) {
    cur.push_back(path[i]);
    if (path[i] == '/' && cur.size() > 1) mkdir(cur.c_str(), 0755);
  }
  mkdir(path.c_str(), 0755);
}

// 服务失败计数自增
void bump(const std::string& param) {
  ros::param::set(param, agc::pgetI(param, 0) + 1);
}

struct Pose {
  double x, y, z, yaw, speed;
};

}  // namespace

class IntruderDriver {
 public:
  IntruderDriver() : nh_() {
    model_ = agc::pgetStr("/agc/gazebo/target_model", "intruder_uav");
    hz_ = agc::pgetD("/agc/rates/drive_hz", 20.0);
    height_ = agc::pgetD("/agc/target/default_height_m", 2.0);
    scenario_file_ = agc::pgetStr("/agc/scenario_file", "");

    const agc::Scenario sc = agc::loadScenario(scenario_file_);
    if (sc.loaded) {
      target_start_ = sc.target_start;
      mode_ = sc.target_mode;
      speed_ = sc.target_speed;
      heading_deg_ = sc.target_heading_deg;
      amplitude_ = sc.target_amplitude;
      period_ = sc.target_period;
      radius_ = sc.target_radius;
      waypoints_ = sc.waypoints;
    } else {
      // 场景读不到时的工程默认：起点在原点、静止、高度取 default_height_m
      target_start_ = {{0.0, 0.0, height_, 0.0}};
      mode_ = "static";
      speed_ = 0.0;
    }

    // 路径点：起点 + waypoints，去掉连续重复点
    points_.emplace_back(target_start_[0], target_start_[1]);
    for (const auto& wp : waypoints_) {
      if (std::hypot(wp.first - points_.back().first, wp.second - points_.back().second) < 1e-6) {
        continue;
      }
      points_.emplace_back(wp.first, wp.second);
    }
    cum_ = cumulativeLengths(points_);

    start_yaw_ = target_start_[3];
    last_reset_epoch_ = agc::pgetI("/agc/target/reset_epoch", 0);

    ensureDir(agc::runDir());

    const std::string svc = gzNs() + "/set_model_state";
    client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(svc);
    if (!agc::serviceReady(svc, 3.0)) {
      ROS_WARN("intruder_driver: set_model_state 暂不可用，运行中重试");
    }

    std::string pts;
    for (const auto& p : points_) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "(%.2f,%.2f)", p.first, p.second);
      pts += buf;
    }
    ROS_INFO("intruder_driver 启动：mode=%s speed_mps=%.3f height=%.1f "
             "heading_deg=%.1f amplitude_m=%.2f period_s=%.2f radius_m=%.2f waypoints=%s",
             mode_.c_str(), speed_, height_, heading_deg_, amplitude_, period_, radius_,
             pts.c_str());

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(hz_, 1e-3)),
                             &IntruderDriver::tick, this);
  }

 private:
  // 计算本拍目标位姿
  Pose pose(double dt) {
    if (mode_ == "linear" && speed_ > 0.0 && points_.size() >= 2) {
      arc_ += speed_ * dt;
      double x, y, yaw, s;
      pingpongPose(points_, cum_, arc_, &x, &y, &yaw, &s);
      return Pose{x, y, height_, yaw, speed_};
    }
    if (mode_ == "sinusoid") {
      arc_ += dt;  // arc_ 在此模式下即相位时间
      const double hd = heading_deg_ * M_PI / 180.0;
      const double c = std::cos(hd), sn = std::sin(hd);
      // 横向正弦偏移及其导数（period<=0 退化为直线：幅值/导数均为 0）
      double lat = 0.0, lat_dot = 0.0;
      if (period_ > 0.0) {
        const double w = 2.0 * M_PI / period_;
        lat = amplitude_ * std::sin(w * arc_);
        lat_dot = amplitude_ * w * std::cos(w * arc_);
      }
      const double fwd = speed_ * arc_;  // 沿航向基准前进距离
      // 位置 = 起点 + 航向前进 + 横向偏移
      const double x = target_start_[0] + c * fwd - sn * lat;
      const double y = target_start_[1] + sn * fwd + c * lat;
      // 速度 = 航向分量 speed + 横向分量 lat_dot，用于偏航与实际合速度
      const double vx = c * speed_ - sn * lat_dot;
      const double vy = sn * speed_ + c * lat_dot;
      return Pose{x, y, height_, std::atan2(vy, vx), std::hypot(vx, vy)};
    }
    if (mode_ == "circle" && radius_ > 0.0) {
      arc_ += dt;  // arc_ 在此模式下即相位时间
      const double theta = (speed_ / radius_) * arc_;
      const double x = target_start_[0] + radius_ * std::cos(theta);
      const double y = target_start_[1] + radius_ * std::sin(theta);
      return Pose{x, y, height_, theta + M_PI / 2.0, speed_};
    }
    // static：保持 target_start
    return Pose{target_start_[0], target_start_[1], height_, start_yaw_, 0.0};
  }

  bool write(const Pose& p) {
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = model_;
    srv.request.model_state.pose.position.x = p.x;
    srv.request.model_state.pose.position.y = p.y;
    srv.request.model_state.pose.position.z = p.z;
    const std::array<double, 4> q = agc::quatFromYaw(p.yaw);
    srv.request.model_state.pose.orientation.x = q[0];
    srv.request.model_state.pose.orientation.y = q[1];
    srv.request.model_state.pose.orientation.z = q[2];
    srv.request.model_state.pose.orientation.w = q[3];
    srv.request.model_state.reference_frame = agc::FRAME_WORLD;
    if (!client_.call(srv)) return false;
    return srv.response.success;
  }

  // 轮询 /agc/scenario_file：变化即重载目标运动（mode/speed/waypoints）并重建路径
  void reloadScenario() {
    const std::string f = agc::pgetStr("/agc/scenario_file", "");
    if (f == scenario_file_) return;
    scenario_file_ = f;
    const agc::Scenario sc = agc::loadScenario(scenario_file_);
    if (!sc.loaded) {
      ROS_WARN("intruder_driver: 场景文件变更后无法读取 %s，保持原运动", f.c_str());
      return;
    }
    target_start_ = sc.target_start;
    start_yaw_ = target_start_[3];
    mode_ = sc.target_mode;
    speed_ = sc.target_speed;
    heading_deg_ = sc.target_heading_deg;
    amplitude_ = sc.target_amplitude;
    period_ = sc.target_period;
    radius_ = sc.target_radius;
    waypoints_ = sc.waypoints;
    // 路径点：起点 + waypoints，去掉连续重复点
    points_.clear();
    points_.emplace_back(target_start_[0], target_start_[1]);
    for (const auto& wp : waypoints_) {
      if (std::hypot(wp.first - points_.back().first, wp.second - points_.back().second) < 1e-6)
        continue;
      points_.emplace_back(wp.first, wp.second);
    }
    cum_ = cumulativeLengths(points_);
    arc_ = 0.0;  // 相位归零
    ROS_INFO("intruder_driver: 场景文件变更 %s，重载 mode=%s speed=%.3f "
             "heading=%.1f amplitude=%.2f period=%.2f radius=%.2f（相位归零）",
             f.c_str(), mode_.c_str(), speed_, heading_deg_, amplitude_, period_, radius_);
  }

  void tick(const ros::TimerEvent&) {
    reloadScenario();
    const bool enabled = agc::pgetB("/agc/target/enabled", true);
    const int epoch = agc::pgetI("/agc/target/reset_epoch", 0);
    if (epoch != last_reset_epoch_) {  // 复位序号变化 -> 相位清零
      last_reset_epoch_ = epoch;
      arc_ = 0.0;
      ROS_INFO("intruder_driver: 检测到 reset_epoch=%d，内部相位归零", epoch);
    }

    const double t = agc::simNow().toSec();
    if (t <= 0.0) {
      ROS_WARN_THROTTLE(5.0, "intruder_driver: 仿真时钟未就绪，等待 /clock");
      return;
    }
    double dt = has_last_t_ ? (t - last_t_) : 0.0;
    last_t_ = t;
    has_last_t_ = true;
    if (dt <= 0.0 || dt > 1.0) dt = 0.0;  // 时钟停滞或跳变：本拍不推进

    const Pose p = pose(dt);

    if (!enabled) {
      ROS_INFO_THROTTLE(5.0, "intruder_driver: target.enabled=false，冻结目标位姿");
    } else if (!write(p)) {
      bump("/agc/target/service_failures");
      ROS_WARN_THROTTLE(2.0, "intruder_driver: set_model_state 失败");
    }

    agc::appendJsonl("target_trajectory.jsonl",
                     agc::JsonObj()
                         .n("t_sim", t)
                         .n("x", p.x)
                         .n("y", p.y)
                         .n("z", p.z)
                         .n("yaw", p.yaw)
                         .n("speed", p.speed));
  }

  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  ros::Timer timer_;

  std::string model_, mode_;
  std::string scenario_file_;  // 场景文件缓存（变化即重载运动与路径）
  double hz_ = 20.0, height_ = 2.0, speed_ = 0.0, start_yaw_ = 0.0;
  // 机动扩展参数（仅 sinusoid/circle 使用）
  double heading_deg_ = 0.0, amplitude_ = 0.0, period_ = 0.0, radius_ = 0.0;
  std::array<double, 4> target_start_{{0.0, 0.0, 2.0, 0.0}};
  std::vector<std::pair<double, double>> waypoints_, points_;
  std::vector<double> cum_;

  double arc_ = 0.0;
  int last_reset_epoch_ = 0;
  bool has_last_t_ = false;
  double last_t_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "intruder_driver");
  IntruderDriver node;
  ros::spin();
  return 0;
}