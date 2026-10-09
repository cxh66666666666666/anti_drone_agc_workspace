// ground_commander —— 地面速度指令生成（M0 接口契约 v1.1）。
//
// 与 scripts/ground_commander.py 语义逐条对齐：
// - /agc/ground/cmd_vel（geometry_msgs/TwistStamped）唯一发布者，frame_id=car/base_link，
//   20 Hz 每拍都发（含零速）。
// - 纯跟踪：预瞄点取路径投影点前方 lookahead_m；
//   angular.z = k_heading*heading_err + k_lateral*cross_track（按 max_angular 限幅）；
//   linear.x = max_linear_speed * max(0, cos(heading_err))（按 max_linear 限幅）。
// - 路径进度索引只增不减，避免倒车。
// - 过期/终态/不可达/急停/无路径/未进入 GROUND_APPROACH -> 立即零速并记录具体原因。
#include <cmath>
#include <algorithm>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <air_ground_core/AgcTaskState.h>

#include "agc/agc_common.hpp"

namespace {

using Pt2 = std::array<double, 2>;

// 点 p 在线段 a-b 上的投影；返回 (投影点, 弧参数 t, 距离)
Pt2 projectOnSegment(const Pt2& p, const Pt2& a, const Pt2& b, double* t_out, double* d_out) {
  const double ax = a[0], ay = a[1];
  const double vx = b[0] - ax, vy = b[1] - ay;
  const double L2 = vx * vx + vy * vy;
  if (L2 <= 1e-12) {
    *t_out = 0.0;
    *d_out = std::hypot(p[0] - ax, p[1] - ay);
    return {{ax, ay}};
  }
  double t = ((p[0] - ax) * vx + (p[1] - ay) * vy) / L2;
  t = std::min(std::max(t, 0.0), 1.0);
  const Pt2 proj{{ax + t * vx, ay + t * vy}};
  *t_out = t;
  *d_out = std::hypot(p[0] - proj[0], p[1] - proj[1]);
  return proj;
}

// 累积弧长
std::vector<double> cumulative(const std::vector<Pt2>& pts) {
  std::vector<double> cum{0.0};
  for (size_t i = 1; i < pts.size(); ++i) {
    cum.push_back(cum.back() + std::hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  }
  return cum;
}

// 取弧长 s 处的点与其所在段单位方向
std::pair<Pt2, Pt2> pointAtArc(const std::vector<Pt2>& pts, const std::vector<double>& cum, double s) {
  const double total = cum.back();
  if (total <= 0.0) return {pts[0], {{1.0, 0.0}}};
  s = std::min(std::max(s, 0.0), total);
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    if (s <= cum[i + 1] || i == pts.size() - 2) {
      const double seg = cum[i + 1] - cum[i];
      const double t = seg <= 0.0 ? 0.0 : (s - cum[i]) / seg;
      const Pt2 p{{pts[i][0] + t * (pts[i + 1][0] - pts[i][0]),
                   pts[i][1] + t * (pts[i + 1][1] - pts[i][1])}};
      double dx = pts[i + 1][0] - pts[i][0], dy = pts[i + 1][1] - pts[i][1];
      double n = std::hypot(dx, dy);
      if (n == 0.0) n = 1.0;
      return {p, {{dx / n, dy / n}}};
    }
  }
  return {pts.back(), {{1.0, 0.0}}};
}

}  // namespace

class GroundCommander {
 public:
  GroundCommander() : nh_() {
    lookahead_ = agc::cfgD("commander/lookahead_m", 0.6);
    k_heading_ = agc::cfgD("commander/k_heading", 1.5);
    k_lateral_ = agc::cfgD("commander/k_lateral", 0.8);
    max_w_ = agc::cfgD("commander/max_angular_speed_radps", 1.2);
    max_v_ = agc::cfgD("commander/max_linear_speed_mps", 1.0);
    goal_tol_ = agc::cfgD("commander/goal_tolerance_m", 0.5);
    goal_stop_ = agc::cfgD("commander/goal_stop_distance_m", 0.30);
    cmd_hz_ = agc::cfgD("rates/cmd_hz", 20.0);
    path_max_age_ = agc::cfgD("freshness/path_max_age_s", 2.0);
    goal_max_age_ = agc::cfgD("freshness/goal_max_age_s", 1.0);
    odom_max_age_ = agc::cfgD("freshness/odom_max_age_s", 0.5);

    path_sub_ = nh_.subscribe("/agc/ground/path", 1, &GroundCommander::pathCb, this);
    goal_sub_ = nh_.subscribe("/agc/ground/goal", 1, &GroundCommander::goalCb, this);
    odom_sub_ = nh_.subscribe("/agc/ground/odom", 1, &GroundCommander::odomCb, this);
    state_sub_ = nh_.subscribe("/agc/task/state", 1, &GroundCommander::stateCb, this);
    pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/agc/ground/cmd_vel", 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / cmd_hz_), &GroundCommander::tick, this);
    ROS_INFO("ground_commander 就绪：lookahead=%.2f k_heading=%.2f k_lateral=%.2f",
             lookahead_, k_heading_, k_lateral_);
  }

 private:
  // ------------------------------------------------------------ 回调
  void pathCb(const nav_msgs::Path::ConstPtr& msg) {
    // 新路径（stamp 或点数变化）时重置进度索引
    const std::tuple<uint32_t, uint32_t, size_t> sid(
        msg->header.stamp.sec, msg->header.stamp.nsec, msg->poses.size());
    if (!have_path_id_ || sid != path_stamp_id_) {
      path_stamp_id_ = sid;
      have_path_id_ = true;
      idx_ = 0;
    }
    path_msg_ = msg;
  }
  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) { goal_msg_ = msg; }
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) { odom_msg_ = msg; }
  void stateCb(const air_ground_core::AgcTaskState::ConstPtr& msg) { task_state_ = msg; }

  // ------------------------------------------------------------ 纯跟踪
  struct Pursuit {
    double v = 0.0, w = 0.0, cross_track = 0.0, heading_err = 0.0;
  };

  Pursuit pursuit(const std::array<double, 3>& pose, const std::vector<Pt2>& pts) {
    Pursuit r;
    const double x = pose[0], y = pose[1], yaw = pose[2];
    const std::vector<double> cum = cumulative(pts);
    const int n = static_cast<int>(pts.size());
    // 只从当前进度索引往后找投影，保证索引单调不减
    const int stop = std::max(n - 1, 1);
    bool found = false;
    int best_i = 0;
    Pt2 best_proj{{0.0, 0.0}};
    double best_d = 0.0, best_t = 0.0;
    for (int i = idx_; i < stop; ++i) {
      double t = 0.0, d = 0.0;
      const Pt2 proj = projectOnSegment({{x, y}}, pts[i], pts[std::min(i + 1, n - 1)], &t, &d);
      if (!found || d < best_d) {
        found = true;
        best_i = i;
        best_proj = proj;
        best_d = d;
        best_t = t;
      }
    }
    if (!found) return r;
    idx_ = std::max(idx_, best_i);  // 进度索引只增不减
    const double s_proj = cum[best_i] + best_t * (cum[std::min(best_i + 1, static_cast<int>(cum.size()) - 1)] - cum[best_i]);
    const Pt2 look = pointAtArc(pts, cum, s_proj + lookahead_).first;

    const double heading_err = agc::normalizeAngle(std::atan2(look[1] - y, look[0] - x) - yaw);
    // 有向横向误差：投影处路径左法线与 (车-投影) 的点积
    const Pt2 seg_dir = pointAtArc(pts, cum, s_proj).second;
    const double ex = x - best_proj[0], ey = y - best_proj[1];
    const double cross_track = (-seg_dir[1]) * ex + seg_dir[0] * ey;

    double w = k_heading_ * heading_err + k_lateral_ * cross_track;
    w = std::min(std::max(w, -max_w_), max_w_);
    double v = max_v_ * std::max(0.0, std::cos(heading_err));
    v = std::min(std::max(v, 0.0), max_v_);
    r.v = v;
    r.w = w;
    r.cross_track = cross_track;
    r.heading_err = heading_err;
    return r;
  }

  // ------------------------------------------------------------ 判定
  // 返回需要立即零速的原因；空串表示可以驱动
  std::string zeroReason() {
    if (!odom_msg_) return "no_odom";
    if (agc::stampAge(odom_msg_->header.stamp) > odom_max_age_) return "odom_stale";
    if (!path_msg_ || path_msg_->poses.empty()) return "no_path";
    if (!goal_msg_) return "no_goal";
    if (agc::stampAge(goal_msg_->header.stamp) > goal_max_age_) return "goal_stale";
    if (agc::stampAge(path_msg_->header.stamp) > path_max_age_) return "path_stale";
    if (task_state_ && agc::isTerminal(task_state_->state)) return "task_terminal";
    // 只有进入 GROUND_APPROACH 才允许驱动：IDLE/AIR_SEARCH/TARGET_ACQUIRED 期间必须零速，
    // 否则未收到任务许可就会跟 goal 跑（无信息时也必须安全停车）。
    if (!task_state_) return "no_task_state";
    if (static_cast<int>(task_state_->state) != agc::ST_GROUND_APPROACH) return "task_not_approaching";
    if (agc::pgetB("/agc/planner/unreachable", false)) return "planner_unreachable";
    if (agc::pgetB("/agc/commander/force_stop", false)) return "force_stop";
    return "";
  }

  bool carPose(std::array<double, 3>* out) {
    if (!odom_msg_) return false;
    const geometry_msgs::Pose& p = odom_msg_->pose.pose;
    (*out)[0] = p.position.x;
    (*out)[1] = p.position.y;
    (*out)[2] = agc::yawFromQuat(p.orientation);
    return true;
  }

  // ------------------------------------------------------------ 定时器
  void tick(const ros::TimerEvent&) {
    const ros::Time t = agc::simNow();
    std::string reason = zeroReason();
    bool goal_reached = false;
    double v = 0.0, w = 0.0;

    if (reason.empty()) {
      std::array<double, 3> pose;
      if (!carPose(&pose)) {
        reason = "no_odom_pose";
      } else {
        std::vector<Pt2> pts;
        pts.reserve(path_msg_->poses.size());
        for (const auto& p : path_msg_->poses) {
          pts.push_back({{p.pose.position.x, p.pose.position.y}});
        }
        const double gx = goal_msg_->pose.position.x;
        const double gy = goal_msg_->pose.position.y;
        const double dist_goal = std::hypot(pose[0] - gx, pose[1] - gy);
        if (dist_goal <= goal_tol_) {
          goal_reached = true;  // 到达：零速 + 记录 goal_reached
          reason = "goal_reached";
        } else if (dist_goal <= goal_stop_) {
          reason = "goal_stop_distance";
        } else {
          const Pursuit r = pursuit(pose, pts);
          v = r.v;
          w = r.w;
          if (!pts.empty()) reason = "tracking";
        }
      }
    }

    if (reason != "tracking") {
      v = 0.0;
      w = 0.0;
    }

    publish(t, v, w);  // 每拍都发（含零速）

    if (reason != last_reason_) {
      last_reason_ = reason;
      agc::JsonObj o;
      o.n("t_sim", t.toSec());
      if (reason.empty()) o.nullv("reason"); else o.v("reason", reason);
      o.bv("goal_reached", goal_reached).n("cmd_vx", v).n("cmd_wz", w);
      agc::appendJsonl("commander_log.jsonl", o);
      if (reason == "goal_reached") {
        ROS_INFO("ground_commander: goal_reached，零速停车");
      } else {
        ROS_INFO("ground_commander: 原因变化 -> %s (v=%.3f w=%.3f)", reason.c_str(), v, w);
      }
    }
  }

  void publish(const ros::Time& t, double v, double w) {
    geometry_msgs::TwistStamped msg;
    msg.header.frame_id = agc::FRAME_CAR;  // 速度表达在车体系
    msg.header.stamp = t;
    msg.twist.linear.x = v;
    msg.twist.angular.z = w;
    pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber path_sub_, goal_sub_, odom_sub_, state_sub_;
  ros::Publisher pub_;
  ros::Timer timer_;

  double lookahead_ = 0.6, k_heading_ = 1.5, k_lateral_ = 0.8;
  double max_w_ = 1.2, max_v_ = 1.0, goal_tol_ = 0.5, goal_stop_ = 0.30;
  double cmd_hz_ = 20.0, path_max_age_ = 2.0, goal_max_age_ = 1.0, odom_max_age_ = 0.5;

  nav_msgs::Path::ConstPtr path_msg_;
  geometry_msgs::PoseStamped::ConstPtr goal_msg_;
  nav_msgs::Odometry::ConstPtr odom_msg_;
  air_ground_core::AgcTaskState::ConstPtr task_state_;

  int idx_ = 0;
  std::tuple<uint32_t, uint32_t, size_t> path_stamp_id_;
  bool have_path_id_ = false;
  std::string last_reason_;  // 空串表示尚未记录（Python 的 None）
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_commander");
  GroundCommander node;
  ros::spin();
  return 0;
}