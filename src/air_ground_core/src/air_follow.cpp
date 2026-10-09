// air_follow —— 无人机跟随移动目标（安全距离带），N2（C++ 实时链路）
//
// 职责：把「新鲜目标观测」变为**可选跟随参考** /agc/air/follow_ref（无人机期望位置，world 系），
// 由唯一 setpoint 发布者 lqr_tracker_node 消费。无人机仍只供信息，路径规划仍在地面。
//
// 安全距离带（水平距离 d = |drone_xy - target_xy|）：
//   d > safe_max：过远跟近（参考 = 目标 + 单位向 * safe_max）
//   d < safe_min：过近退离（参考 = 目标 + 单位向 * safe_min）
//   safe_min ≤ d ≤ safe_max：带内悬停（参考冻结在进入带时的保持点，避免持续摆动）
// 目标过期/失联：冻结在最近保持点（不追旧目标）；任务非运行期同样冻结。
// 高度：恒定取 altitude_m。参考跳变按 max_step_m 限幅。
//
// 参数（/agc/air/follow/*，见 agc_params.yaml）：enabled/safe_min_m/safe_max_m/hysteresis_m/
//   altitude_m/ref_max_age_s/confidence_min/max_step_m/rate_hz。
// 观测与参考逐帧落 air_follow_log.jsonl（距离、参考、模式、新鲜度、有效性）。
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <air_ground_core/AgcTaskState.h>
#include <air_ground_core/TargetTrackArray.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "agc/agc_common.hpp"

namespace {
const char* const FRAME = "world";

enum BandMode { BAND_HOLD = 0, BAND_APPROACH = 1, BAND_RETREAT = 2 };
const char* bandName(int m) {
  switch (m) {
    case BAND_APPROACH: return "approach";
    case BAND_RETREAT: return "retreat";
    default: return "hold_in_band";
  }
}
}  // namespace

class AirFollow {
 public:
  AirFollow() : nh_() {
    enabled_ = agc::pgetB("/agc/air/follow/enabled", false);
    safe_min_ = agc::pgetD("/agc/air/follow/safe_min_m", 2.0);
    safe_max_ = agc::pgetD("/agc/air/follow/safe_max_m", 3.5);
    hyst_ = agc::pgetD("/agc/air/follow/hysteresis_m", 0.3);
    altitude_ = agc::pgetD("/agc/air/follow/altitude_m",
                           agc::pgetD("/agc/air/takeoff_altitude_m", 2.0));
    ref_max_age_ = agc::pgetD("/agc/air/follow/ref_max_age_s", 0.5);
    confidence_min_ = agc::pgetD("/agc/air/follow/confidence_min", 0.5);
    max_step_ = agc::pgetD("/agc/air/follow/max_step_m", 0.6);
    rate_hz_ = agc::pgetD("/agc/air/follow/rate_hz", 20.0);
    target_id_ = agc::pgetStr("/agc/target/id", "intruder_uav");
    if (safe_min_ > safe_max_) std::swap(safe_min_, safe_max_);

    sub_targets_ = nh_.subscribe("/agc/targets", 10, &AirFollow::targetsCb, this);
    sub_odom_ = nh_.subscribe("/agc/air/odom", 10, &AirFollow::odomCb, this);
    sub_state_ = nh_.subscribe("/agc/task/state", 10, &AirFollow::stateCb, this);
    pub_ref_ = nh_.advertise<geometry_msgs::PoseStamped>("/agc/air/follow_ref", 10);
    pub_valid_ = nh_.advertise<std_msgs::Bool>("/agc/air/follow_valid", 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(rate_hz_, 1e-3)),
                             &AirFollow::tick, this);
    ROS_INFO("air_follow 就绪：enabled=%d 安全带[%.2f, %.2f]m 高度%.2f 最大步长%.2f/拍（水平跟随，位置参考）",
             enabled_, safe_min_, safe_max_, altitude_, max_step_);
  }

 private:
  void targetsCb(const air_ground_core::TargetTrackArray::ConstPtr& msg) {
    has_target_ = false;
    for (const auto& tr : msg->tracks) {
      if (tr.target_id == target_id_ || msg->tracks.size() == 1) {
        target_x_ = tr.pose.pose.position.x;
        target_y_ = tr.pose.pose.position.y;
        target_conf_ = tr.confidence;
        target_stamp_ = tr.header.stamp.toSec();  // 最后一次真实观测时刻
        has_target_ = true;
        break;
      }
    }
  }
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    drone_x_ = msg->pose.pose.position.x;
    drone_y_ = msg->pose.pose.position.y;
    drone_z_ = msg->pose.pose.position.z;
    has_drone_ = true;
  }
  void stateCb(const air_ground_core::AgcTaskState::ConstPtr& msg) {
    task_state_ = msg->state;
    has_task_state_ = true;
  }

  void tick(const ros::TimerEvent&) {
    if (!enabled_) return;  // 未启用：不产生任何参考（既有场景不受影响）
    const double t = agc::simNow().toSec();
    if (t <= 0.0) return;

    const bool active = has_task_state_ && !agc::isTerminal(task_state_) &&
                        task_state_ != agc::ST_IDLE;
    const double age = has_target_ ? (t - target_stamp_) : agc::INF;
    const bool fresh = has_target_ && age <= ref_max_age_ &&
                       static_cast<double>(target_conf_) >= confidence_min_;

    double rx, ry;
    std::string mode;
    if (active && fresh && has_drone_) {
      const double dx = target_x_ - drone_x_, dy = target_y_ - drone_y_;  // 指向目标
      const double d = std::hypot(dx, dy);
      double ux = 1.0, uy = 0.0;
      if (d > 1e-6) { ux = dx / d; uy = dy / d; }
      // 带内/带外 + 滞回，避免边界抖动
      if (band_mode_ == BAND_HOLD) {
        if (d > safe_max_) band_mode_ = BAND_APPROACH;
        else if (d < safe_min_) band_mode_ = BAND_RETREAT;
      } else if (band_mode_ == BAND_APPROACH) {
        if (d < safe_min_) band_mode_ = BAND_RETREAT;
        else if (d <= safe_max_ - hyst_) band_mode_ = BAND_HOLD;
      } else {  // RETREAT
        if (d > safe_max_) band_mode_ = BAND_APPROACH;
        else if (d >= safe_min_ + hyst_) band_mode_ = BAND_HOLD;
      }
      if (band_mode_ == BAND_APPROACH) {
        // 过远跟近：参考沿「无人机->目标」方向前进（每拍至多 max_step），把 d 降到带内；
        // 方向取自目标而非无人机相对目标的径向，避免参考随无人机漂移旋转（否则绕圈）。
        const double want = std::min(d - (safe_max_ - hyst_), max_step_);
        rx = drone_x_ + ux * want;
        ry = drone_y_ + uy * want;
        hold_x_ = rx; hold_y_ = ry; has_hold_ = true;
        mode = bandName(band_mode_);
      } else if (band_mode_ == BAND_RETREAT) {
        // 过近退离：沿远离目标方向后退（每拍至多 max_step），把 d 抬回带内。
        const double want = std::min((safe_min_ + hyst_) - d, max_step_);
        rx = drone_x_ - ux * want;
        ry = drone_y_ - uy * want;
        hold_x_ = rx; hold_y_ = ry; has_hold_ = true;
        mode = bandName(band_mode_);
      } else {
        if (!has_hold_) { hold_x_ = drone_x_; hold_y_ = drone_y_; has_hold_ = true; }
        rx = hold_x_; ry = hold_y_;
        mode = bandName(band_mode_);
      }
    } else {
      // 失联/未激活：冻结在最近保持点（不追旧目标）
      if (!has_hold_) { hold_x_ = drone_x_; hold_y_ = drone_y_; has_hold_ = true; }
      band_mode_ = BAND_HOLD;
      rx = hold_x_; ry = hold_y_;
      mode = active ? "hold_no_target" : "hold_inactive";
    }

    // 参考跳变限幅（相对上一次发布参考）
    if (has_last_ref_) {
      const double ddx = rx - last_rx_, ddy = ry - last_ry_;
      const double dd = std::hypot(ddx, ddy);
      if (dd > max_step_) {
        rx = last_rx_ + ddx / dd * max_step_;
        ry = last_ry_ + ddy / dd * max_step_;
      }
    }
    last_rx_ = rx; last_ry_ = ry; has_last_ref_ = true;
    const double rz = altitude_;

    geometry_msgs::PoseStamped ref;
    ref.header.stamp = agc::simNow();
    ref.header.frame_id = FRAME;
    ref.pose.position.x = rx;
    ref.pose.position.y = ry;
    ref.pose.position.z = rz;
    ref.pose.orientation.w = 1.0;
    pub_ref_.publish(ref);

    std_msgs::Bool vb;
    vb.data = (active && fresh);
    pub_valid_.publish(vb);

    const double d = (has_drone_ && has_target_)
                         ? std::hypot(drone_x_ - target_x_, drone_y_ - target_y_)
                         : -1.0;
    agc::JsonObj o;
    o.n("t_sim", t).n("target_x", target_x_).n("target_y", target_y_)
        .n("drone_x", drone_x_).n("drone_y", drone_y_).n("drone_z", drone_z_)
        .n("distance_m", d).v("band_mode", mode)
        .n("ref_x", rx).n("ref_y", ry).n("ref_z", rz)
        .n("target_age_s", has_target_ ? age : -1.0).bv("valid", active && fresh)
        .iv("task_state", task_state_);
    agc::appendJsonl("air_follow_log.jsonl", o);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_targets_, sub_odom_, sub_state_;
  ros::Publisher pub_ref_, pub_valid_;
  ros::Timer timer_;

  bool enabled_ = false;
  double safe_min_ = 2.0, safe_max_ = 3.5, hyst_ = 0.3, altitude_ = 2.0;
  double ref_max_age_ = 0.5, confidence_min_ = 0.5, max_step_ = 0.15, rate_hz_ = 20.0;
  std::string target_id_;

  bool has_target_ = false, has_drone_ = false, has_task_state_ = false, has_hold_ = false,
       has_last_ref_ = false;
  double target_x_ = 0, target_y_ = 0, target_conf_ = 0, target_stamp_ = 0;
  double drone_x_ = 0, drone_y_ = 0, drone_z_ = 0;
  double hold_x_ = 0, hold_y_ = 0, last_rx_ = 0, last_ry_ = 0;
  int band_mode_ = BAND_HOLD;
  int32_t task_state_ = agc::ST_IDLE;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "air_follow");
  AirFollow node;
  ros::spin();
  return 0;
}