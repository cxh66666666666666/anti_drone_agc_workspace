// gt_adapter —— 真值目标适配器（M0 接口契约 v1.1，阶段 A 唯一目标源）
//
// C++ 等价实现，行为基准为 scripts/gt_adapter.py。发布 /agc/targets
// （air_ground_core/TargetTrackArray），关键约束：
// 1. 订阅 /gazebo/model_states 取 intruder_uav，按 /agc/rates/targets_hz 发布。
// 2. 数组 header.stamp = 发布时刻；每个 TargetTrack.header.stamp = 最后一次真实观测时刻。
//    重发旧 track 绝不刷新观测时间：本拍无新观测则 tracks 必为空。
// 3. pose/twist 均在 world 系：twist 由位置短时窗差分得 world 速度，不做机体系旋转；
//    这与 Odometry 的 child-frame twist 约定不同（见契约三）。
// 4. 无噪声基线：confidence=1，协方差取 /agc/gt/pose_std_m、/agc/gt/twist_std_m 的平方。
// 5. /agc/target/enabled=false 时视为目标不存在 -> 发空数组。
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

#include <air_ground_core/TargetTrack.h>
#include <air_ground_core/TargetTrackArray.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <array>
#include <string>

#include "agc/agc_common.hpp"

namespace {

// 阶段 A 目标源在 run 开始前固定，只有 gt_adapter 会发布 /agc/targets
const char* const SOURCE = "gt_adapter";

// 位置/速度对角协方差（对角元素 = std^2）
void diagCov(double sigma, boost::array<double, 36>& c) {
  c.assign(0.0);
  const double s2 = sigma * sigma;
  c[0] = c[7] = c[14] = s2;
}

struct Sample {
  double t, x, y, z, yaw;
};

}  // namespace

class GtAdapter {
 public:
  GtAdapter() : nh_(), history_() {
    ns_ = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
    while (!ns_.empty() && ns_.back() == '/') ns_.pop_back();
    target_model_ = agc::pgetStr("/agc/gazebo/target_model", "intruder_uav");
    target_id_ = agc::pgetStr("/agc/target/id", "intruder_uav");
    rate_hz_ = agc::pgetD("/agc/rates/targets_hz", 10.0);
    pose_std_m_ = agc::pgetD("/agc/gt/pose_std_m", 0.01);
    twist_std_m_ = agc::pgetD("/agc/gt/twist_std_m", 0.05);
    diff_window_s_ = agc::pgetD("/agc/gt/diff_window_s", 0.15);
    seed_ = agc::pgetStr("/agc/log/seed", "unseeded");
    // M4：目标源由 /agc/target/source 在 run 前固定（launch 单一设置）。
    // 仅当本节点是选中源时才 advertise /agc/targets，避免与 air_fusion 双发布者。
    target_source_ = agc::pgetStr("/agc/target/source", "gt_adapter");
    if (target_source_ == SOURCE) {
      pub_ = nh_.advertise<air_ground_core::TargetTrackArray>("/agc/targets", 10);
      has_pub_ = true;
    } else {
      ROS_WARN("gt_adapter: target/source=%s，本节点不发布 /agc/targets", target_source_.c_str());
    }

    sub_ = nh_.subscribe(ns_ + "/model_states", 1, &GtAdapter::onModelStates, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(rate_hz_, 1e-3)),
                             &GtAdapter::tick, this);

    ROS_INFO(
        "gt_adapter: source=%s (唯一目标源，run 前固定); noise model=none "
        "(noise-free baseline); seed=%s; pose_std=%.3f m twist_std=%.3f m/s; target_id=%s",
        SOURCE, seed_.c_str(), pose_std_m_, twist_std_m_, target_id_.c_str());
  }

 private:
  void onModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    int idx = -1;
    for (size_t i = 0; i < msg->name.size(); ++i)
      if (msg->name[i] == target_model_) {
        idx = static_cast<int>(i);
        break;
      }
    if (idx < 0 || idx >= static_cast<int>(msg->pose.size())) {
      target_present_ = false;
      return;
    }
    const geometry_msgs::Pose& pose = msg->pose[idx];
    // gazebo_msgs/ModelStates 没有 header，观测时刻取收到该帧的仿真时钟
    const ros::Time stamp = agc::simNow();
    double t = stamp.toSec();
    if (t <= 0.0) t = agc::wallNow();
    history_.push_back(Sample{t, pose.position.x, pose.position.y, pose.position.z,
                              agc::yawFromQuat(pose.orientation)});
    if (history_.size() > 500) history_.pop_front();  // 对齐 Python deque(maxlen=500)
    last_pos_ = {pose.position.x, pose.position.y, pose.position.z};
    last_quat_ = {pose.orientation.x, pose.orientation.y, pose.orientation.z,
                  pose.orientation.w};
    last_obs_stamp_ = stamp;
    target_present_ = true;
    has_last_pose_ = true;
    new_obs_ = true;
  }

  // 短时窗差分得 world 线速度与 yaw 角速度（不做机体系旋转）
  void velocity(std::array<double, 3>& v, double& wz) const {
    v = {{0.0, 0.0, 0.0}};
    wz = 0.0;
    if (history_.size() < 2) return;
    const Sample& a = history_.back();
    Sample b = history_.front();
    for (const Sample& e : history_) {  // 取窗内最早样本作差分基准
      if (a.t - e.t <= diff_window_s_) {
        b = e;
        break;
      }
    }
    const double dt = a.t - b.t;
    if (dt <= 1e-6) return;
    v = {{(a.x - b.x) / dt, (a.y - b.y) / dt, (a.z - b.z) / dt}};
    wz = agc::normalizeAngle(a.yaw - b.yaw) / dt;
  }

  air_ground_core::TargetTrack buildTrack() {
    air_ground_core::TargetTrack tr;
    tr.header.stamp = last_obs_stamp_;  // 最后一次真实观测时刻，非发布时刻
    tr.header.frame_id = agc::FRAME_WORLD;
    tr.target_id = target_id_;
    tr.pose.pose.position.x = last_pos_[0];
    tr.pose.pose.position.y = last_pos_[1];
    tr.pose.pose.position.z = last_pos_[2];
    tr.pose.pose.orientation.x = last_quat_[0];
    tr.pose.pose.orientation.y = last_quat_[1];
    tr.pose.pose.orientation.z = last_quat_[2];
    tr.pose.pose.orientation.w = last_quat_[3];
    diagCov(pose_std_m_, tr.pose.covariance);
    std::array<double, 3> v;
    double wz = 0.0;
    velocity(v, wz);
    tr.twist.twist.linear.x = v[0];
    tr.twist.twist.linear.y = v[1];
    tr.twist.twist.linear.z = v[2];
    tr.twist.twist.angular.x = 0.0;
    tr.twist.twist.angular.y = 0.0;
    tr.twist.twist.angular.z = wz;
    diagCov(twist_std_m_, tr.twist.covariance);
    tr.confidence = 1.0f;
    tr.source = SOURCE;
    return tr;
  }

  void tick(const ros::TimerEvent&) {
    air_ground_core::TargetTrackArray arr;
    arr.header.stamp = agc::simNow();
    arr.header.frame_id = agc::FRAME_WORLD;

    const bool enabled = agc::pgetB("/agc/target/enabled", true);
    if (enabled && new_obs_ && target_present_ && has_last_pose_)
      arr.tracks.push_back(buildTrack());
    // 无论是否发出，本拍观测已被消费：下拍若无新消息则发空数组
    new_obs_ = false;
    if (has_pub_) pub_.publish(arr);
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer timer_;

  std::string ns_, target_model_, target_id_, seed_, target_source_;
  bool has_pub_ = false;
  double rate_hz_, pose_std_m_, twist_std_m_, diff_window_s_;

  std::deque<Sample> history_;
  std::array<double, 3> last_pos_{{0.0, 0.0, 0.0}};
  std::array<double, 4> last_quat_{{0.0, 0.0, 0.0, 1.0}};
  ros::Time last_obs_stamp_;
  bool has_last_pose_ = false;
  bool new_obs_ = false;
  bool target_present_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gt_adapter");
  GtAdapter node;
  ros::spin();
  return 0;
}