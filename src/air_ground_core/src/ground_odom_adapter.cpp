// ground_odom_adapter —— 地面车实际车体状态适配器（M0 接口契约 v1.1）
// 基准：scripts/ground_odom_adapter.py，C++ 版逐条对齐。
//
// 1. 来源必须是 Gazebo 实际车体状态：订阅 /gazebo/model_states，取 agc_car（回退
//    ground_car）的 pose。禁止读取 CarStatus 的位置字段或任何驱动器内部积分状态冒充反馈。
// 2. twist 约定：ROS Odometry 的 twist 表达在 child frame(car/base_link)。对 world 位置做
//    短时窗（最近 ~diff_window_s 差分）数值微分得 world 速度，再用 R(q_world_car)^T 旋转到
//    车体系；角速度用相邻 yaw 差分（车体 z 轴）。
// 3. 按 /agc/rates/ground_odom_hz（默认 30 Hz）定时发布；超过 odom_max_age_s 无新状态 ->
//    告警并置 /agc/ground/odom_valid=false。
// 4. 发布 TF world -> car/base_link（不发布裸 base_link）；同一仿真时刻去重。
#include "agc/agc_common.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>

#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

namespace {
const size_t kHistoryMax = 500;  // 与 Python deque(maxlen=500) 一致

struct Sample {
  double t, x, y, yaw;
};
}  // namespace

class GroundOdomAdapter {
 public:
  GroundOdomAdapter() : nh_() {
    ns_ = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
    while (!ns_.empty() && ns_.back() == '/') ns_.pop_back();
    const std::string primary = agc::pgetStr("/agc/gazebo/car_model", "agc_car");
    // 模型名回退：agc_car -> ground_car
    car_models_.push_back(primary);
    if (primary != "ground_car") car_models_.push_back("ground_car");
    odom_max_age_s_ = agc::pgetD("/agc/freshness/odom_max_age_s", 0.5);
    rate_hz_ = agc::pgetD("/agc/rates/ground_odom_hz", 30.0);
    diff_window_s_ = agc::pgetD("/agc/ground/diff_window_s", 0.15);
    car_name_ = car_models_.front();

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/agc/ground/odom", 10);
    ros::param::set("/agc/ground/odom_valid", false);

    model_sub_ = nh_.subscribe(ns_ + "/model_states", 1,
                               &GroundOdomAdapter::onModelStates, this);
    const double period = 1.0 / std::max(rate_hz_, 1e-3);
    timer_ = nh_.createTimer(ros::Duration(period), &GroundOdomAdapter::tick, this);

    std::string models;
    for (const auto& m : car_models_) models += (models.empty() ? "" : ",") + m;
    ROS_INFO("ground_odom_adapter started: gazebo_ns=%s car_models=[%s] rate=%.1fHz",
             ns_.c_str(), models.c_str(), rate_hz_);
  }

 private:
  void onModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    int idx = -1;
    for (const auto& name : car_models_) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), name);
      if (it != msg->name.end()) {
        idx = static_cast<int>(std::distance(msg->name.begin(), it));
        car_name_ = name;
        break;
      }
    }
    if (idx < 0) return;
    const geometry_msgs::Pose& p = msg->pose[static_cast<size_t>(idx)];
    // gazebo_msgs/ModelStates 没有 header，取收到该帧的仿真时刻
    double t = agc::simNow().toSec();
    if (t <= 0.0) t = agc::wallNow();  // 时钟未就绪时退回墙钟差分
    cur_pos_ = {{p.position.x, p.position.y, p.position.z}};
    cur_quat_ = {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
    have_cur_ = true;
    history_.push_back(Sample{t, p.position.x, p.position.y, agc::yawFromQuat(p.orientation)});
    if (history_.size() > kHistoryMax) history_.pop_front();
    last_rx_wall_ = agc::wallNow();
    last_rx_sim_ = t;
    found_ = true;
  }

  double age() const {
    const ros::Time now = agc::simNow();
    if (now.toSec() > 0.0 && last_rx_sim_ > 0.0) return std::max(0.0, now.toSec() - last_rx_sim_);
    return agc::wallNow() - last_rx_wall_;
  }

  // 短时窗差分得 world 线速度与 yaw 角速度
  void velocity(std::array<double, 3>* v_world, double* wz) const {
    *v_world = {{0.0, 0.0, 0.0}};
    *wz = 0.0;
    if (history_.size() < 2) return;
    const Sample& s1 = history_.back();
    Sample s0 = history_.front();
    for (const Sample& e : history_) {  // 取窗口内最早样本
      if (s1.t - e.t <= diff_window_s_) {
        s0 = e;
        break;
      }
    }
    const double dt = s1.t - s0.t;
    if (dt <= 1e-6) return;
    (*v_world)[0] = (s1.x - s0.x) / dt;
    (*v_world)[1] = (s1.y - s0.y) / dt;
    (*v_world)[2] = 0.0;
    *wz = agc::normalizeAngle(s1.yaw - s0.yaw) / dt;
  }

  void setValid(bool ok) {
    if (ok != valid_flag_) {
      ros::param::set("/agc/ground/odom_valid", ok);
      valid_flag_ = ok;
    }
  }

  void tick(const ros::TimerEvent&) {
    if (!found_ || !have_cur_) {
      ROS_ERROR_THROTTLE(5.0, "ground odom: car model %s not found in %s/model_states",
                         car_models_.front().c_str(), ns_.c_str());
      setValid(false);
      return;
    }
    const double a = age();
    if (a > odom_max_age_s_) {
      ROS_WARN_THROTTLE(2.0, "ground odom stale: age=%.3f s > %.3f s (car=%s)",
                        a, odom_max_age_s_, car_name_.c_str());
      setValid(false);
      return;
    }

    std::array<double, 3> v_world;
    double wz = 0.0;
    velocity(&v_world, &wz);
    const std::array<double, 3> v_body = agc::matTVec(agc::quatToMatrix(cur_quat_), v_world);

    // 同一仿真时刻重复发布 TF 会触发 tf2 TF_REPEATED_DATA，去重
    const ros::Time stamp = agc::simNow();
    if (has_last_pub_ && stamp == last_pub_stamp_) return;
    last_pub_stamp_ = stamp;
    has_last_pub_ = true;

    nav_msgs::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = agc::FRAME_WORLD;
    o.child_frame_id = agc::FRAME_CAR;
    o.pose.pose.position.x = cur_pos_[0];
    o.pose.pose.position.y = cur_pos_[1];
    o.pose.pose.position.z = cur_pos_[2];
    o.pose.pose.orientation.x = cur_quat_[0];
    o.pose.pose.orientation.y = cur_quat_[1];
    o.pose.pose.orientation.z = cur_quat_[2];
    o.pose.pose.orientation.w = cur_quat_[3];
    o.twist.twist.linear.x = v_body[0];
    o.twist.twist.linear.y = v_body[1];
    o.twist.twist.linear.z = v_body[2];
    o.twist.twist.angular.x = 0.0;
    o.twist.twist.angular.y = 0.0;
    o.twist.twist.angular.z = wz;
    odom_pub_.publish(o);

    agc::sendTf(o.header.stamp, agc::FRAME_WORLD, agc::FRAME_CAR, cur_pos_, cur_quat_);
    setValid(true);
  }

  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber model_sub_;
  ros::Timer timer_;

  std::string ns_, car_name_;
  std::vector<std::string> car_models_;
  double odom_max_age_s_ = 0.5, rate_hz_ = 30.0, diff_window_s_ = 0.15;

  bool have_cur_ = false, found_ = false;
  std::array<double, 3> cur_pos_{{0, 0, 0}};
  std::array<double, 4> cur_quat_{{0, 0, 0, 1}};
  std::deque<Sample> history_;
  double last_rx_wall_ = 0.0, last_rx_sim_ = 0.0;

  bool valid_flag_ = false;
  ros::Time last_pub_stamp_;
  bool has_last_pub_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_odom_adapter");
  GroundOdomAdapter node;
  ros::spin();
  return 0;
}