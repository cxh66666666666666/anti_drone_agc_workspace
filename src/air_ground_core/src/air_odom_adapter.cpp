// air_odom_adapter —— 空中本地位姿适配器（M0 接口契约 v1.1）
// 基准：scripts/air_odom_adapter.py，C++ 版逐条对齐其参数、话题、帧与失效判定。
//
// 1. 来源：优先 /mavros/local_position/odom；节点启动 5 s(仿真时钟)内该话题无消息，
//    回退订阅 /mavros/local_position/pose(+velocity_local)，实际来源写入 source_topic。
// 2. 标定 T_world_air_local：收到首帧有效 MAVROS 位姿时同时取 /gazebo/model_states 的
//    iris 真值，按 2D 刚体 world = R(dyaw)*air_local + t 求 (dx,dy,dz,dyaw)；
//    不假定 air_local 原点等于 spawn 点；随后 10 帧用真值统计残差 residual_m。
// 3. 速度约定：MAVROS 速度视为 air_local(ENU) 系速度估计，先经 R(dyaw) 到 world 线速度，
//    再用 R(q_world_body)^T 旋转到机体系后写入 twist（ROS Odometry 的 twist 在 child frame）。
//    角速度按 MAVROS 约定原在机体系，直接透传。
// 4. TF：标定后静态 world->air_local、动态 world->drone/base_link；绝不发布裸 base_link。
// 5. 事件驱动：每收一帧发布一次（≈30 Hz）；超过 /agc/freshness/odom_max_age_s 无新数据
//    -> 告警并置 /agc/air/odom_valid=false。
// 6. 失效：相邻 world 位置跳变 > jump_threshold_m 或 pose 协方差主对角超阈 -> 停发并置
//    odom_valid=false；服务 /agc/air/recalibrate 重新标定后恢复。
#include "agc/agc_common.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

namespace {

const int kCalibResidualFrames = 10;  // 标定后用随后多少帧统计残差

// 四元数乘法（x, y, z, w）：world 姿态 = q(dyaw) * q_air
std::array<double, 4> qmul(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  const double ax = a[0], ay = a[1], az = a[2], aw = a[3];
  const double bx = b[0], by = b[1], bz = b[2], bw = b[3];
  return {{aw * bx + ax * bw + ay * bz - az * by,
           aw * by - ax * bz + ay * bw + az * bx,
           aw * bz + ax * by - ay * bx + az * bw,
           aw * bw - ax * bx - ay * by - az * bz}};
}

// 递归创建目录（标定 JSON 落盘前保证父目录存在）
void ensureDir(const std::string& path) {
  if (path.empty()) return;
  std::string cur;
  for (size_t i = 0; i < path.size(); ++i) {
    cur.push_back(path[i]);
    if (path[i] == '/' && cur.size() > 1) mkdir(cur.c_str(), 0755);
  }
  mkdir(path.c_str(), 0755);
}

}  // namespace

class AirOdomAdapter {
 public:
  AirOdomAdapter() : nh_() {
    ns_ = agc::pgetStr("/agc/gazebo/ns", "/gazebo");
    while (!ns_.empty() && ns_.back() == '/') ns_.pop_back();  // 对齐 Python rstrip("/")
    drone_model_ = agc::pgetStr("/agc/gazebo/drone_model", "iris");
    odom_max_age_s_ = agc::pgetD("/agc/freshness/odom_max_age_s", 0.5);
    jump_threshold_m_ = agc::pgetD("/agc/air/jump_threshold_m", 2.0);
    cov_jump_threshold_ = agc::pgetD("/agc/air/cov_jump_threshold", 1.0);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/agc/air/odom", 10);

    ros::param::set("/agc/air/odom_valid", false);

    model_sub_ = nh_.subscribe(ns_ + "/model_states", 1,
                               &AirOdomAdapter::onModelStates, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10,
                              &AirOdomAdapter::onOdom, this);

    recal_srv_ = nh_.advertiseService("/agc/air/recalibrate",
                                      &AirOdomAdapter::onRecalibrate, this);
    // 启动 5 s 后检查主话题是否出现过（仿真时钟）
    fallback_timer_ = nh_.createTimer(ros::Duration(5.0),
                                      &AirOdomAdapter::onFallbackCheck, this, true);
    freshness_timer_ = nh_.createTimer(ros::Duration(0.1),
                                       &AirOdomAdapter::onFreshnessCheck, this);

    ROS_INFO("air_odom_adapter started: primary=/mavros/local_position/odom "
             "gazebo_ns=%s drone_model=%s", ns_.c_str(), drone_model_.c_str());
  }

 private:
  // ---------------------------------------------------------- 订阅回调
  void onModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), drone_model_);
    if (it == msg->name.end()) return;
    const size_t i = static_cast<size_t>(std::distance(msg->name.begin(), it));
    const geometry_msgs::Pose& p = msg->pose[i];
    std::lock_guard<std::mutex> lk(mtx_);
    truth_pos_ = {{p.position.x, p.position.y, p.position.z}};
    truth_quat_ = {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
    have_truth_ = true;  // ModelStates 无 header，真值以当前仿真时刻为准
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    primary_seen_ = true;
    source_topic_ = "/mavros/local_position/odom";
    const geometry_msgs::Pose& p = msg->pose.pose;
    const geometry_msgs::Twist& tw = msg->twist.twist;
    onMeasurement({{p.position.x, p.position.y, p.position.z}},
                  {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}},
                  {{tw.linear.x, tw.linear.y, tw.linear.z}},
                  {{tw.angular.x, tw.angular.y, tw.angular.z}},
                  &msg->pose.covariance, &msg->twist.covariance);
  }

  void onTwistFallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    twist_fallback_ = msg;
    have_twist_fallback_ = true;
  }

  void onPoseFallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    source_topic_ = "/mavros/local_position/pose+velocity_local";
    const geometry_msgs::Pose& p = msg->pose;
    std::array<double, 3> lin{{0.0, 0.0, 0.0}}, ang{{0.0, 0.0, 0.0}};
    if (have_twist_fallback_) {
      const geometry_msgs::Twist& tw = twist_fallback_->twist;
      lin = {{tw.linear.x, tw.linear.y, tw.linear.z}};
      ang = {{tw.angular.x, tw.angular.y, tw.angular.z}};
    }
    // Fallback 的 PoseStamped 不带协方差，传 nullptr 表示未知
    onMeasurement({{p.position.x, p.position.y, p.position.z}},
                  {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}},
                  lin, ang, nullptr, nullptr);
  }

  void onFallbackCheck(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (primary_seen_ || fallback_active_) return;
    fallback_active_ = true;
    source_topic_ = "/mavros/local_position/pose+velocity_local";
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10,
                              &AirOdomAdapter::onPoseFallback, this);
    vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10,
                             &AirOdomAdapter::onTwistFallback, this);
    ROS_WARN("no /mavros/local_position/odom within 5.0 s -> fallback to "
             "/mavros/local_position/pose + /mavros/local_position/velocity_local");
  }

  // ---------------------------------------------------------- 核心处理（调用方持锁）
  void onMeasurement(const std::array<double, 3>& pos_air,
                     const std::array<double, 4>& quat_air,
                     const std::array<double, 3>& lin_air,
                     const std::array<double, 3>& ang_body,
                     const boost::array<double, 36>* pcov,
                     const boost::array<double, 36>* tcov) {
    const ros::Time now = agc::simNow();
    last_rx_wall_ = agc::wallNow();
    last_rx_sim_ = now.toSec();
    have_last_rx_ = true;
    if (stopped_) return;
    if (!calib_done_ && !tryCalibrate(pos_air, quat_air)) return;

    const std::array<double, 3> wpos = posToWorld(pos_air);
    const std::array<double, 4> wquat = qmul(agc::quatFromYaw(calib_dyaw_), quat_air);

    // 位置跳变（EKF 重置/变换突变）
    if (has_prev_world_) {
      const double d = agc::dist3(prev_world_, wpos);
      if (d > jump_threshold_m_) {
        invalidate("position_jump " + num(d) + " m > " + num(jump_threshold_m_) + " m");
        return;
      }
    }
    // pose 协方差主对角异常增大
    if (pcov != nullptr) {
      const double mx = std::max(std::fabs((*pcov)[0]),
                                 std::max(std::fabs((*pcov)[7]), std::fabs((*pcov)[14])));
      if (mx > cov_jump_threshold_) {
        invalidate("pose_covariance_diag " + num(mx) + " > " + num(cov_jump_threshold_));
        return;
      }
    }

    // 速度：air_local(ENU) 线速度 -> world -> 机体系 twist
    const std::array<double, 3> v_world = vecToWorld(lin_air);
    const std::array<double, 3> v_body = agc::matTVec(agc::quatToMatrix(wquat), v_world);

    prev_world_ = wpos;
    has_prev_world_ = true;
    updateResidual(wpos);
    publish(now, wpos, wquat, v_body, ang_body, pcov, tcov);
  }

  bool tryCalibrate(const std::array<double, 3>& pos_air,
                    const std::array<double, 4>& quat_air) {
    if (!have_truth_) return false;
    const double dyaw = agc::normalizeAngle(agc::yawFromQuat(truth_quat_[0], truth_quat_[1],
                                                            truth_quat_[2], truth_quat_[3]) -
                                            agc::yawFromQuat(quat_air[0], quat_air[1],
                                                             quat_air[2], quat_air[3]));
    const double c = std::cos(dyaw), s = std::sin(dyaw);
    calib_dx_ = truth_pos_[0] - (c * pos_air[0] - s * pos_air[1]);
    calib_dy_ = truth_pos_[1] - (s * pos_air[0] + c * pos_air[1]);
    calib_dz_ = truth_pos_[2] - pos_air[2];
    calib_dyaw_ = dyaw;
    calib_done_ = true;
    calib_sim_ = agc::simNow().toSec();
    calib_wall_ = agc::wallNow();
    residuals_.clear();
    residual_m_ = std::numeric_limits<double>::quiet_NaN();
    resid_finalized_ = false;
    usable_ = true;
    has_prev_world_ = false;
    stopped_ = false;

    broadcastStatic();
    ROS_INFO("air odom calibrated: dx=%.4f dy=%.4f dz=%.4f dyaw=%.5f rad (source=%s)",
             calib_dx_, calib_dy_, calib_dz_, calib_dyaw_, source_topic_.c_str());
    writeCalibration();
    return true;
  }

  std::array<double, 3> posToWorld(const std::array<double, 3>& pos) const {
    const double c = std::cos(calib_dyaw_), s = std::sin(calib_dyaw_);
    return {{c * pos[0] - s * pos[1] + calib_dx_,
             s * pos[0] + c * pos[1] + calib_dy_,
             pos[2] + calib_dz_}};
  }

  std::array<double, 3> vecToWorld(const std::array<double, 3>& v) const {
    const double c = std::cos(calib_dyaw_), s = std::sin(calib_dyaw_);
    return {{c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]}};
  }

  void updateResidual(const std::array<double, 3>& wpos) {
    if (!have_truth_) return;
    residuals_.push_back(agc::dist3(truth_pos_, wpos));
    if (static_cast<int>(residuals_.size()) >= kCalibResidualFrames && !resid_finalized_) {
      double sum = 0.0;
      for (double r : residuals_) sum += r;
      residual_m_ = sum / static_cast<double>(residuals_.size());
      resid_finalized_ = true;
      writeCalibration();
      ROS_INFO("air odom calibration residual_m=%.5f (over %zu frames)",
               residual_m_, residuals_.size());
    }
  }

  void publish(const ros::Time& stamp, const std::array<double, 3>& wpos,
               const std::array<double, 4>& wquat, const std::array<double, 3>& v_body,
               const std::array<double, 3>& ang, const boost::array<double, 36>* pcov,
               const boost::array<double, 36>* tcov) {
    // 同一仿真时刻（/clock 未推进）重复发布会造成 tf2 TF_REPEATED_DATA 噪声，去重
    if (has_last_pub_ && stamp == last_pub_stamp_) return;
    last_pub_stamp_ = stamp;
    has_last_pub_ = true;

    nav_msgs::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = agc::FRAME_WORLD;
    o.child_frame_id = agc::FRAME_DRONE;
    o.pose.pose.position.x = wpos[0];
    o.pose.pose.position.y = wpos[1];
    o.pose.pose.position.z = wpos[2];
    o.pose.pose.orientation.x = wquat[0];
    o.pose.pose.orientation.y = wquat[1];
    o.pose.pose.orientation.z = wquat[2];
    o.pose.pose.orientation.w = wquat[3];
    o.twist.twist.linear.x = v_body[0];
    o.twist.twist.linear.y = v_body[1];
    o.twist.twist.linear.z = v_body[2];
    o.twist.twist.angular.x = ang[0];
    o.twist.twist.angular.y = ang[1];
    o.twist.twist.angular.z = ang[2];
    if (pcov != nullptr) o.pose.covariance = *pcov;
    if (tcov != nullptr) o.twist.covariance = *tcov;
    odom_pub_.publish(o);

    // 动态 TF：world -> drone/base_link（不发布裸 base_link）
    agc::sendTf(stamp, agc::FRAME_WORLD, agc::FRAME_DRONE, wpos, wquat);

    if (!valid_flag_) {
      ros::param::set("/agc/air/odom_valid", true);
      valid_flag_ = true;
    }
  }

  void broadcastStatic() {
    // world -> air_local 静态变换：world = R(dyaw)*air_local + t
    agc::sendStaticTf(agc::FRAME_WORLD, agc::FRAME_AIR_LOCAL,
                      {{calib_dx_, calib_dy_, calib_dz_}}, agc::quatFromYaw(calib_dyaw_));
  }

  void onFreshnessCheck(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (stopped_ || !have_last_rx_) return;
    const ros::Time now = agc::simNow();
    double age;
    if (now.toSec() > 0.0 && last_rx_sim_ > 0.0) {
      age = std::max(0.0, now.toSec() - last_rx_sim_);
    } else {
      age = agc::wallNow() - last_rx_wall_;
    }
    if (age > odom_max_age_s_) {
      if (valid_flag_) {
        ROS_WARN("air odom stale: age=%.3f s > %.3f s", age, odom_max_age_s_);
        ros::param::set("/agc/air/odom_valid", false);
        valid_flag_ = false;
      }
    } else if (!valid_flag_ && calib_done_) {
      ros::param::set("/agc/air/odom_valid", true);
      valid_flag_ = true;
    }
  }

  void invalidate(const std::string& reason) {
    stopped_ = true;
    valid_flag_ = false;
    ros::param::set("/agc/air/odom_valid", false);
    ros::param::set("/agc/air/last_odom_event", reason);
    ROS_ERROR("air odom INVALID (%s); publish stopped, current run -> "
              "INFRA_FAILURE/ODOM_INVALID", reason.c_str());
  }

  // ---------------------------------------------------------- 标定记录
  void writeCalibration() {
    if (!calib_done_) return;
    agc::JsonObj t;
    t.n("dx", calib_dx_).n("dy", calib_dy_).n("dz", calib_dz_).n("dyaw", calib_dyaw_);
    agc::JsonObj rec;
    rec.v("source_topic", source_topic_);
    rec.raw("T_world_air_local", t.dump());
    rec.n("t_calib_sim", calib_sim_);
    rec.n("t_calib_wall", calib_wall_);
    if (std::isnan(residual_m_)) rec.nullv("residual_m");
    else rec.n("residual_m", residual_m_);
    rec.bv("usable", usable_ && calib_done_ && !stopped_);
    const std::string s = rec.dump();

    ros::param::set("/agc/air/calibration_json", s);
    const std::string path = agc::runDir() + "/air_calibration.json";
    ensureDir(agc::runDir());
    std::ofstream f(path);
    if (!f.is_open()) {
      ROS_WARN("failed to write air_calibration.json: %s", path.c_str());
      return;
    }
    f << s;
  }

  // ---------------------------------------------------------- 服务
  bool onRecalibrate(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      calib_done_ = false;
      stopped_ = false;
      has_prev_world_ = false;
      residuals_.clear();
      resid_finalized_ = false;
      residual_m_ = std::numeric_limits<double>::quiet_NaN();
      valid_flag_ = false;
      ros::param::set("/agc/air/odom_valid", false);
    }
    ROS_WARN("air odom recalibrate requested; awaiting next MAVROS frame");
    const double deadline = agc::wallNow() + 2.0;
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (calib_done_) break;
      }
      if (agc::wallNow() >= deadline) break;
      ros::WallDuration(0.02).sleep();  // 服务线程内有限等待，不阻塞 odom 回调
    }
    std::lock_guard<std::mutex> lk(mtx_);
    if (calib_done_) {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "recalibrated: dx=%.4f dy=%.4f dz=%.4f dyaw=%.5f rad source=%s",
                    calib_dx_, calib_dy_, calib_dz_, calib_dyaw_, source_topic_.c_str());
      res.success = true;
      res.message = buf;
      return true;
    }
    res.success = false;
    res.message = "recalibration pending: no MAVROS frame within 2.0 s";
    return true;
  }

  // 数值格式化：仅用于失效原因字符串
  static std::string num(double v) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.3f", v);
    return buf;
  }

  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber model_sub_, odom_sub_, pose_sub_, vel_sub_;
  ros::ServiceServer recal_srv_;
  ros::Timer fallback_timer_, freshness_timer_;
  std::mutex mtx_;

  std::string ns_, drone_model_, source_topic_;
  double odom_max_age_s_ = 0.5, jump_threshold_m_ = 2.0, cov_jump_threshold_ = 1.0;

  // 真值与标定
  bool have_truth_ = false;
  std::array<double, 3> truth_pos_{{0, 0, 0}};
  std::array<double, 4> truth_quat_{{0, 0, 0, 1}};
  bool calib_done_ = false;
  double calib_dx_ = 0, calib_dy_ = 0, calib_dz_ = 0, calib_dyaw_ = 0;
  double calib_sim_ = 0.0, calib_wall_ = 0.0;
  std::vector<double> residuals_;
  double residual_m_ = std::numeric_limits<double>::quiet_NaN();
  bool resid_finalized_ = false, usable_ = false, stopped_ = false;

  // 运行时
  std::array<double, 3> prev_world_{{0, 0, 0}};
  bool has_prev_world_ = false;
  double last_rx_wall_ = 0.0, last_rx_sim_ = 0.0;
  bool have_last_rx_ = false;
  geometry_msgs::TwistStamped::ConstPtr twist_fallback_;
  bool have_twist_fallback_ = false;
  bool primary_seen_ = false, fallback_active_ = false, valid_flag_ = false;
  ros::Time last_pub_stamp_;
  bool has_last_pub_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "air_odom_adapter");
  AirOdomAdapter node;
  // 服务回调内做有限等待（≤2 s），需与订阅/定时器回调并行，故用多线程 spinner
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}