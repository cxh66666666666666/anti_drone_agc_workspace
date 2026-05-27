#include "lqr_trajectory_tracker/lqr_tracker_node.h"
#include <ros/ros.h>
#include <cmath>

namespace lqr_trajectory_tracker {

LQRTrackerNode::LQRTrackerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , has_state_(false)
    , has_mavros_state_(false)
    , trajectory_duration_(10.0)
    , control_rate_(50.0)
    , dt_(0.02)
    , q_pos_xy_(10.0)
    , q_pos_z_(15.0)
    , q_vel_xy_(1.0)
    , q_vel_z_(2.0)
    , r_accel_(0.1)
    , max_position_error_(0.0)
    , mean_position_error_(0.0)
    , start_point_(0, 0, 0)
    , end_point_(10, 0, 5)
    , center_point_(5, 0, 5)
    , radius_(5.0)
    , height_(5.0)
    , angular_velocity_(0.5) {
    
    loadParameters();
    
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &LQRTrackerNode::poseCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 10, &LQRTrackerNode::stateCallback, this);
    
    pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    accel_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/lqr_tracker/accel_cmd", 10);
    ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/lqr_tracker/ref_trajectory", 10);
    pos_error_pub_ = nh_.advertise<std_msgs::Float64>("/lqr_tracker/position_error", 10);
    vel_error_pub_ = nh_.advertise<std_msgs::Float64>("/lqr_tracker/velocity_error", 10);
    total_error_pub_ = nh_.advertise<std_msgs::Float64>("/lqr_tracker/total_error", 10);
    
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    param_set_client_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    
    generateTrajectory();
    
    // 配置LQR权重矩阵
    Eigen::Matrix3d q_pos = Eigen::Matrix3d::Zero();
    q_pos(0, 0) = q_pos_xy_;
    q_pos(1, 1) = q_pos_xy_;
    q_pos(2, 2) = q_pos_z_;
    
    Eigen::Matrix3d q_vel = Eigen::Matrix3d::Zero();
    q_vel(0, 0) = q_vel_xy_;
    q_vel(1, 1) = q_vel_xy_;
    q_vel(2, 2) = q_vel_z_;
    
    Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * r_accel_;
    
    lqr_controller_.setQ(q_pos, q_vel);
    lqr_controller_.setR(r);
    lqr_controller_.setDt(dt_);
    lqr_controller_.solveRiccati();
    
    ROS_INFO("[LQR] Controller configured with Q_pos=[%.1f, %.1f, %.1f], Q_vel=[%.1f, %.1f, %.1f], R=%.2f",
             q_pos_xy_, q_pos_xy_, q_pos_z_, q_vel_xy_, q_vel_xy_, q_vel_z_, r_accel_);
}

LQRTrackerNode::~LQRTrackerNode() {}

// loadParameters: 从参数服务器加载所有配置参数
void LQRTrackerNode::loadParameters() {
    pnh_.param<std::string>("trajectory_type", trajectory_type_, "circular");
    pnh_.param<double>("trajectory_duration", trajectory_duration_, 30.0);
    pnh_.param<double>("control_rate", control_rate_, 50.0);
    
    // LQR权重参数
    pnh_.param<double>("q_pos_xy", q_pos_xy_, 10.0);
    pnh_.param<double>("q_pos_z", q_pos_z_, 15.0);
    pnh_.param<double>("q_vel_xy", q_vel_xy_, 1.0);
    pnh_.param<double>("q_vel_z", q_vel_z_, 2.0);
    pnh_.param<double>("r_accel", r_accel_, 0.1);
    
    pnh_.param<double>("start_x", start_point_.x(), 0.0);
    pnh_.param<double>("start_y", start_point_.y(), 0.0);
    pnh_.param<double>("start_z", start_point_.z(), 0.0);
    
    pnh_.param<double>("end_x", end_point_.x(), 10.0);
    pnh_.param<double>("end_y", end_point_.y(), 0.0);
    pnh_.param<double>("end_z", end_point_.z(), 5.0);
    
    pnh_.param<double>("center_x", center_point_.x(), 0.0);
    pnh_.param<double>("center_y", center_point_.y(), 0.0);
    pnh_.param<double>("center_z", center_point_.z(), 5.0);
    
    pnh_.param<double>("radius", radius_, 3.0);
    pnh_.param<double>("height", height_, 5.0);
    pnh_.param<double>("angular_velocity", angular_velocity_, 0.3);

    pnh_.param<int>("random_num_waypoints", random_params_.num_waypoints, 10);
    pnh_.param<double>("random_space_min_x", random_params_.space_min_x, -20.0);
    pnh_.param<double>("random_space_max_x", random_params_.space_max_x, 20.0);
    pnh_.param<double>("random_space_min_y", random_params_.space_min_y, -20.0);
    pnh_.param<double>("random_space_max_y", random_params_.space_max_y, 20.0);
    pnh_.param<double>("random_space_min_z", random_params_.space_min_z, 2.0);
    pnh_.param<double>("random_space_max_z", random_params_.space_max_z, 10.0);
    pnh_.param<double>("random_max_velocity", random_params_.max_velocity, 5.0);
    pnh_.param<bool>("random_closed_loop", random_params_.closed_loop, false);
    pnh_.param<int>("random_seed", random_params_.seed, 42);

    pnh_.param<double>("fixed_horizontal_left_x", fixed_horizontal_params_.left_center_x, -3.0);
    pnh_.param<double>("fixed_horizontal_left_y", fixed_horizontal_params_.left_center_y, 0.0);
    pnh_.param<double>("fixed_horizontal_right_x", fixed_horizontal_params_.right_center_x, 3.0);
    pnh_.param<double>("fixed_horizontal_right_y", fixed_horizontal_params_.right_center_y, 0.0);
    pnh_.param<double>("fixed_horizontal_radius", fixed_horizontal_params_.radius, 3.0);
    pnh_.param<double>("fixed_horizontal_height", fixed_horizontal_params_.height, 2.0);
    pnh_.param<double>("fixed_horizontal_angular_velocity", fixed_horizontal_params_.angular_velocity, 0.5);

    dt_ = 1.0 / control_rate_;
    
    ROS_INFO("LQR Tracker Node initialized");
    ROS_INFO("Trajectory type: %s, Duration: %.2f s, Control rate: %.2f Hz", 
             trajectory_type_.c_str(), trajectory_duration_, control_rate_);
}

// generateTrajectory: 根据配置参数生成参考轨迹
void LQRTrackerNode::generateTrajectory() {
    if (trajectory_type_ == "linear") {
        trajectory_ = traj_generator_.generateLinearTrajectory(
            start_point_, end_point_, trajectory_duration_, dt_);
    } else if (trajectory_type_ == "circular") {
        trajectory_ = traj_generator_.generateCircularTrajectory(
            center_point_, radius_, height_, angular_velocity_, trajectory_duration_, dt_);
    } else if (trajectory_type_ == "helical") {
        trajectory_ = traj_generator_.generateHelicalTrajectory(
            center_point_, radius_, 0.0, height_, angular_velocity_, trajectory_duration_, dt_);
    } else if (trajectory_type_ == "random") {
        trajectory_ = traj_generator_.generateRandomTrajectory(
            random_params_, trajectory_duration_, dt_);
    } else if (trajectory_type_ == "fixed_horizontal") {
        trajectory_ = traj_generator_.generateFixedHorizontalTrajectory(
            fixed_horizontal_params_, trajectory_duration_, dt_);
    } else {
        ROS_WARN("Unknown trajectory type: %s, using circular", trajectory_type_.c_str());
        trajectory_ = traj_generator_.generateCircularTrajectory(
            center_point_, radius_, height_, angular_velocity_, trajectory_duration_, dt_);
    }
    
    ROS_INFO("Generated trajectory with %zu points", trajectory_.size());
    publishTrajectoryVisualization();
}

// poseCallback: 订阅mavros位置数据，回调更新当前无人机的位姿和状态
void LQRTrackerNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    current_state_.position.x() = msg->pose.position.x;
    current_state_.position.y() = msg->pose.position.y;
    current_state_.position.z() = msg->pose.position.z;
    has_state_ = true;
}

// stateCallback: 订阅mavros状态数据，回调更新当前无人机的状态
void LQRTrackerNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    mavros_state_ = *msg;
    has_mavros_state_ = true;
}

// publishTrajectoryVisualization: 发布参考轨迹可视化到话题供可视化
void LQRTrackerNode::publishTrajectoryVisualization() {
    if (trajectory_.empty()) {
        ROS_WARN("[Visualization] Trajectory is empty, cannot publish visualization");
        return;
    }
    
    nav_msgs::Path ref_path;
    ref_path.header.frame_id = "map";
    ref_path.header.stamp = ros::Time::now();
    
    for (const auto& point : trajectory_) {
        geometry_msgs::PoseStamped pose;
        pose.header = ref_path.header;
        pose.pose.position.x = point.position.x();
        pose.pose.position.y = point.position.y();
        pose.pose.position.z = point.position.z();
        ref_path.poses.push_back(pose);
    }
    
    ref_path_pub_.publish(ref_path);
    
    // 输出调试信息
    ROS_INFO("[Visualization] Published trajectory with %zu points", trajectory_.size());
    ROS_INFO("[Visualization] Trajectory range: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
             trajectory_.front().position.x(), trajectory_.back().position.x(),
             trajectory_.front().position.y(), trajectory_.back().position.y(),
             trajectory_.front().position.z(), trajectory_.back().position.z());
}

// publishPositionSetpoint: 发布位置设置点到话题供控制
void LQRTrackerNode::publishPositionSetpoint(const Eigen::Vector3d& position) {
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    
    setpoint.pose.position.x = position.x();
    setpoint.pose.position.y = position.y();
    setpoint.pose.position.z = position.z();
    setpoint.pose.orientation = current_pose_.pose.orientation;
    
    pos_pub_.publish(setpoint);
}

// computeTrackingError: 计算当前状态与参考轨迹点的跟踪误差
TrackingError LQRTrackerNode::computeTrackingError(const State& current, const TrajectoryPoint& ref) {
    TrackingError error;
    error.position_error_vec = current.position - ref.position;
    error.velocity_error_vec = current.velocity - ref.velocity;
    error.position_error = error.position_error_vec.norm();
    error.velocity_error = error.velocity_error_vec.norm();
    return error;
}

// publishTrackingError: 发布位置误差、速度误差和总误差到话题供监控
void LQRTrackerNode::publishTrackingError(const TrackingError& error) {
    std_msgs::Float64 pos_err_msg, vel_err_msg, total_err_msg;
    pos_err_msg.data = error.position_error;
    vel_err_msg.data = error.velocity_error;
    total_err_msg.data = std::sqrt(error.position_error * error.position_error + 
                                   error.velocity_error * error.velocity_error);
    
    pos_error_pub_.publish(pos_err_msg);
    vel_error_pub_.publish(vel_err_msg);
    total_error_pub_.publish(total_err_msg);
}

// logTrackingStats: 打印跟踪统计信息（最大误差、平均误差、RMS误差）
void LQRTrackerNode::logTrackingStats() {
    if (position_errors_.empty()) return;
    
    double sum_pos = 0.0;
    max_position_error_ = 0.0;
    
    for (double err : position_errors_) {
        sum_pos += err;
        if (err > max_position_error_) max_position_error_ = err;
    }
    
    mean_position_error_ = sum_pos / position_errors_.size();
    
    ROS_INFO("========================================");
    ROS_INFO("[Tracking Statistics]");
    ROS_INFO("  Total samples: %zu", position_errors_.size());
    ROS_INFO("  Max position error: %.4f m", max_position_error_);
    ROS_INFO("  Mean position error: %.4f m", mean_position_error_);
    ROS_INFO("  RMS position error: %.4f m", std::sqrt(sum_pos / position_errors_.size()));
    ROS_INFO("========================================");
}

// computeLQRControl: 计算当前状态下的LQR控制（加速度）
Eigen::Vector3d LQRTrackerNode::computeLQRControl(const State& current, const TrajectoryPoint& ref) {
    // 计算跟踪误差
    TrackingError error = computeTrackingError(current, ref);
    
    // 记录误差
    position_errors_.push_back(error.position_error);
    velocity_errors_.push_back(error.velocity_error);
    
    // 发布误差
    publishTrackingError(error);
    
    // 使用LQR控制器计算控制量（加速度）
    State ref_state;
    ref_state.position = ref.position;
    ref_state.velocity = ref.velocity;
    
    ControlOutput control = lqr_controller_.computeControl(current, ref_state);
    
    // 发布控制指令用于调试
    geometry_msgs::AccelStamped accel_msg;
    accel_msg.header.stamp = ros::Time::now();
    accel_msg.header.frame_id = "map";
    accel_msg.accel.linear.x = control.acceleration.x();
    accel_msg.accel.linear.y = control.acceleration.y();
    accel_msg.accel.linear.z = control.acceleration.z();
    accel_pub_.publish(accel_msg);
    
    // 计算期望位置：参考位置 + LQR修正
    // 使用双积分器模型：p = p_ref + 0.5 * a * dt^2
    Eigen::Vector3d position_correction = 0.5 * control.acceleration * dt_ * dt_;
    
    // 添加速度前馈：p = p_ref + v_ref * dt + 0.5 * a * dt^2
    position_correction += ref.velocity * dt_;
    
    return ref.position + position_correction;
}

// ===============================================================
// 后续内容为验证修复，非LQR相关代码，参考drone_local_controller包中的实现
// ===============================================================
bool LQRTrackerNode::disableRTLAndFailsafe() {
    ROS_INFO("[PX4 Config] Disabling RTL and RC failsafe for SITL...");
    
    mavros_msgs::ParamSet param_set;
    param_set.request.param_id = "NAV_RCL_ACT";
    param_set.request.value.integer = 0;
    param_set.request.value.real = 0.0;
    
    if (param_set_client_.call(param_set)) {
        if (param_set.response.success) {
            ROS_INFO("[PX4 Config] NAV_RCL_ACT set to 0 (disabled)");
        }
    }
    
    param_set.request.param_id = "NAV_DLL_ACT";
    param_set.request.value.integer = 0;
    param_set.request.value.real = 0.0;
    
    if (param_set_client_.call(param_set)) {
        if (param_set.response.success) {
            ROS_INFO("[PX4 Config] NAV_DLL_ACT set to 0 (disabled)");
        }
    }
    
    param_set.request.param_id = "COM_RCL_EXCEPT";
    param_set.request.value.integer = 4;
    param_set.request.value.real = 4.0;
    
    if (param_set_client_.call(param_set)) {
        if (param_set.response.success) {
            ROS_INFO("[PX4 Config] COM_RCL_EXCEPT set to 4 (allow offboard without RC)");
        }
    }
    
    ros::Duration(0.5).sleep();
    return true;
}

bool LQRTrackerNode::sendSetpointsBeforeOffboard(double duration_seconds, double rate_hz) {
    ROS_INFO("[Setpoint Pre-send] Sending setpoints for %.1f seconds at %.1f Hz...", duration_seconds, rate_hz);
    
    if (!has_state_) {
        ROS_ERROR("[Setpoint Pre-send] No local position received!");
        return false;
    }

    ros::Rate rate(rate_hz);
    int num_setpoints = static_cast<int>(duration_seconds * rate_hz);
    
    for (int i = 0; i < num_setpoints && ros::ok(); ++i) {
        publishPositionSetpoint(current_state_.position);
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("[Setpoint Pre-send] Sent %d setpoints", num_setpoints);
    return true;
}

bool LQRTrackerNode::waitForOffboardMode(double timeout_seconds) {
    ROS_INFO("[Offboard Mode] Waiting for OFFBOARD mode...");
    ros::Time start = ros::Time::now();
    ros::Rate rate(10);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        if (mavros_state_.mode == "OFFBOARD") {
            ROS_INFO("[Offboard Mode] Confirmed");
            return true;
        }
        
        if ((ros::Time::now() - start).toSec() > timeout_seconds) {
            ROS_ERROR("[Offboard Mode] Timeout. Current mode: %s", mavros_state_.mode.c_str());
            return false;
        }
        
        rate.sleep();
    }
    return false;
}

// setOffboardMode: 请求切换到OFFBOARD模式，支持重试机制
bool LQRTrackerNode::setOffboardMode(int max_retries) {
    ROS_INFO("[Offboard Mode] Setting OFFBOARD mode...");
    
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = "OFFBOARD";
        
        if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
            if (waitForOffboardMode(3.0)) {
                ros::Duration(0.5).sleep();
                return true;
            }
        }
        
        if (attempt < max_retries) {
            ROS_WARN("[Offboard Mode] Retrying... (%d/%d)", attempt, max_retries);
            ros::Duration(1.0).sleep();
        }
    }
    
    ROS_ERROR("[Offboard Mode] Failed after %d attempts", max_retries);
    return false;
}

// armMotorsWithCheck: 检查前置条件后解锁电机
bool LQRTrackerNode::armMotorsWithCheck() {
    ROS_INFO("[Arming] Arming motors...");
    
    if (!has_mavros_state_ || !mavros_state_.connected) {
        ROS_ERROR("[Arming] MAVROS not connected");
        return false;
    }
    
    if (mavros_state_.mode != "OFFBOARD") {
        ROS_ERROR("[Arming] Not in OFFBOARD mode: %s", mavros_state_.mode.c_str());
        return false;
    }
    
    if (!has_state_) {
        ROS_ERROR("[Arming] No local position");
        return false;
    }
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("[Arming] Motors armed");
        ros::Duration(0.5).sleep();
        return true;
    }
    
    ROS_ERROR("[Arming] Failed");
    return false;
}

// trackingLoop: 主跟踪循环，按控制频率执行LQR控制并发布指令，支持循环飞行
void LQRTrackerNode::trackingLoop() {
    ROS_INFO("[Tracking] Starting trajectory tracking (loop mode)...");
    
    ros::Rate rate(control_rate_);
    ros::Time start_time = ros::Time::now();
    ros::Time last_viz_pub_time = ros::Time::now();
    int loop_count = 0;
    
    // 清空之前的误差记录
    position_errors_.clear();
    velocity_errors_.clear();
    
    while (ros::ok()) {
        ros::spinOnce();
        
        if (!has_state_) {
            ROS_WARN_THROTTLE(1.0, "[Tracking] Lost state estimate...");
            rate.sleep();
            continue;
        }
        
        double elapsed = (ros::Time::now() - start_time).toSec();
        
        // 每1秒重新发布一次轨迹可视化，确保RViz能接收到
        if ((ros::Time::now() - last_viz_pub_time).toSec() >= 1.0) {
            publishTrajectoryVisualization();
            last_viz_pub_time = ros::Time::now();
        }
        
        // 检查是否完成一次轨迹循环
        if (elapsed > trajectory_duration_) {
            loop_count++;
            ROS_INFO("[Tracking] Loop %d completed! Restarting trajectory...", loop_count);
            
            // 重置开始时间，实现循环
            start_time = ros::Time::now();
            elapsed = 0.0;
            
            // 清空误差记录，开始新的统计
            if (!position_errors_.empty()) {
                logTrackingStats();
                position_errors_.clear();
                velocity_errors_.clear();
            }
        }
        
        // 获取当前参考轨迹点
        TrajectoryPoint ref_point = traj_generator_.getPointAtTime(trajectory_, elapsed);
        
        // 使用LQR计算控制指令
        Eigen::Vector3d desired_position = computeLQRControl(current_state_, ref_point);
        
        // 发布位置指令
        publishPositionSetpoint(desired_position);
        
        // 每5秒打印一次跟踪状态
        static int print_count = 0;
        if (++print_count % (5 * static_cast<int>(control_rate_)) == 0) {
            TrackingError current_error = computeTrackingError(current_state_, ref_point);
            ROS_INFO("[Tracking] Loop=%d, t=%.1fs, Pos=[%.2f, %.2f, %.2f], Error=%.3fm", 
                     loop_count + 1,
                     elapsed,
                     current_state_.position.x(),
                     current_state_.position.y(),
                     current_state_.position.z(),
                     current_error.position_error);
        }
        
        rate.sleep();
    }
    
    // 打印统计信息
    logTrackingStats();
}

// run: 主入口函数，完成初始化、模式切换、解锁后启动跟踪
void LQRTrackerNode::run() {
    ROS_INFO("========================================");
    ROS_INFO("LQR Trajectory Tracker Starting...");
    ROS_INFO("========================================");
    
    // 等待MAVROS连接
    ROS_INFO("[Init] Waiting for MAVROS...");
    ros::Rate check_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (has_state_ && has_mavros_state_ && mavros_state_.connected) {
            ROS_INFO("[Init] MAVROS connected, Altitude: %.2f m", current_pose_.pose.position.z);
            break;
        }
        check_rate.sleep();
    }
    
    // 配置PX4
    disableRTLAndFailsafe();
    
    // 预发送setpoints
    if (!sendSetpointsBeforeOffboard(2.0, 20.0)) {
        ROS_ERROR("[Init] Failed to pre-send setpoints");
        return;
    }
    
    // 设置OFFBOARD模式
    if (mavros_state_.mode != "OFFBOARD") {
        if (!setOffboardMode(3)) {
            ROS_ERROR("[Init] Failed to set OFFBOARD mode");
            return;
        }
    }
    
    // 解锁
    if (!mavros_state_.armed) {
        if (!armMotorsWithCheck()) {
            ROS_ERROR("[Init] Failed to arm");
            return;
        }
    }
    
    ros::Duration(2.0).sleep();
    
    // 开始跟踪
    trackingLoop();
    
    ROS_INFO("========================================");
    ROS_INFO("LQR Trajectory Tracker Finished");
    ROS_INFO("========================================");
}

}  // namespace lqr_trajectory_tracker

int main(int argc, char** argv) {
    ros::init(argc, argv, "lqr_tracker_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try {
        lqr_trajectory_tracker::LQRTrackerNode node(nh, pnh);
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}
