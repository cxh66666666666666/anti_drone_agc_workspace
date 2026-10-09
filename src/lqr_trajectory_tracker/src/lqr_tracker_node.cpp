/**
 * @file lqr_tracker_node.cpp
 * @brief LQR轨迹跟踪节点类实现
 * @author 陈鑫豪
 * @date 2026-06-04
 */
#include <clocale>
#include "lqr_trajectory_tracker/lqr_tracker_node.h"
#include <ros/ros.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <cmath>

namespace lqr_trajectory_tracker {

/**
 * @brief 构造函数
 * @param nh ROS节点句柄
 * @param pnh 私有节点句柄
 */
LQRTrackerNode::LQRTrackerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , has_state_(false)
    , has_mavros_state_(false)
    , rl_offset_(0.0, 0.0, 0.0)
    , has_rl_offset_(false)
    , rl_commanded_speed_scale_(1.0)
    , rl_pause_progress_(false)
    , rl_phase_direction_(1.0)
    , rl_speed_scale_(0.3)
    , rl_velocity_gain_(0.0)
    , max_rl_velocity_(1.0)
    , position_lead_time_(0.02)
    , trajectory_duration_(10.0)
    , loop_trajectory_(true)
    , control_rate_(50.0)
    , dt_(0.02)
    , q_pos_xy_(10.0)
    , q_pos_z_(15.0)
    , q_vel_xy_(1.0)
    , q_vel_z_(2.0)
    , r_accel_(0.1)
    , start_point_(0, 0, 0)
    , end_point_(10, 0, 5)
    , center_point_(5, 0, 5)
    , radius_(5.0)
    , height_(5.0)
    , angular_velocity_(0.5) 
{
    
    loadParameters();
    
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &LQRTrackerNode::poseCallback, this);
    velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10,
                                  &LQRTrackerNode::velocityCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 10, &LQRTrackerNode::stateCallback, this);
    local_offset_sub_ = nh_.subscribe("/rl_planner/local_offset", 10, &LQRTrackerNode::localOffsetCallback, this);
    local_control_sub_ = nh_.subscribe("/rl_planner/local_control", 10, &LQRTrackerNode::localControlCallback, this);
    waypoints_sub_ = nh_.subscribe("/lqr_tracker/waypoints", 10, &LQRTrackerNode::waypointsCallback, this);
    if (follow_enabled_)
    {
        follow_ref_sub_ = nh_.subscribe("/agc/air/follow_ref", 10,
                                        &LQRTrackerNode::followRefCallback, this);
        ROS_INFO("[LQR] 已启用可选跟随参考 /agc/air/follow_ref（超时 %.2fs）", follow_timeout_s_);
    }
    reset_trajectory_service_ = nh_.advertiseService(
        "/lqr_tracker/reset_trajectory", &LQRTrackerNode::resetTrajectoryCallback, this);
    reload_trajectory_service_ = nh_.advertiseService(
        "/lqr_tracker/reload_trajectory", &LQRTrackerNode::reloadTrajectoryCallback, this);
    
    pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    accel_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/lqr_tracker/accel_cmd", 10);
    ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/lqr_tracker/ref_trajectory", 10);
    adjusted_ref_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/lqr_tracker/adjusted_ref", 10);
    reference_velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        "/lqr_tracker/reference_velocity", 10);
    reference_mid_lookahead_velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        "/lqr_tracker/reference_mid_lookahead_velocity", 10);
    reference_lookahead_velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        "/lqr_tracker/reference_lookahead_velocity", 10);
    flown_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/lqr_tracker/flown_path", 10);
    pos_error_pub_ = nh_.advertise<std_msgs::Float64>("/lqr_tracker/position_error", 10);
    vel_error_pub_ = nh_.advertise<std_msgs::Float64>("/lqr_tracker/velocity_error", 10);
    total_error_pub_ = nh_.advertise<std_msgs::Float64>("/lqr_tracker/total_error", 10);
    
    generateTrajectory();

    // 初始化飞行轨迹 Marker（LINE_STRIP，红色）
    flown_path_marker_.header.frame_id = "map";
    flown_path_marker_.ns = "flown_path";
    flown_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    flown_path_marker_.action = visualization_msgs::Marker::ADD;
    flown_path_marker_.scale.x = 0.1;  // 线宽
    flown_path_marker_.color.r = 1.0;
    flown_path_marker_.color.g = 0.0;
    flown_path_marker_.color.b = 0.0;
    flown_path_marker_.color.a = 1.0;
    flown_path_marker_.lifetime = ros::Duration(0);  // 永久显示
    
    // 配置LQR位置权重矩阵
    Eigen::Matrix3d q_pos = Eigen::Matrix3d::Zero();
    q_pos(0, 0) = q_pos_xy_;
    q_pos(1, 1) = q_pos_xy_;
    q_pos(2, 2) = q_pos_z_;
    
    // 配置LQR速度权重矩阵
    Eigen::Matrix3d q_vel = Eigen::Matrix3d::Zero();
    q_vel(0, 0) = q_vel_xy_;
    q_vel(1, 1) = q_vel_xy_;
    q_vel(2, 2) = q_vel_z_;
    
    // 配置LQR加速度权重矩阵
    Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * r_accel_;
    
    lqr_controller_.setQ(q_pos, q_vel);
    lqr_controller_.setR(r);
    lqr_controller_.setDt(dt_);
    lqr_controller_.solveRiccati();
    
    ROS_INFO("[LQR] 控制器配置完成，Q_pos=[%.1f, %.1f, %.1f], Q_vel=[%.1f, %.1f, %.1f], R=%.2f",
             q_pos_xy_, q_pos_xy_, q_pos_z_, q_vel_xy_, q_vel_xy_, q_vel_z_, r_accel_);
}

/**
 * @brief 析构函数
 */
LQRTrackerNode::~LQRTrackerNode() {}

/**
 * @brief 从参数服务器加载所有配置参数
 * @details 各参数解释在config/default_params.yaml
 */
void LQRTrackerNode::loadParameters() 
{
    // 轨迹类型: linear | circular | helical | fixed_horizontal | figure_eight | step | waypoints
    pnh_.param<std::string>("trajectory_type", trajectory_type_, "fixed_horizontal");
    pnh_.param<double>("waypoint_speed", waypoint_speed_, 1.02);
    pnh_.param<double>("avoidance_lookahead_time", avoidance_lookahead_time_, 2.0);
    if (avoidance_lookahead_time_ < 0.0)
    {
        ROS_WARN("[LQR] avoidance_lookahead_time 不能为负，使用 0.0");
        avoidance_lookahead_time_ = 0.0;
    }
    pnh_.param<double>("trajectory_duration", trajectory_duration_, 30.0);
    pnh_.param<bool>("loop_trajectory", loop_trajectory_, true);
    pnh_.param<double>("control_rate", control_rate_, 50.0);
    pnh_.param<double>("rl_speed_scale", rl_speed_scale_, 0.3);  // 避障时的参考速度缩放系数
    pnh_.param<double>("rl_velocity_gain", rl_velocity_gain_, 0.0);  // 累积偏移默认只改位置参考
    pnh_.param<double>("max_rl_velocity", max_rl_velocity_, 1.0);
    pnh_.param<double>("position_lead_time", position_lead_time_, dt_);

    // N2：可选跟随参考（/agc/air/follow_ref）；默认关闭，不影响既有轨迹类型
    pnh_.param<bool>("follow_enabled", follow_enabled_, false);
    pnh_.param<double>("follow_timeout_s", follow_timeout_s_, 0.5);
    has_follow_ref_ = false;
    
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
    pnh_.param<double>("angular_velocity", angular_velocity_, 10);

    pnh_.param<double>("fixed_horizontal_left_x", fixed_horizontal_params_.left_center_x, -3.0);
    pnh_.param<double>("fixed_horizontal_left_y", fixed_horizontal_params_.left_center_y, 0.0);
    pnh_.param<double>("fixed_horizontal_right_x", fixed_horizontal_params_.right_center_x, 3.0);
    pnh_.param<double>("fixed_horizontal_right_y", fixed_horizontal_params_.right_center_y, 0.0);
    pnh_.param<double>("fixed_horizontal_radius", fixed_horizontal_params_.radius, 3.0);
    pnh_.param<double>("fixed_horizontal_height", fixed_horizontal_params_.height, 2.0);
    pnh_.param<double>("fixed_horizontal_angular_velocity", fixed_horizontal_params_.angular_velocity, 0.2);

    pnh_.param<double>("figure_eight_center_x", figure_eight_params_.center_x, 0.0);
    pnh_.param<double>("figure_eight_center_y", figure_eight_params_.center_y, 0.0);
    pnh_.param<double>("figure_eight_radius_x", figure_eight_params_.radius_x, 5.0);
    pnh_.param<double>("figure_eight_radius_y", figure_eight_params_.radius_y, 3.0);
    pnh_.param<double>("figure_eight_height", figure_eight_params_.height, 2.0);
    pnh_.param<double>("figure_eight_angular_velocity", figure_eight_params_.angular_velocity, 0.2);

    // 悬停轨迹参数（用于 RL 训练）
    pnh_.param<double>("step_x", step_position_.x(), 0.0);
    pnh_.param<double>("step_y", step_position_.y(), 0.0);
    pnh_.param<double>("step_z", step_position_.z(), 5.0);

    dt_ = 1.0 / control_rate_;
    
    ROS_INFO("[LQR] 参数加载完成");
    ROS_INFO("[LQR] 类型: %s, 轨迹持续时间: %.2f s, 控制频率: %.2f Hz", 
             trajectory_type_.c_str(), trajectory_duration_, control_rate_);
}

/**
 * @brief 根据配置参数生成参考轨迹
 */
void LQRTrackerNode::generateTrajectory() 
{
    if (trajectory_type_ == "linear") 
    {
        trajectory_ = traj_generator_.generateLinearTrajectory(
            start_point_, end_point_, trajectory_duration_, dt_);
    } 
    else if (trajectory_type_ == "circular") 
    {
        trajectory_ = traj_generator_.generateCircularTrajectory(
            center_point_, radius_, height_, angular_velocity_, trajectory_duration_, dt_);
    } 
    else if (trajectory_type_ == "helical") 
    {
        trajectory_ = traj_generator_.generateHelicalTrajectory(
            center_point_, radius_, 0.0, height_, angular_velocity_, trajectory_duration_, dt_);
    } 
    else if (trajectory_type_ == "fixed_horizontal") 
    {
        trajectory_ = traj_generator_.generateFixedHorizontalTrajectory(
            fixed_horizontal_params_, trajectory_duration_, dt_);
    }
    else if (trajectory_type_ == "figure_eight")
    {
        trajectory_ = traj_generator_.generateFigureEightTrajectory(
            figure_eight_params_, trajectory_duration_, dt_);
    }
    else if (trajectory_type_ == "step") 
    {
        trajectory_ = traj_generator_.generateStepTrajectory(
            step_position_, trajectory_duration_, dt_);
        ROS_INFO("[LQR] 悬停轨迹: (%.1f, %.1f, %.1f)", 
                 step_position_.x(), step_position_.y(), step_position_.z());
    } 
    else if (trajectory_type_ == "waypoints")
    {
        if (waypoints_.empty())
        {
            // 尚未收到 A* 航点：在起飞高度悬停等待
            Eigen::Vector3d hover(start_point_.x(), start_point_.y(), 5.0);
            trajectory_ = traj_generator_.generateStepTrajectory(
                hover, 600.0, dt_);
            ROS_INFO("[LQR] 航点轨迹: 等待航点输入，悬停于 (%.1f, %.1f, %.1f)",
                     hover.x(), hover.y(), hover.z());
        }
        else if (waypoints_.size() == 1)
        {
            trajectory_ = traj_generator_.generateStepTrajectory(
                waypoints_.front(), 600.0, dt_);
            ROS_INFO("[LQR] 单点航点保持: (%.2f, %.2f, %.2f)",
                     waypoints_.front().x(), waypoints_.front().y(),
                     waypoints_.front().z());
        }
        else
        {
            trajectory_ = traj_generator_.generateWaypointTrajectory(
                waypoints_, waypoint_speed_, dt_);
            ROS_INFO("[LQR] 航点轨迹: %zu 个航点, 速度 %.2f m/s, 生成 %zu 个轨迹点",
                     waypoints_.size(), waypoint_speed_, trajectory_.size());
        }
    }
    else 
    {
        ROS_WARN("[LQR] 未知轨迹类型: %s, 使用圆形轨迹", trajectory_type_.c_str());
        trajectory_ = traj_generator_.generateCircularTrajectory(
            center_point_, radius_, height_, angular_velocity_, trajectory_duration_, dt_);
    }
    // 中文化轨迹生成完成信息
    ROS_INFO("[LQR] 轨迹生成完成，包含 %zu 个点",  trajectory_.size());
    publishTrajectoryVisualization();
}

/**
 * @brief 订阅mavros位置数据，回调更新当前无人机的位姿和状态
 */
void LQRTrackerNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    current_pose_ = *msg;
    current_state_.position.x() = msg->pose.position.x;
    current_state_.position.y() = msg->pose.position.y;
    current_state_.position.z() = msg->pose.position.z;
    has_state_ = true;
}

void LQRTrackerNode::velocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_state_.velocity.x() = msg->twist.linear.x;
    current_state_.velocity.y() = msg->twist.linear.y;
    current_state_.velocity.z() = msg->twist.linear.z;
}

/**
 * @brief 订阅mavros状态数据，回调更新当前无人机的状态
 */
void LQRTrackerNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) 
{
    mavros_state_ = *msg;
    has_mavros_state_ = true;
}

/**
 * @brief 订阅rl_planner局部避障偏移量，回调更新参考轨迹修正量
 */
void LQRTrackerNode::localOffsetCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) 
{
    last_rl_offset_time_ = msg->header.stamp;  // 记录最后收到偏移消息的时间戳
    // 检测零偏移量：障碍物消失时清除偏移，恢复正常轨迹跟踪
    if (msg->vector.x == 0.0 && msg->vector.y == 0.0 && msg->vector.z == 0.0)
    {
        rl_offset_ = Eigen::Vector3d::Zero();
        has_rl_offset_ = false;
        return;
    }
    rl_offset_ = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
    has_rl_offset_ = true;
}

void LQRTrackerNode::localControlCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    last_rl_control_time_ = msg->header.stamp;
    rl_commanded_speed_scale_ = std::max(0.0, std::min(1.0, msg->vector.x));
    rl_pause_progress_ = msg->vector.y > 0.5;
    rl_phase_direction_ = msg->vector.z < -0.5 ? -1.0 : 1.0;
}

/**
 * @brief 接收 A* 全局路径航点：重生成时间参数化轨迹并从 t=0 开始跟踪
 */
void LQRTrackerNode::waypointsCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (msg->poses.empty())
    {
        ROS_WARN("[LQR] 收到空航点路径，忽略");
        return;
    }
    waypoints_.clear();
    for (const auto& pose : msg->poses)
    {
        waypoints_.push_back(Eigen::Vector3d(pose.pose.position.x,
                                             pose.pose.position.y,
                                             pose.pose.position.z));
    }
    generateTrajectory();
    trajectory_start_time_ = ros::Time::now();
    last_control_loop_time_ = trajectory_start_time_;
    rl_commanded_speed_scale_ = 1.0;
    rl_pause_progress_ = false;
    rl_phase_direction_ = 1.0;
    publishTrajectoryVisualization();
    ROS_INFO("[LQR] 收到新航点路径: %zu 点, 起点(%.1f,%.1f) 终点(%.1f,%.1f)",
             waypoints_.size(), waypoints_.front().x(), waypoints_.front().y(),
             waypoints_.back().x(), waypoints_.back().y());
}

/**
 * @brief 接收可选跟随参考（/agc/air/follow_ref，N2）。仅在 follow_enabled 时覆盖基础参考；
 *        超时未更新则回退到基础轨迹（不追陈旧参考）。
 */
void LQRTrackerNode::followRefCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    follow_ref_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                                  msg->pose.position.z);
    last_follow_time_ = ros::Time::now();
    has_follow_ref_ = true;
}

/**
 * @brief 将跟踪相位、RL 偏移与误差统计复位到 t=0（reset/reload 共用）
 */
void LQRTrackerNode::resetTrackingState()
{
    trajectory_start_time_ = ros::Time::now();
    last_control_loop_time_ = trajectory_start_time_;
    rl_commanded_speed_scale_ = 1.0;
    rl_pause_progress_ = false;
    rl_phase_direction_ = 1.0;
    rl_offset_ = Eigen::Vector3d::Zero();
    has_rl_offset_ = false;
    position_errors_.clear();
    velocity_errors_.clear();
    flown_path_marker_.points.clear();
}

bool LQRTrackerNode::resetTrajectoryCallback(std_srvs::Trigger::Request&,
                                             std_srvs::Trigger::Response& res)
{
    resetTrackingState();

    if (!trajectory_.empty())
    {
        publishPositionSetpoint(trajectory_.front().position);
    }

    res.success = true;
    res.message = "trajectory restarted from t=0";
    ROS_INFO("[Tracking] 收到 episode reset，参考轨迹已从 t=0 重新开始");
    return true;
}

/**
 * @brief 重载轨迹：重新读取 /lqr_tracker_node 几何参数并生成新轨迹，
 *        供训练换场景时调用（清空旧轨迹 + 相位复位到 t=0）。
 */
bool LQRTrackerNode::reloadTrajectoryCallback(std_srvs::Trigger::Request&,
                                              std_srvs::Trigger::Response& res)
{
    trajectory_.clear();
    loadParameters();
    generateTrajectory();

    resetTrackingState();

    if (!trajectory_.empty())
    {
        publishPositionSetpoint(trajectory_.front().position);
    }

    res.success = true;
    res.message = "trajectory reloaded from t=0";
    ROS_INFO("[Tracking] 换场景：轨迹几何已重载并复位到 t=0");
    return true;
}

/**
 * @brief 发布参考轨迹可视化到话题供可视化
 */
void LQRTrackerNode::publishTrajectoryVisualization() 
{
    if (trajectory_.empty()) 
    {
        ROS_WARN("[Visualization] 未生成轨迹，无法发布可视化");
        return;
    }
    
    nav_msgs::Path ref_path;
    ref_path.header.frame_id = "map";
    ref_path.header.stamp = ros::Time::now();
    
    for (const auto& point : trajectory_) 
    {
        geometry_msgs::PoseStamped pose;
        pose.header = ref_path.header;
        pose.pose.position.x = point.position.x();
        pose.pose.position.y = point.position.y();
        pose.pose.position.z = point.position.z();
        ref_path.poses.push_back(pose);
    }
    
    ref_path_pub_.publish(ref_path);
    
    ROS_INFO("[Visualization] 发布轨迹，包含 %zu 个点", trajectory_.size());
    Eigen::Vector3d minimum = trajectory_.front().position;
    Eigen::Vector3d maximum = trajectory_.front().position;
    for (const auto& point : trajectory_)
    {
        minimum = minimum.cwiseMin(point.position);
        maximum = maximum.cwiseMax(point.position);
    }
    ROS_INFO("[Visualization] 轨迹范围: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
             minimum.x(), maximum.x(), minimum.y(), maximum.y(),
             minimum.z(), maximum.z());
}

/**
 * @brief 发布位置设置点到话题供控制
 * @param position 目标位置（x, y, z）
 */
void LQRTrackerNode::publishPositionSetpoint(const Eigen::Vector3d& position) 
{
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    
    setpoint.pose.position.x = position.x();
    setpoint.pose.position.y = position.y();
    setpoint.pose.position.z = position.z();
    setpoint.pose.orientation = current_pose_.pose.orientation;
    
    pos_pub_.publish(setpoint);
}

/**
 * @brief 计算当前状态与参考轨迹点的跟踪误差
 * @param current 当前状态（位置、速度）
 * @param ref 参考轨迹点（位置、速度）
 * @return TrackingError 跟踪误差结构体，包含位置误差、速度误差和总误差
 */
TrackingError LQRTrackerNode::computeTrackingError(const State& current, const TrajectoryPoint& ref) 
{
    TrackingError error;
    error.position_error_vec = current.position - ref.position;
    error.velocity_error_vec = current.velocity - ref.velocity;
    // 计算欧几里得距离作为位置误差
    error.position_error = error.position_error_vec.norm();
    error.velocity_error = error.velocity_error_vec.norm();
    return error;
}

/**
 * @brief 发布位置误差、速度误差和总误差到话题供监控
 * @param error 跟踪误差结构体，包含位置误差、速度误差和总误差
 */
void LQRTrackerNode::publishTrackingError(const TrackingError& error) 
{
    std_msgs::Float64 pos_err_msg, vel_err_msg, total_err_msg;
    pos_err_msg.data = error.position_error;
    vel_err_msg.data = error.velocity_error;
    total_err_msg.data = std::sqrt(error.position_error * error.position_error + 
                                   error.velocity_error * error.velocity_error);
    
    pos_error_pub_.publish(pos_err_msg);
    vel_error_pub_.publish(vel_err_msg);
    total_error_pub_.publish(total_err_msg);
}

/**
 * @brief 打印跟踪统计信息（最大误差、平均误差、RMS误差）
 */
void LQRTrackerNode::logTrackingStats() 
{
    if (position_errors_.empty()) return;
    
    double sum_pos = 0.0;
    double max_position_error = 0.0;
    
    for (double err : position_errors_) 
    {
        sum_pos += err;
        if (err > max_position_error) max_position_error = err;
    }
    
    const double mean_position_error = sum_pos / position_errors_.size();
    
    ROS_INFO("========================================");
    ROS_INFO("[Tracking Statistics]");
    ROS_INFO("  总样本数: %zu", position_errors_.size());
    ROS_INFO("  最大位置误差: %.4f m", max_position_error);
    ROS_INFO("  平均位置误差: %.4f m", mean_position_error);
    ROS_INFO("  RMS误差: %.4f m", std::sqrt(sum_pos / position_errors_.size()));
    ROS_INFO("========================================");
}

/**
 * @brief 计算当前状态下的LQR控制（加速度）
 * @param current 当前状态（位置、速度）
 * @param ref 参考轨迹点（位置、速度）
 * @return Eigen::Vector3d 计算得到的加速度指令
 */
Eigen::Vector3d LQRTrackerNode::computeLQRControl(const State& current, const TrajectoryPoint& ref) 
{
    TrackingError error = computeTrackingError(current, ref);
    
    position_errors_.push_back(error.position_error);
    velocity_errors_.push_back(error.velocity_error);
    
    publishTrackingError(error);
    
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
    
    // 位置前馈：补偿 PX4 位置环的表观响应延迟。默认值保持原有
    // dt 前馈；实验可通过 position_lead_time 覆盖并独立评估。
    position_correction += ref.velocity * position_lead_time_;
    
    return ref.position + position_correction;
}

/**
 * @brief 主跟踪循环，按控制频率执行LQR控制并发布指令，支持循环飞行
 */
void LQRTrackerNode::trackingLoop() 
{
    ROS_INFO("[Tracking] 开始轨迹跟踪...");
    
    ros::Rate rate(control_rate_);
    trajectory_start_time_ = ros::Time::now();
    ros::Time last_viz_pub_time = ros::Time::now();
    int loop_count = 0;
    int print_count = 0;
    
    // 清空之前的误差记录
    position_errors_.clear();
    velocity_errors_.clear();
    
    while (ros::ok()) 
    {
        ros::spinOnce();
        
        if (!has_state_) 
        {
            ROS_WARN_THROTTLE(1.0, "[Tracking] 未收到状态估计...");
            rate.sleep();
            continue;
        }
        
        const ros::Time loop_time = ros::Time::now();
        if (last_control_loop_time_.isZero())
        {
            last_control_loop_time_ = loop_time;
        }
        if (!last_rl_control_time_.isZero() &&
            (loop_time - last_rl_control_time_).toSec() > 0.5)
        {
            // A lost planner heartbeat must never release an explicit hold.
            // Reset/recovery relies on the hold while Gazebo and PX4
            // state-estimates catch up; clearing it here lets the old
            // trajectory immediately pull the vehicle away again.  Only a
            // non-paused command is allowed to fall back to normal tracking.
            if (!rl_pause_progress_)
            {
                rl_commanded_speed_scale_ = 1.0;
                rl_phase_direction_ = 1.0;
            }
        }
        const ros::Duration loop_delta = loop_time - last_control_loop_time_;
        // Advance trajectory phase by the same scale used for reference
        // velocity. Moving the start time by the unconsumed portion makes
        // position and velocity references slow down together.
        const double phase_scale = (rl_pause_progress_ ? 0.0 :
            rl_phase_direction_ * rl_commanded_speed_scale_);
        trajectory_start_time_ += ros::Duration(
            loop_delta.toSec() * (1.0 - phase_scale));
        last_control_loop_time_ = loop_time;
        // speed_scale=0 freezes phase; intermediate values advance it slowly.
        double elapsed = (loop_time - trajectory_start_time_).toSec();
        if (elapsed < 0.0)
        {
            trajectory_start_time_ = ros::Time::now();
            elapsed = 0.0;
        }
        
        // 每1秒重新发布一次轨迹可视化，确保RViz能接收到
        if ((ros::Time::now() - last_viz_pub_time).toSec() >= 1.0) 
        {
            publishTrajectoryVisualization();
            last_viz_pub_time = ros::Time::now();
        }
        
        // 检查是否完成一次轨迹循环（使用实际轨迹时间长度）
        // waypoints 轨迹不循环：到达终点后保持最后参考点（悬停等待终止判定）
        double cycle_duration = trajectory_.empty() ? trajectory_duration_ : trajectory_.back().timestamp;
        if (loop_trajectory_ && trajectory_type_ != "waypoints" && elapsed > cycle_duration)
        {
            loop_count++;
            ROS_INFO("[Tracking] 第 %d 次循环完成（周期 %.1fs）！重启轨迹...", loop_count, cycle_duration);
            
            // 重置开始时间，实现循环
            trajectory_start_time_ = ros::Time::now();
            elapsed = 0.0;
            
            // 清空误差记录，开始新的统计
            if (!position_errors_.empty()) 
            {
                logTrackingStats();
                position_errors_.clear();
                velocity_errors_.clear();
            }
        }
        
        // 获取基础参考轨迹点。RL只提供局部避障偏移，不替代基础轨迹。
        TrajectoryPoint ref_point = traj_generator_.getPointAtTime(trajectory_, elapsed);
        geometry_msgs::Vector3Stamped reference_velocity_msg;
        reference_velocity_msg.header.stamp = ros::Time::now();
        reference_velocity_msg.header.frame_id = "map";
        // This topic defines the trajectory-forward frame for perception.
        // Reverse recovery changes the commanded phase, not what "front" means.
        reference_velocity_msg.vector.x = ref_point.velocity.x() * rl_commanded_speed_scale_;
        reference_velocity_msg.vector.y = ref_point.velocity.y() * rl_commanded_speed_scale_;
        reference_velocity_msg.vector.z = ref_point.velocity.z() * rl_commanded_speed_scale_;
        reference_velocity_pub_.publish(reference_velocity_msg);

        const double mid_lookahead_elapsed = std::min(
            elapsed + 0.5 * avoidance_lookahead_time_, cycle_duration);
        const TrajectoryPoint mid_lookahead_ref =
            traj_generator_.getPointAtTime(trajectory_, mid_lookahead_elapsed);
        geometry_msgs::Vector3Stamped mid_lookahead_velocity_msg;
        mid_lookahead_velocity_msg.header = reference_velocity_msg.header;
        mid_lookahead_velocity_msg.vector.x = mid_lookahead_ref.velocity.x();
        mid_lookahead_velocity_msg.vector.y = mid_lookahead_ref.velocity.y();
        mid_lookahead_velocity_msg.vector.z = mid_lookahead_ref.velocity.z();
        reference_mid_lookahead_velocity_pub_.publish(mid_lookahead_velocity_msg);

        const double lookahead_elapsed = std::min(
            elapsed + avoidance_lookahead_time_, cycle_duration);
        const TrajectoryPoint lookahead_ref =
            traj_generator_.getPointAtTime(trajectory_, lookahead_elapsed);
        geometry_msgs::Vector3Stamped lookahead_velocity_msg;
        lookahead_velocity_msg.header = reference_velocity_msg.header;
        lookahead_velocity_msg.vector.x = lookahead_ref.velocity.x();
        lookahead_velocity_msg.vector.y = lookahead_ref.velocity.y();
        lookahead_velocity_msg.vector.z = lookahead_ref.velocity.z();
        reference_lookahead_velocity_pub_.publish(lookahead_velocity_msg);

        // RL偏移超时保护：超过0.5s未收到 /rl_planner/local_offset 则自动清零
        if (has_rl_offset_ && (ros::Time::now() - last_rl_offset_time_).toSec() > 0.5)
        {
            ROS_WARN_THROTTLE(2.0, "[Tracking] RL偏移超时(%.2fs)，自动清零",
                              (ros::Time::now() - last_rl_offset_time_).toSec());
            rl_offset_ = Eigen::Vector3d::Zero();
            has_rl_offset_ = false;
        }

        if (has_rl_offset_)
        {
            ref_point.position += rl_offset_;
            Eigen::Vector3d rl_velocity = rl_offset_ * rl_velocity_gain_;
            double rl_velocity_norm = rl_velocity.norm();
            if (rl_velocity_norm > max_rl_velocity_ && rl_velocity_norm > 1e-6)
            {
                rl_velocity *= max_rl_velocity_ / rl_velocity_norm;
            }
            ref_point.velocity += rl_velocity;
        }
        // Speed control is independent of whether the local position offset is
        // zero, so the warning zone can slow down while still following the
        // unmodified path.
        ref_point.velocity *= phase_scale;

        // N2：可选跟随参考覆盖基础参考（唯一 setpoint 发布者仍是本节点；超时回退基础轨迹）
        if (follow_enabled_ && has_follow_ref_ &&
            (ros::Time::now() - last_follow_time_).toSec() <= follow_timeout_s_)
        {
            ref_point.position = follow_ref_;
            ref_point.velocity = Eigen::Vector3d::Zero();
        }

        geometry_msgs::PoseStamped adjusted_ref_msg;
        adjusted_ref_msg.header.stamp = ros::Time::now();
        adjusted_ref_msg.header.frame_id = "map";
        adjusted_ref_msg.pose.position.x = ref_point.position.x();
        adjusted_ref_msg.pose.position.y = ref_point.position.y();
        adjusted_ref_msg.pose.position.z = ref_point.position.z();
        adjusted_ref_msg.pose.orientation = current_pose_.pose.orientation;
        adjusted_ref_pub_.publish(adjusted_ref_msg);
        
        // 使用LQR计算控制指令
        Eigen::Vector3d desired_position = computeLQRControl(current_state_, ref_point);
        
        // 发布位置指令
        publishPositionSetpoint(desired_position);
        
        // 每5秒打印一次跟踪状态
        if (++print_count % (5 * static_cast<int>(control_rate_)) == 0) 
        {
            TrackingError current_error = computeTrackingError(current_state_, ref_point);
            ROS_INFO("[Tracking] 第 %d 次循环, t=%.1fs, Pos=[%.2f, %.2f, %.2f], Error=%.3fm", 
                     loop_count + 1,
                     elapsed,
                     current_state_.position.x(),
                     current_state_.position.y(),
                     current_state_.position.z(),
                     current_error.position_error);
        }
        
        // 更新飞行轨迹 Marker，追加当前位置
        geometry_msgs::Point pt;
        pt.x = current_state_.position.x();
        pt.y = current_state_.position.y();
        pt.z = current_state_.position.z();
        flown_path_marker_.points.push_back(pt);

        // 限制轨迹点数量，防止长时间运行内存无限增长（5000点 ≈ 100秒 @50Hz）
        static const size_t kMaxFlownPathPoints = 5000;
        if (flown_path_marker_.points.size() > kMaxFlownPathPoints)
        {
            size_t excess = flown_path_marker_.points.size() - kMaxFlownPathPoints;
            flown_path_marker_.points.erase(flown_path_marker_.points.begin(),
                                            flown_path_marker_.points.begin() + excess);
        }

        flown_path_marker_.header.stamp = ros::Time::now();
        flown_path_pub_.publish(flown_path_marker_);

        rate.sleep();
    }
    
    // 打印统计信息
    logTrackingStats();
}

/**
 * @brief 主入口函数
 */
void LQRTrackerNode::run() {
    ROS_INFO("========================================");
    ROS_INFO("LQR轨迹跟踪器启动...");
    ROS_INFO("========================================");
    
    ROS_INFO("[Init] 等待MAVROS连接...");
    ros::Rate check_rate(10);
    while (ros::ok()) 
    {
        ros::spinOnce();
        if (has_state_ && has_mavros_state_ && mavros_state_.connected) 
        {
            ROS_INFO("[Init] MAVROS连接成功, Altitude: %.2f m", current_pose_.pose.position.z);
            break;
        }
        check_rate.sleep();
    }

    // 请求 drone_launch_controller 释放控制权（停止悬停发布）
    ros::ServiceClient release_client = nh_.serviceClient<std_srvs::Trigger>("/drone_launch_controller/release_control");
    std_srvs::Trigger release_srv;
    if (release_client.call(release_srv))
    {
        ROS_INFO("[Init] 控制权释放结果: %s", release_srv.response.message.c_str());
    }
    else
    {
        ROS_WARN("[Init] 无法联系 drone_launch_controller 释放控制权，可能已释放或未运行");
    }
    // 交接窗口必须尽量短：交接期无 setpoint 发布会造成 OFFBOARD 空窗
    ros::Duration(0.05).sleep();
    
    trackingLoop();
    
    ROS_INFO("========================================");
    ROS_INFO("LQR轨迹跟踪器完成跟踪");
    ROS_INFO("========================================");
}

}

int main(int argc, char** argv) 
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lqr_tracker_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try 
    {
        lqr_trajectory_tracker::LQRTrackerNode node(nh, pnh);
        node.run();
    } 
    catch (const std::exception& e) 
    {
        ROS_ERROR("LQR轨迹跟踪器异常: %s", e.what());
        return 1;
    }
    
    return 0;
}
