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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>
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
    , angular_velocity_(0.5) 
{
    
    loadParameters();
    
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &LQRTrackerNode::poseCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 10, &LQRTrackerNode::stateCallback, this);
    local_offset_sub_ = nh_.subscribe("/rl_planner/local_offset", 10, &LQRTrackerNode::localOffsetCallback, this);
    
    pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    accel_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/lqr_tracker/accel_cmd", 10);
    ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/lqr_tracker/ref_trajectory", 10);
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
    // 轨迹类型: linear | circular | helical | fixed_horizontal
    pnh_.param<std::string>("trajectory_type", trajectory_type_, "fixed_horizontal");
    pnh_.param<double>("trajectory_duration", trajectory_duration_, 30.0);
    pnh_.param<double>("control_rate", control_rate_, 50.0);
    
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

    pnh_.param<double>("fixed_horizontal_left_x", fixed_horizontal_params_.left_center_x, -3.0);
    pnh_.param<double>("fixed_horizontal_left_y", fixed_horizontal_params_.left_center_y, 0.0);
    pnh_.param<double>("fixed_horizontal_right_x", fixed_horizontal_params_.right_center_x, 3.0);
    pnh_.param<double>("fixed_horizontal_right_y", fixed_horizontal_params_.right_center_y, 0.0);
    pnh_.param<double>("fixed_horizontal_radius", fixed_horizontal_params_.radius, 3.0);
    pnh_.param<double>("fixed_horizontal_height", fixed_horizontal_params_.height, 2.0);
    pnh_.param<double>("fixed_horizontal_angular_velocity", fixed_horizontal_params_.angular_velocity, 0.5);

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
    rl_offset_ = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
    has_rl_offset_ = true;
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
    ROS_INFO("[Visualization] 轨迹范围: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
             trajectory_.front().position.x(), trajectory_.back().position.x(),
             trajectory_.front().position.y(), trajectory_.back().position.y(),
             trajectory_.front().position.z(), trajectory_.back().position.z());
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
    max_position_error_ = 0.0;
    
    for (double err : position_errors_) 
    {
        sum_pos += err;
        if (err > max_position_error_) max_position_error_ = err;
    }
    
    mean_position_error_ = sum_pos / position_errors_.size();
    
    ROS_INFO("========================================");
    ROS_INFO("[Tracking Statistics]");
    ROS_INFO("  总样本数: %zu", position_errors_.size());
    ROS_INFO("  最大位置误差: %.4f m", max_position_error_);
    ROS_INFO("  平均位置误差: %.4f m", mean_position_error_);
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
    
    // 添加速度前馈：p = p_ref + v_ref * dt + 0.5 * a * dt^2
    position_correction += ref.velocity * dt_;
    
    return ref.position + position_correction;
}

/**
 * @brief 主跟踪循环，按控制频率执行LQR控制并发布指令，支持循环飞行
 */
void LQRTrackerNode::trackingLoop() 
{
    ROS_INFO("[Tracking] 开始轨迹跟踪...");
    
    ros::Rate rate(control_rate_);
    ros::Time start_time = ros::Time::now();
    ros::Time last_viz_pub_time = ros::Time::now();
    int loop_count = 0;
    
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
        
        // 计算当前时间与开始时间的差值（单位：秒）
        double elapsed = (ros::Time::now() - start_time).toSec();
        
        // 每1秒重新发布一次轨迹可视化，确保RViz能接收到
        if ((ros::Time::now() - last_viz_pub_time).toSec() >= 1.0) 
        {
            publishTrajectoryVisualization();
            last_viz_pub_time = ros::Time::now();
        }
        
        // 检查是否完成一次轨迹循环
        if (elapsed > trajectory_duration_) 
        {
            loop_count++;
            ROS_INFO("[Tracking] 第 %d 次循环完成！重启轨迹...", loop_count);
            
            // 重置开始时间，实现循环
            start_time = ros::Time::now();
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

        if (has_rl_offset_)
        {
            ref_point.position += rl_offset_;
        }
        
        // 使用LQR计算控制指令
        Eigen::Vector3d desired_position = computeLQRControl(current_state_, ref_point);
        
        // 发布位置指令
        publishPositionSetpoint(desired_position);
        
        // 每5秒打印一次跟踪状态
        static int print_count = 0;
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
