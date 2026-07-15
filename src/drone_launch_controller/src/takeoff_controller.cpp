/**
 * @file takeoff_controller.cpp
 * @brief 无人机起飞控制器类实现
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#include <drone_launch_controller/takeoff_controller.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

namespace drone_launch_controller {

/**
 * @brief 无人机起飞控制器类构造函数
 * @details 超时时间默认为60秒
 * @param nh ROS节点句柄
 * @param pose_ptr 当前位置指针
 * @param local_pos_pub 本地位置发布器
 */
TakeoffController::TakeoffController(geometry_msgs::PoseStamped* pose_ptr, ros::Publisher& local_pos_pub, std::mutex& pose_mutex)
    : pose_ptr_(pose_ptr)
    , local_pos_pub_ref_(local_pos_pub)
    , pose_mutex_ref_(pose_mutex)
    , target_altitude_(0.0f)
    , takeoff_timeout_(60.0f)
    , takeoff_in_progress_(false)
    , current_state_(INIT)
    , hover_maintain_running_(false)
{
    ROS_INFO("[TakeoffController] 初始化完成");
}

/**
 * @brief 无人机起飞控制器类析构函数
 */
TakeoffController::~TakeoffController() 
{
    stopHoverMaintainThread();
    stopTakeoffThread();
    ROS_INFO("[TakeoffController] 销毁完成");
}

/**
 * @brief 获取当前无人机高度
 * @return float 当前高度值（单位：米）
 */
float TakeoffController::getCurrentAltitude() const 
{
    std::lock_guard<std::mutex> lock(pose_mutex_ref_);
    return pose_ptr_->pose.position.z;
}

/**
 * @brief 检查起飞是否正在进行中
 * @return true 如果起飞正在进行中，否则返回false
 */
bool TakeoffController::isInProgress() const 
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return takeoff_in_progress_;
}

/**
 * @brief 获取当前无人机状态
 * @return FlightState 当前状态枚举值
 */
FlightState TakeoffController::getCurrentState() const 
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

/**
 * @brief 设置当前无人机状态
 * @param state 目标状态枚举值
 */
void TakeoffController::setState(FlightState state) 
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_state_ != state) 
    {
        ROS_INFO("[TakeoffController] 状态变化: %d -> %d", current_state_, state);
        current_state_ = state;
    }
}

/**
 * @brief 启动无人机起飞
 * @param target_altitude 目标高度（单位：米）
 * @param timeout 超时时间（单位：秒）
 * @return true 如果起飞成功，否则返回false
 */
bool TakeoffController::takeoff(float target_altitude, float timeout) 
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (takeoff_in_progress_) 
        {
            ROS_WARN("[TakeoffController] 已在起飞中，无法重复起飞");
            return false;
        }
        takeoff_in_progress_ = true;
    }

    ROS_INFO("[TakeoffController] 停止悬停线程");
    stopHoverMaintainThread();

    target_altitude_ = target_altitude;
    takeoff_timeout_ = timeout;
    takeoff_start_time_ = ros::Time::now();

    setState(TAKEOFF);

    ROS_INFO("[TakeoffController] 启动起飞线程: target=%.2fm, timeout=%.1fs", 
             target_altitude, timeout);
    takeoff_thread_ = std::thread(&TakeoffController::takeoffLoop, this);

    return true;
}

/**
 * @brief 起飞线程循环
 * @details 每次循环增加0.5米高度，直到达到目标高度或超时时间
 */
void TakeoffController::takeoffLoop() 
{
    ROS_INFO("[TakeoffController] 启动起飞线程");
    ROS_INFO("[TakeoffController] 目标高度: %.2f m", target_altitude_);

    ros::Rate rate(20);
    float step_size = 0.5f;
    float current_target = 0.0f;
    int stable_count = 0;
    const int stable_threshold = 20;

    ROS_INFO("[TakeoffController] 等待 OFFBOARD 模式 stabilize...");
    ros::Duration(0.5).sleep();

    float initial_alt = getCurrentAltitude();
    ROS_INFO("[TakeoffController] 初始高度: %.2f m", initial_alt);

    while (ros::ok()) 
    {
        bool in_progress;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            in_progress = takeoff_in_progress_;
            if (!in_progress) break;
        }

        if (checkGPSLoss()) 
        {
            ROS_ERROR("[TakeoffController] GPS 信号检测不到，起飞失败");
            setState(EMERGENCY);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                takeoff_in_progress_ = false;
            }
            ROS_ERROR("[TakeoffController] 状态转换: TAKEOFF -> EMERGENCY (GPS丢失)");
            break;
        }


        if (checkControlTimeout()) 
        {
            ROS_ERROR("[TakeoffController] 起飞超时");
            setState(EMERGENCY);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                takeoff_in_progress_ = false;
            }
            ROS_ERROR("[TakeoffController] 状态转换: TAKEOFF -> EMERGENCY (超时)");
            break;
        }

        float current_alt = getCurrentAltitude();
        float target_alt;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            target_alt = target_altitude_;
        }
        
        if (current_target < target_alt) 
        {
            current_target = std::min(current_target + step_size / 20.0f, target_alt);
        }

        publishSetpoint(current_target);

        static int log_counter = 0;
        if (++log_counter >= 20) 
        {
            ROS_INFO("[TakeoffController] 上升: current=%.2f m, target=%.2f m, setpoint=%.2f m", 
                     current_alt, target_alt, current_target);
            log_counter = 0;
        }

        if (current_alt >= target_alt - 0.2f) 
        {
            stable_count++;
            if (stable_count >= stable_threshold) 
            {
                ROS_INFO("[TakeoffController] 目标高度已达到并稳定保持: %.2f m", current_alt);
                setState(HOVER);
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    takeoff_in_progress_ = false;
                }
                ROS_INFO("[TakeoffController] 状态转换: TAKEOFF -> HOVER (目标高度到达)");
                
                startHoverMaintainThread();
                break;
            }
        } 
        else 
        {
            stable_count = 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[TakeoffController] 起飞循环结束，最终状态: %d", getCurrentState());
}

/**
 * @brief 启动悬停线程
 */
void TakeoffController::startHoverMaintainThread() 
{
    if (hover_maintain_running_) 
    {
        return;
    }
    
    hover_maintain_running_ = true;
    ROS_INFO("[TakeoffController] 启动悬停线程");
    hover_maintain_thread_ = std::thread(&TakeoffController::hoverMaintainLoop, this);
}

/**
 * @brief 停止悬停线程
 */
void TakeoffController::stopHoverMaintainThread()
{
    hover_maintain_running_ = false;
    if (hover_maintain_thread_.joinable()) 
    {
        hover_maintain_thread_.join();
        ROS_INFO("[TakeoffController] 悬停线程已停止");
    }
}

/**
 * @brief 释放控制权：停止悬停发布
 */
void TakeoffController::releaseControl()
{
    ROS_INFO("[TakeoffController] 收到释放控制权请求，停止悬停发布...");
    stopHoverMaintainThread();
    ROS_INFO("[TakeoffController] 控制权已释放");
}

/**
 * @brief 悬停线程循环
 */
void TakeoffController::hoverMaintainLoop() 
{
    ROS_INFO("[TakeoffController] 启动悬停线程");
    
    ros::Rate rate(20);
    int log_counter = 0;
    
    while (ros::ok() && hover_maintain_running_) 
    {
        publishSetpoint(target_altitude_);
        
        if (++log_counter >= 100) 
        {
            ROS_INFO("[TakeoffController] 悬停高度: %.2f m", target_altitude_);
            log_counter = 0;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("[TakeoffController] 悬停线程循环结束");
}

/**
 * @brief 发布悬停高度设置点
 * @param altitude 悬停高度（单位：米）
 */
void TakeoffController::publishSetpoint(float altitude) 
{
    geometry_msgs::PoseStamped setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    
    {
        std::lock_guard<std::mutex> lock(pose_mutex_ref_);
        setpoint.pose.position.x = pose_ptr_->pose.position.x;
        setpoint.pose.position.y = pose_ptr_->pose.position.y;
        setpoint.pose.orientation = pose_ptr_->pose.orientation;
    }
    
    setpoint.pose.position.z = altitude;
    local_pos_pub_ref_.publish(setpoint);
}

/**
 * @brief 检查GPS数据丢失
 * @return true 如果GPS数据丢失，否则返回false
 */
bool TakeoffController::checkGPSLoss() 
{
    std::lock_guard<std::mutex> lock(pose_mutex_ref_);
    if (pose_ptr_->header.stamp.isZero()) 
    {
        ROS_WARN("[TakeoffController] GPS数据丢失");
        return true;
    }

    ros::Time now = ros::Time::now();
    ros::Time pose_time = pose_ptr_->header.stamp;
    double time_diff = (now - pose_time).toSec();
    
    if (time_diff > 2.0) 
    {
        ROS_WARN("[TakeoffController] GPS数据超时，时间差: %.2f 秒", time_diff);
        return true;
    }

    return false;
}

/**
 * @brief 检查控制超时
 * @return true 如果控制超时，否则返回false
 */
bool TakeoffController::checkControlTimeout() 
{
    ros::Time now = ros::Time::now();
    ros::Duration elapsed = now - takeoff_start_time_;
    return (elapsed.toSec() > takeoff_timeout_);
}

/**
 * @brief 停止起飞线程
 */
void TakeoffController::stopTakeoffThread() 
{
    if (takeoff_thread_.joinable()) 
    {
        takeoff_thread_.join();
        ROS_INFO("[TakeoffController] 起飞线程已停止");
    }
}

}
