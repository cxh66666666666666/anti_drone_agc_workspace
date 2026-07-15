/**
 * @file takeoff_controller.h
 * @brief 无人机起飞控制器类
 * @details 该类负责控制无人机的起飞过程，包括起飞、悬停、降落、紧急停止等。
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#ifndef TAKEOFF_CONTROLLER_H
#define TAKEOFF_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <mutex>

namespace drone_launch_controller {

enum FlightState {
    INIT = 0,
    CONNECTED = 1,
    HEALTH_CHECK = 2,
    ARMED = 3,
    TAKEOFF = 4,
    HOVER = 5,
    LAND = 6,
    EMERGENCY = 7
};

class TakeoffController {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param pose_ptr 当前位置指针
     * @param local_pos_pub 本地位置发布器
     */
    explicit TakeoffController(geometry_msgs::PoseStamped* pose_ptr, ros::Publisher& local_pos_pub, std::mutex& pose_mutex);

    /**
     * @brief 析构函数
     */
    ~TakeoffController();
    
    /**
     * @brief 起飞
     * @param target_altitude 目标高度（米）
     * @param timeout 超时时间（秒）
     * @return true 如果起飞成功，否则返回 false
     */
    bool takeoff(float target_altitude, float timeout);

    /**
     * @brief 获取当前高度
     * @return 当前高度（米）
     */
    float getCurrentAltitude() const;

    /**
     * @brief 检查起飞是否正在进行中
     * @return true 如果起飞正在进行中，否则返回 false
     */
    bool isInProgress() const;

    /**
     * @brief 检查GPS是否丢失
     * @return true 如果GPS丢失，否则返回 false
     */
    bool checkGPSLoss();

    /**
     * @brief 检查控制超时
     * @return true 如果控制超时，否则返回 false
     */
    bool checkControlTimeout();

    /**
     * @brief 获取当前状态
     * @return 当前状态
     */
    FlightState getCurrentState() const;

    /**
     * @brief 设置当前状态
     * @param state 新状态
     */
    void setState(FlightState state);

    /**
     * @brief 释放控制权：停止悬停发布，将 /mavros/setpoint_position/local 交还给 LQR 跟踪器
     */
    void releaseControl();

private:
    /**
     * @brief 起飞循环
     */
    void takeoffLoop();

    /**
     * @brief 悬停循环
     */
    void hoverMaintainLoop();

    /**
     * @brief 发布目标高度
     * @param altitude 目标高度（米）
     */
    void publishSetpoint(float altitude);

    /**
     * @brief 停止起飞线程
     */
    void stopTakeoffThread();

    /**
     * @brief 启动悬停维护线程
     */
    void startHoverMaintainThread();
    
    /**
     * @brief 停止悬停维护线程
     */
    void stopHoverMaintainThread();

    geometry_msgs::PoseStamped* pose_ptr_;
    ros::Publisher& local_pos_pub_ref_;
    std::mutex& pose_mutex_ref_;

    float target_altitude_;
    float takeoff_timeout_;
    bool takeoff_in_progress_;
    FlightState current_state_;
    ros::Time takeoff_start_time_;
    std::thread takeoff_thread_;

    mutable std::mutex state_mutex_;

    std::thread hover_maintain_thread_;
    bool hover_maintain_running_;
};

}

#endif
