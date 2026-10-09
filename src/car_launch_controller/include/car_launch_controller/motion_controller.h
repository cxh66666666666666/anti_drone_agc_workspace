/**
 * @file motion_controller.h
 * @brief 地面小车运动控制器类
 * @details 通过运动学积分持续写入 /ground_gazebo/set_model_state 控制小车沿 world +x 前进，
 *          并在到位后保持位姿（对齐无人机悬停保持语义）。
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <mutex>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>

#include <car_launch_controller/health_checker.h>

namespace car_launch_controller {

// 小车状态，对齐无人机的 FlightState
enum CarState {
    INIT = 0,
    CONNECTED = 1,
    HEALTH_CHECK = 2,
    ENABLED = 3,
    DRIVING = 4,
    HOLDING = 5,
    STOPPED = 6,
    EMERGENCY = 7
};

class MotionController {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄（用于读参数与创建服务客户端）
     * @param health_checker 健康检查器指针，用于查询车模存在性与位姿
     */
    MotionController(ros::NodeHandle& nh, HealthChecker* health_checker);

    /**
     * @brief 析构函数
     */
    ~MotionController();

    /**
     * @brief 使能运动控制（接管车模）
     * @return true 表示使能成功
     */
    bool enableMotion();

    /**
     * @brief 沿 world +x 行驶指定距离
     * @param distance 目标距离（米），<=0 直接失败
     * @param speed 线速度（米/秒），<=0 取参数 ~reference_speed（默认 1.0）
     * @param timeout 超时（秒），<=0 取 30.0
     * @param achieved_distance 输出实际行驶距离
     * @return true 表示到达目标距离
     */
    bool drive(float distance, float speed, float timeout, float& achieved_distance);

    /**
     * @brief 释放控制权（可重复调用）
     */
    void releaseControl();

    /**
     * @brief 获取当前状态
     * @return 当前 CarState
     */
    CarState getCurrentState() const;

    /**
     * @brief 是否已使能（ENABLED 及之后）
     * @return true 表示已使能
     */
    bool isMotionEnabled() const;

    /**
     * @brief 获取当前线速度，非 DRIVING 时为 0
     * @return 线速度（米/秒）
     */
    float getLinearSpeed() const;

    /**
     * @brief 获取控制模式
     * @return "NONE"（未使能）或 "CONTROLLER"（本节点持有控制权）
     */
    std::string getControlMode() const;

private:
    /**
     * @brief 写入车模位姿到 set_model_state
     * @param pose 目标位姿（world 系）
     */
    void writeModelState(const geometry_msgs::Pose& pose);

    /**
     * @brief 保持定时器回调，仅 HOLDING 时写保持位姿
     * @param event ROS定时器事件
     */
    void holdTimerCallback(const ros::TimerEvent& event);

    /**
     * @brief 设置当前状态并记录变化
     * @param state 新状态
     */
    void setState(CarState state);

    ros::NodeHandle& nh_;
    HealthChecker* health_checker_;
    ros::ServiceClient set_model_state_client_;

    mutable std::mutex state_mutex_;
    CarState current_state_;
    float linear_speed_;

    geometry_msgs::Pose initial_pose_;
    geometry_msgs::Pose hold_pose_;

    std::string model_name_;
    double control_rate_;

    ros::Timer hold_timer_;
};

}

#endif