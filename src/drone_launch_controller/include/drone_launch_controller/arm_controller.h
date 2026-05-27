/**
 * @file arm_controller.h
 * @brief 无人机电机控制类
 * @details 该类负责控制无人机的电机解锁与上锁，包括检查安全检查（地理围栏、高度限制、速度限制等）。
 * @author 陈鑫豪
 * @date 2026-05-27
 * @version 0.1
 * @note 该类依赖于Mavros消息类型和ROS服务客户端。
 */
#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Trigger.h>

namespace drone_launch_controller {

class ArmController {
public:
    /**
     * @brief 构造函数
     * @param nh：ROS句柄引用
     */
    explicit ArmController(ros::NodeHandle& nh);

    /**
     * @brief 解锁无人机电机
     * @param force：是否强制解锁（默认false）
     */
    bool armMotors(bool force = false);

    /**
     * @brief 上锁无人机电机
     */
    bool disarmMotors();

    /**
     * @brief 检查无人机电机是否已上锁
     * @return true：已上锁，false：未上锁
     */
    bool isArmed() const;

    /**
     * @brief 设置无人机电机解锁超时时间
     * @param seconds：超时时间（秒）
     */
    void setArmTimeout(double seconds);

    /**
     * @brief 检查无人机是否超出地理围栏范围
     * @return true：超出范围，false：未超出范围
     */
    bool checkGeofence() const;

    /**
     * @brief 检查无人机高度超出限制
     * @return true：超出限制，false：未超出限制
     */
    bool checkAltitudeLimit() const;

    /**
     * @brief 检查无人机速度超出限制
     * @return true：超出限制，false：未超出限制
     */
    bool checkSpeedLimit() const;

private:
    /**
     * @brief 状态回调函数
     * @param state：Mavros状态消息指针
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& state);

    ros::NodeHandle& nh_;
    ros::ServiceClient arming_client_;
    ros::Subscriber state_sub_;

    mavros_msgs::CommandBool arm_cmd_;
    bool current_armed_state_;
    double arm_timeout_seconds_;

    double max_altitude_limit_;
    double max_speed_limit_;
    double geofence_radius_;
};

}

#endif
