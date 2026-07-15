/**
 * @file drone_launch_controller.h
 * @brief 无人机启动控制器节点类声明
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#ifndef DRONE_LAUNCH_CONTROLLER_H
#define DRONE_LAUNCH_CONTROLLER_H

#include <ros/ros.h>
#include <drone_launch_controller/takeoff_controller.h>
#include <drone_launch_controller/flight_status_publisher.h>
#include <drone_launch_controller/health_checker.h>
#include <drone_launch_controller/Takeoff.h>
#include <drone_launch_controller/DroneHealthCheck.h>
#include <drone_launch_controller/ArmMotors.h>
#include <drone_launch_controller/ConnectionStatus.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>

namespace drone_launch_controller {

class DroneLaunchControllerNode {
public:
    /**
     * @brief 构造函数
     */
    DroneLaunchControllerNode();

    /**
     * @brief 运行无人机启动控制器节点
     */
    void run();

private:
    /**
     * @brief 处理Mavros状态回调
     * @param state Mavros状态指针
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& state);
    
    /**
     * @brief 处理本地位置回调
     * @param pose 本地位置指针
     */
    void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);

    /**
     * @brief 发布设置点
     * @param pose 设置点
     */
    void publishSetpoint(const geometry_msgs::PoseStamped& pose);

    /**
     * @brief 发送设置点前的检查
     * @param duration_seconds 检查持续时间（秒）
     * @param rate_hz 检查频率（Hz）
     * @return true：检查通过，false：检查失败
     */
    bool sendSetpointsBeforeOffboard(double duration_seconds, double rate_hz);

    /**
     * @brief 禁用自动返回和失败安全
     * @return true：成功，false：失败
     */
    bool disableRTLAndFailsafe();

    /**
     * @brief 等待离线模式
     * @param timeout_seconds 超时时间（秒）
     * @return true：成功，false：超时
     */
    bool waitForOffboardMode(double timeout_seconds);

    /**
     * @brief 设置离线模式
     * @param max_retries 最大重试次数（默认3次）
     * @return true：成功，false：失败
     */
    bool setOffboardMode(int max_retries = 3);

    /**
     * @brief 解锁无人机电机
     * @param force 是否强制解锁（默认false）
     * @return true：成功，false：失败
     */
    bool armMotorsWithCheck();

    /**
     * @brief 处理起飞请求
     * @param req 起飞请求
     * @param res 起飞响应
     * @return true：成功，false：失败
     */
    bool takeoffCallback(drone_launch_controller::Takeoff::Request& req,
                         drone_launch_controller::Takeoff::Response& res);

    /**
     * @brief 处理健康检查请求
     * @param req 健康检查请求
     * @param res 健康检查响应
     * @return true：成功，false：失败
     */
    bool healthCheckCallback(drone_launch_controller::DroneHealthCheck::Request& req,
                             drone_launch_controller::DroneHealthCheck::Response& res);

    /**
     * @brief 处理解锁电机请求
     * @param req 解锁电机请求
     * @param res 解锁电机响应
     * @return true：成功，false：失败
     */
    bool armMotorsCallback(drone_launch_controller::ArmMotors::Request& req,
                           drone_launch_controller::ArmMotors::Response& res);

    /**
     * @brief 处理连接状态请求
     * @param req 连接状态请求
     * @param res 连接状态响应
     * @return true：成功，false：失败
     */
    bool connectionStatusCallback(drone_launch_controller::ConnectionStatus::Request& req,
                                   drone_launch_controller::ConnectionStatus::Response& res);

    /**
     * @brief 处理释放控制权请求：停止悬停发布
     */
    bool releaseControlCallback(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res);

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::ServiceServer takeoff_service_;
    ros::ServiceServer health_check_service_;
    ros::ServiceServer arm_service_;
    ros::ServiceServer connection_service_;
    ros::ServiceServer release_control_service_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient param_set_client_;
    ros::Publisher local_pos_pub_;

    TakeoffController takeoff_controller_;
    FlightStatusPublisher status_publisher_;
    HealthChecker health_checker_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    std::mutex pose_mutex_;
    bool state_received_ = false;
    bool pose_received_ = false;
};

}

#endif
