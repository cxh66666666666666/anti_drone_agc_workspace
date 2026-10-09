/**
 * @file car_launch_controller.h
 * @brief 地面小车启动控制器节点类声明
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#ifndef CAR_LAUNCH_CONTROLLER_H
#define CAR_LAUNCH_CONTROLLER_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <car_launch_controller/health_checker.h>
#include <car_launch_controller/motion_controller.h>
#include <car_launch_controller/car_status_publisher.h>
#include <car_launch_controller/ConnectionStatus.h>
#include <car_launch_controller/CarHealthCheck.h>
#include <car_launch_controller/EnableMotion.h>
#include <car_launch_controller/Drive.h>

namespace car_launch_controller {

class CarLaunchControllerNode {
public:
    /**
     * @brief 构造函数
     */
    CarLaunchControllerNode();

    /**
     * @brief 运行地面小车启动控制器节点
     */
    void run();

private:
    /**
     * @brief 处理连接状态请求
     * @param req 连接状态请求
     * @param res 连接状态响应
     * @return true：成功，false：失败
     */
    bool connectionStatusCallback(car_launch_controller::ConnectionStatus::Request& req,
                                  car_launch_controller::ConnectionStatus::Response& res);

    /**
     * @brief 处理健康检查请求
     * @param req 健康检查请求
     * @param res 健康检查响应
     * @return true：成功，false：失败
     */
    bool healthCheckCallback(car_launch_controller::CarHealthCheck::Request& req,
                             car_launch_controller::CarHealthCheck::Response& res);

    /**
     * @brief 处理使能运动控制请求
     * @param req 使能运动控制请求
     * @param res 使能运动控制响应
     * @return true：成功，false：失败
     */
    bool enableMotionCallback(car_launch_controller::EnableMotion::Request& req,
                              car_launch_controller::EnableMotion::Response& res);

    /**
     * @brief 处理行驶请求
     * @param req 行驶请求
     * @param res 行驶响应
     * @return true：成功，false：失败
     */
    bool driveCallback(car_launch_controller::Drive::Request& req,
                       car_launch_controller::Drive::Response& res);

    /**
     * @brief 处理释放控制权请求
     * @param req 释放控制权请求
     * @param res 释放控制权响应
     * @return true：成功，false：失败
     */
    bool releaseControlCallback(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res);

    ros::NodeHandle nh_;
    ros::ServiceServer connection_service_;
    ros::ServiceServer health_check_service_;
    ros::ServiceServer enable_motion_service_;
    ros::ServiceServer drive_service_;
    ros::ServiceServer release_control_service_;

    // 初始化顺序：health_checker_ -> motion_controller_ -> status_publisher_
    HealthChecker health_checker_;
    MotionController motion_controller_;
    CarStatusPublisher status_publisher_;
};

}

#endif