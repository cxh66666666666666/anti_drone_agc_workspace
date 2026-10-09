/**
 * @file car_status_publisher.h
 * @brief 地面小车状态发布器类
 * @details 定时发布小车控制模式、运动状态、位置与速度。
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#ifndef CAR_STATUS_PUBLISHER_H
#define CAR_STATUS_PUBLISHER_H

#include <ros/ros.h>
#include <car_launch_controller/CarStatus.h>
#include <car_launch_controller/motion_controller.h>
#include <car_launch_controller/health_checker.h>

namespace car_launch_controller {

class CarStatusPublisher {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param motion_controller 运动控制器指针
     * @param health_checker 健康检查器指针（用于读取位姿）
     */
    CarStatusPublisher(ros::NodeHandle& nh, MotionController* motion_controller,
                       HealthChecker* health_checker);

    /**
     * @brief 析构函数
     */
    ~CarStatusPublisher();

    /**
     * @brief 开始发布小车状态
     * @param rate 发布频率（单位：Hz）
     */
    void startPublishing(double rate = 10.0);

    /**
     * @brief 停止发布小车状态
     */
    void stopPublishing();

private:
    /**
     * @brief 定时器回调函数，用于发布小车状态
     * @param event ROS定时器事件
     */
    void publishTimerCallback(const ros::TimerEvent& event);

    ros::NodeHandle& nh_;
    ros::Publisher status_pub_;
    ros::Timer publish_timer_;

    MotionController* motion_controller_;
    HealthChecker* health_checker_;

    bool publishing_;
};

}

#endif