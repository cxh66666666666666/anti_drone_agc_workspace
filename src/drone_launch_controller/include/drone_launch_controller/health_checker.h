/**
 * @file health_checker.h
 * @brief 无人机健康检查器类
 * @details 该类负责检查无人机的健康状态，包括检查安全检查（地理围栏、高度限制、速度限制等）。
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#ifndef HEALTH_CHECKER_H
#define HEALTH_CHECKER_H

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/ESCStatus.h>
#include <mavros_msgs/CommandBool.h>
#include <drone_launch_controller/DroneHealthStatus.h>

namespace drone_launch_controller {

class HealthChecker {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit HealthChecker(ros::NodeHandle& nh);

    /**
     * @brief 检查电池状态
     * @return true 如果电池状态正常，否则返回false
     */
    bool checkBattery();

    /**
     * @brief 检查GPS状态
     * @return true 如果GPS状态正常，否则返回false
     */
    bool checkGPS();
    
    /**
     * @brief 检查IMU状态
     * @return true 如果IMU状态正常，否则返回false
     */
    bool checkIMU();

    /**
     * @brief 检查ESC状态
     * @return true 如果ESC状态正常，否则返回false
     */
    bool checkESCs();
    
    /**
     * @brief 执行完整的健康检查
     * @return DroneHealthStatus 无人机健康状态结构体
     */
    DroneHealthStatus performFullCheck();

    /**
     * @brief 获取电池电压
     * @return float 电池电压值（单位：伏特）
     */
    float getBatteryVoltage();
    
    /**
     * @brief 获取电池百分比
     * @return float 电池百分比值（范围：0-100）
     */
    float getBatteryPercentage();

private:
    /**
     * @brief 电池状态回调函数
     * @param msg 电池状态消息指针
     */
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    
    /**
     * @brief GPS状态回调函数
     * @param msg GPS状态消息指针
     */
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    
    /**
     * @brief IMU状态回调函数
     * @param msg IMU状态消息指针
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    /**
     * @brief ESC状态回调函数
     * @param msg ESC状态消息指针
     */
    void escCallback(const mavros_msgs::ESCStatus::ConstPtr& msg);

    ros::NodeHandle& nh_;
    ros::Subscriber battery_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber esc_sub_;

    sensor_msgs::BatteryState battery_state_;
    sensor_msgs::NavSatFix gps_state_;
    sensor_msgs::Imu imu_state_;
    mavros_msgs::ESCStatus esc_state_;

    bool battery_ok_;
    bool gps_ok_;
    bool imu_ok_;
    bool esc_ok_;
};

}

#endif
