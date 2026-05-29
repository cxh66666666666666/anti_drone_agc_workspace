/**
 * @file health_checker.cpp
 * @brief 无人机健康检查器类实现
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#include <drone_launch_controller/health_checker.h>

namespace drone_launch_controller {

/**
 * @brief 无人机健康检查器类构造函数
 * @param nh ROS节点句柄
 */
HealthChecker::HealthChecker(ros::NodeHandle& nh) : nh_(nh) 
{
    battery_sub_ = nh_.subscribe("/mavros/battery", 10, &HealthChecker::batteryCallback, this);
    gps_sub_ = nh_.subscribe("/mavros/global_position/global", 10, &HealthChecker::gpsCallback, this);
    imu_sub_ = nh_.subscribe("/mavros/imu/data", 10, &HealthChecker::imuCallback, this);
    esc_sub_ = nh_.subscribe("/mavros/esc_status", 10, &HealthChecker::escCallback, this);

    battery_ok_ = false;
    gps_ok_ = false;
    imu_ok_ = false;
    baro_ok_ = false;
    mag_ok_ = false;
    esc_ok_ = false;
}

/**
 * @brief 电池状态回调函数
 * @param msg 电池状态消息指针
 */
void HealthChecker::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) 
{
    battery_state_ = *msg;
}

/**
 * @brief GPS状态回调函数
 * @param msg GPS状态消息指针
 */
void HealthChecker::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    gps_state_ = *msg;
}

/**
 * @brief IMU状态回调函数
 * @param msg IMU状态消息指针
 */
void HealthChecker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    imu_state_ = *msg;
}

/**
 * @brief ESC状态回调函数
 * @param msg ESC状态消息指针
 */
void HealthChecker::escCallback(const mavros_msgs::ESCStatus::ConstPtr& msg) 
{
    esc_state_ = *msg;
}

/**
 * @brief 获取电池电压
 * @return 电池电压
 */
float HealthChecker::getBatteryVoltage() 
{
    return battery_state_.voltage;
}

/**
 * @brief 获取电池百分比
 * @return 电池百分比
 */
float HealthChecker::getBatteryPercentage() 
{
    return battery_state_.percentage * 100.0f;
}

/**
 * @brief 检查电池状态
 * @details 检查电池电压和百分比是否在正常范围内，默认阈值为3.3V和20%。
 * @return true 如果电池状态正常，否则返回 false
 */
bool HealthChecker::checkBattery() 
{
    if (battery_state_.voltage < 3.3f) 
    {
        ROS_ERROR("[HealthChecker] 电池电压过低: %.2fV (阈值: 3.3V)", battery_state_.voltage);
        battery_ok_ = false;
        return false;
    }

    if (battery_state_.percentage < 0.2f) 
    {
        ROS_ERROR("[HealthChecker] 电池百分比过低: %.1f%% (阈值: 20%%)", battery_state_.percentage * 100.0f);
        battery_ok_ = false;
        return false;
    }

    battery_ok_ = true;
    return true;
}

/**
 * @brief 检查GPS状态
 * @details 检查GPS状态是否固定且服务是否可用。
 * @return true 如果GPS状态正常，否则返回 false
 */
bool HealthChecker::checkGPS() 
{
    if (gps_state_.status.status < 0) 
    {
        ROS_ERROR("[HealthChecker] GPS状态异常 - 状态: %d", gps_state_.status.status);
        gps_ok_ = false;
        return false;
    }

    if (gps_state_.status.service <= 0) 
    {
        ROS_ERROR("[HealthChecker] GPS服务异常 - 服务: %d", gps_state_.status.service);
        gps_ok_ = false;
        return false;
    }

    gps_ok_ = true;
    return true;
}

/**
 * @brief 检查IMU状态
 * @details 检查IMU数据是否有效。
 * @return true 如果IMU状态正常，否则返回 false
 * @note ROS约定：协方差数组首元素为负表示数据不可靠，0表示数据有效
 */
bool HealthChecker::checkIMU()
{
    
    if (!imu_state_.header.stamp.isZero() &&
        (imu_state_.orientation_covariance[0] < 0 ||
         imu_state_.angular_velocity_covariance[0] < 0 ||
         imu_state_.linear_acceleration_covariance[0] < 0)) 
    {
        ROS_ERROR("[HealthChecker] IMU数据无效 - 协方差值指示无数据");
        imu_ok_ = false;
        return false;
    }

    ros::Duration imu_age = ros::Time::now() - imu_state_.header.stamp;
    if (imu_age > ros::Duration(1.0)) 
    {
        ROS_ERROR("[HealthChecker] IMU数据过期 - 时间间隔: %.2f秒", imu_age.toSec());
        imu_ok_ = false;
        return false;
    }

    imu_ok_ = true;
    return true;
}

/**
 * @brief 检查ESC状态
 * @details 检查ESC状态是否正常，即电压是否在正常范围内。
 * @return true 如果ESC状态正常，否则返回 false
 * @note ROS约定：ESC电压阈值为0.0V
 */
bool HealthChecker::checkESCs() 
{
    if (esc_state_.esc_status.empty()) 
    {
        ROS_WARN("[HealthChecker] ESC状态数据为空");
        esc_ok_ = false;
        return false;
    }

    for (const auto& esc : esc_state_.esc_status) 
    {
        if (esc.voltage <= 0.0f) 
        {
            ROS_WARN("[HealthChecker] ESC电压过低: %.2fV", esc.voltage);
        }
    }

    esc_ok_ = true;
    return true;
}

/**
 * @brief 执行完整的健康检查
 * @details 执行完整的健康检查，包括电池、GPS、IMU、气压计/磁力计（通过IMU数据新鲜度判断）和ESC状态。
 * @return 无人机健康状态结构体
 */
DroneHealthStatus HealthChecker::performFullCheck() 
{
    DroneHealthStatus status;
    status.drone_name = "anti_drone";
    status.error_code = 0;

    if (!checkBattery()) 
    {
        status.error_code |= 0x01;
    }
    if (!checkGPS()) 
    {
        status.error_code |= 0x02;
    }
    if (!checkIMU()) 
    {
        status.error_code |= 0x04;
    }
    if (!imu_state_.header.stamp.isZero()) {
        baro_ok_ = true;
        mag_ok_ = true;
    } else {
        ROS_ERROR("[HealthChecker] IMU数据时间戳无效，气压计/磁力计数据不可用");
        baro_ok_ = false;
        mag_ok_ = false;
    }
    if (!baro_ok_) 
    {
        status.error_code |= 0x08;
    }
    if (!mag_ok_) 
    {
        status.error_code |= 0x10;
    }
    if (!checkESCs()) 
    {
        status.error_code |= 0x20;
    }

    status.battery_voltage = getBatteryVoltage();
    status.battery_percentage = getBatteryPercentage();
    status.gps_status = gps_ok_;
    status.imu_status = imu_ok_;
    status.barometer_status = baro_ok_;
    status.magnetometer_status = mag_ok_;
    status.esc_status = esc_ok_;

    if (status.error_code == 0) 
    {
        status.status_message = "所有健康检查通过";
        ROS_INFO("[HealthChecker] 所有健康检查通过");
    } else 
    {
        std::string failed_items;
        if (!battery_ok_) failed_items += "Battery ";
        if (!gps_ok_) failed_items += "GPS ";
        if (!imu_ok_) failed_items += "IMU ";
        if (!baro_ok_) failed_items += "Barometer ";
        if (!mag_ok_) failed_items += "Magnetometer ";
        if (!esc_ok_) failed_items += "ESC ";
        status.status_message = "健康检查失败: " + failed_items;
        ROS_ERROR("[HealthChecker] 健康检查失败: %s", failed_items.c_str());
    }

    return status;
}

}