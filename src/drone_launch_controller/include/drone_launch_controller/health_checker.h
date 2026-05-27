#ifndef HEALTH_CHECKER_H
#define HEALTH_CHECKER_H

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/ESCStatus.h>
#include <mavros_msgs/CommandBool.h>

namespace drone_launch_controller {

struct DroneHealthStatus {
    std::string drone_name;
    float battery_voltage;
    float battery_percentage;
    bool gps_status;
    bool imu_status;
    bool barometer_status;
    bool magnetometer_status;
    bool esc_status;
    std::string status_message;
    uint16_t error_code;
};

class HealthChecker {
public:
    explicit HealthChecker(ros::NodeHandle& nh);

    bool checkBattery();
    bool checkGPS();
    bool checkIMU();
    bool checkBarometer();
    bool checkMagnetometer();
    bool checkESCs();
    DroneHealthStatus performFullCheck();

    float getBatteryVoltage();
    float getBatteryPercentage();

private:
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
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
    bool baro_ok_;
    bool mag_ok_;
    bool esc_ok_;
};

}

#endif
