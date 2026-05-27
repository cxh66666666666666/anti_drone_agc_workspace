#include <drone_launch_controller/health_checker.h>

namespace drone_launch_controller {

HealthChecker::HealthChecker(ros::NodeHandle& nh) : nh_(nh) {
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

void HealthChecker::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    battery_state_ = *msg;
}

void HealthChecker::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gps_state_ = *msg;
}

void HealthChecker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_state_ = *msg;
}

void HealthChecker::escCallback(const mavros_msgs::ESCStatus::ConstPtr& msg) {
    esc_state_ = *msg;
}

float HealthChecker::getBatteryVoltage() {
    return battery_state_.voltage;
}

float HealthChecker::getBatteryPercentage() {
    return battery_state_.percentage * 100.0f;
}

bool HealthChecker::checkBattery() {
    if (battery_state_.voltage < 3.3f) {
        ROS_ERROR("Battery voltage too low: %.2fV (threshold: 3.3V)", battery_state_.voltage);
        battery_ok_ = false;
        return false;
    }

    if (battery_state_.percentage < 0.2f) {
        ROS_ERROR("Battery percentage too low: %.1f%% (threshold: 20%%)", battery_state_.percentage * 100.0f);
        battery_ok_ = false;
        return false;
    }

    battery_ok_ = true;
    return true;
}

bool HealthChecker::checkGPS() {
    if (gps_state_.status.status < 0) {
        ROS_ERROR("GPS not fixed - status: %d", gps_state_.status.status);
        gps_ok_ = false;
        return false;
    }

    if (gps_state_.status.service <= 0) {
        ROS_ERROR("GPS service not available - service: %d", gps_state_.status.service);
        gps_ok_ = false;
        return false;
    }

    gps_ok_ = true;
    return true;
}

bool HealthChecker::checkIMU() {
    if (!imu_state_.header.stamp.isZero() &&
        (imu_state_.orientation_covariance[0] < 0 ||
         imu_state_.angular_velocity_covariance[0] < 0 ||
         imu_state_.linear_acceleration_covariance[0] < 0)) {
        ROS_ERROR("IMU data invalid - covariance values indicate no data");
        imu_ok_ = false;
        return false;
    }

    ros::Duration imu_age = ros::Time::now() - imu_state_.header.stamp;
    if (imu_age > ros::Duration(1.0)) {
        ROS_ERROR("IMU data too old: %.2f seconds", imu_age.toSec());
        imu_ok_ = false;
        return false;
    }

    imu_ok_ = true;
    return true;
}

bool HealthChecker::checkBarometer() {
    if (imu_state_.header.stamp.isZero()) {
        ROS_ERROR("Barometer data not available");
        baro_ok_ = false;
        return false;
    }

    baro_ok_ = true;
    return true;
}

bool HealthChecker::checkMagnetometer() {
    if (imu_state_.header.stamp.isZero()) {
        ROS_ERROR("Magnetometer data not available");
        mag_ok_ = false;
        return false;
    }

    mag_ok_ = true;
    return true;
}

bool HealthChecker::checkESCs() {
    if (esc_state_.esc_status.empty()) {
        ROS_WARN("No ESC status data received");
        esc_ok_ = false;
        return false;
    }

    for (const auto& esc : esc_state_.esc_status) {
        if (esc.voltage <= 0.0f) {
            ROS_WARN("ESC has low voltage: %.2f", esc.voltage);
        }
    }

    esc_ok_ = true;
    return true;
}

DroneHealthStatus HealthChecker::performFullCheck() {
    DroneHealthStatus status;
    status.drone_name = "anti_drone";
    status.error_code = 0;

    if (!checkBattery()) {
        status.error_code |= 0x01;
    }
    if (!checkGPS()) {
        status.error_code |= 0x02;
    }
    if (!checkIMU()) {
        status.error_code |= 0x04;
    }
    if (!checkBarometer()) {
        status.error_code |= 0x08;
    }
    if (!checkMagnetometer()) {
        status.error_code |= 0x10;
    }
    if (!checkESCs()) {
        status.error_code |= 0x20;
    }

    status.battery_voltage = getBatteryVoltage();
    status.battery_percentage = getBatteryPercentage();
    status.gps_status = gps_ok_;
    status.imu_status = imu_ok_;
    status.barometer_status = baro_ok_;
    status.magnetometer_status = mag_ok_;
    status.esc_status = esc_ok_;

    if (status.error_code == 0) {
        status.status_message = "All health checks passed";
        ROS_INFO("Full health check passed");
    } else {
        std::string failed_items;
        if (!battery_ok_) failed_items += "Battery ";
        if (!gps_ok_) failed_items += "GPS ";
        if (!imu_ok_) failed_items += "IMU ";
        if (!baro_ok_) failed_items += "Barometer ";
        if (!mag_ok_) failed_items += "Magnetometer ";
        if (!esc_ok_) failed_items += "ESC ";
        status.status_message = "Health check failed: " + failed_items;
        ROS_ERROR("Health check failed: %s", failed_items.c_str());
    }

    return status;
}

}
