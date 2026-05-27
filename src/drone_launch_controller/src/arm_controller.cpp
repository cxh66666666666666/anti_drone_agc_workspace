#include <drone_launch_controller/arm_controller.h>

namespace drone_launch_controller {

ArmController::ArmController(ros::NodeHandle& nh)
    : nh_(nh)
    , current_armed_state_(false)
    , arm_timeout_seconds_(60.0)
    , max_altitude_limit_(120.0)
    , max_speed_limit_(20.0)
    , geofence_radius_(1000.0)
{
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    state_sub_ = nh_.subscribe("/mavros/state", 10, &ArmController::stateCallback, this);

    arm_cmd_.request.value = false;

    ros::Duration(1.0).sleep();
    ROS_INFO("ArmController initialized");
}

void ArmController::stateCallback(const mavros_msgs::State::ConstPtr& state)
{
    current_armed_state_ = state->armed;
}

bool ArmController::armMotors(bool force)
{
    if (!force) {
        if (!checkGeofence()) {
            ROS_WARN("Cannot arm motors: Drone is outside geofence");
            return false;
        }
        if (!checkAltitudeLimit()) {
            ROS_WARN("Cannot arm motors: Altitude limit exceeded");
            return false;
        }
        if (!checkSpeedLimit()) {
            ROS_WARN("Cannot arm motors: Speed limit exceeded");
            return false;
        }
    }

    arm_cmd_.request.value = true;

    if (!arming_client_.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Arming service not available");
        return false;
    }

    if (arming_client_.call(arm_cmd_)) {
        if (arm_cmd_.response.success) {
            ROS_INFO("Motors armed successfully");
            return true;
        } else {
            ROS_ERROR("Failed to arm motors: result=%d", arm_cmd_.response.result);
            return false;
        }
    } else {
        ROS_ERROR("Arming service call failed");
        return false;
    }
}

bool ArmController::disarmMotors()
{
    arm_cmd_.request.value = false;

    if (!arming_client_.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Disarming service not available");
        return false;
    }

    if (arming_client_.call(arm_cmd_)) {
        if (arm_cmd_.response.success) {
            ROS_INFO("Motors disarmed successfully");
            return true;
        } else {
            ROS_ERROR("Failed to disarm motors: result=%d", arm_cmd_.response.result);
            return false;
        }
    } else {
        ROS_ERROR("Disarming service call failed");
        return false;
    }
}

bool ArmController::isArmed() const
{
    return current_armed_state_;
}

void ArmController::setArmTimeout(double seconds)
{
    arm_timeout_seconds_ = seconds;
    ROS_INFO("Arm timeout set to %.1f seconds", seconds);
}

bool ArmController::checkGeofence() const
{
    ros::NodeHandle nh;
    double current_lat, current_lon;
    double home_lat = 0.0, home_lon = 0.0;

    nh.getParam("/mavros/home_position/home/home_lat", home_lat);
    nh.getParam("/mavros/home_position/home/home_lon", home_lon);

    double distance = std::sqrt(
        std::pow(current_lat - home_lat, 2) +
        std::pow(current_lon - home_lon, 2)
    ) * 111319.9;

    if (distance > geofence_radius_) {
        ROS_WARN("Geofence violation: distance %.2f m exceeds limit %.2f m", distance, geofence_radius_);
        return false;
    }
    return true;
}

bool ArmController::checkAltitudeLimit() const
{
    ros::NodeHandle nh;
    double current_altitude;
    nh.getParam("/mavros/global_position/rel_alt", current_altitude);

    if (current_altitude > max_altitude_limit_) {
        ROS_WARN("Altitude limit exceeded: %.2f m > %.2f m", current_altitude, max_altitude_limit_);
        return false;
    }
    return true;
}

bool ArmController::checkSpeedLimit() const
{
    ros::NodeHandle nh;
    double current_speed;
    nh.getParam("/mavros/global_position/vel/xyz", current_speed);

    if (current_speed > max_speed_limit_) {
        ROS_WARN("Speed limit exceeded: %.2f m/s > %.2f m/s", current_speed, max_speed_limit_);
        return false;
    }
    return true;
}

}
