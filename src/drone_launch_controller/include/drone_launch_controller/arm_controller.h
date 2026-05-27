#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Trigger.h>

namespace drone_launch_controller {

class ArmController {
public:
    explicit ArmController(ros::NodeHandle& nh);

    bool armMotors(bool force = false);
    bool disarmMotors();
    bool isArmed() const;

    void setArmTimeout(double seconds);

    bool checkGeofence() const;
    bool checkAltitudeLimit() const;
    bool checkSpeedLimit() const;

private:
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
