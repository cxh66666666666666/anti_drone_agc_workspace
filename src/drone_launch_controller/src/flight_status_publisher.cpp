#include <drone_launch_controller/flight_status_publisher.h>
#include <ros/ros.h>

namespace drone_launch_controller {

FlightStatusPublisher::FlightStatusPublisher(ros::NodeHandle& nh, TakeoffController* takeoff_controller)
    : nh_(nh)
    , takeoff_controller_(takeoff_controller)
    , current_altitude_(0.0f)
    , target_altitude_(0.0f)
    , armed_(0)
    , state_(INIT)
    , publishing_(false)
{
    status_pub_ = nh_.advertise<std_msgs::String>("/drone_launch_controller/flight_status", 10);

    flight_mode_ = "INIT";
    current_altitude_ = 0.0f;
    target_altitude_ = 0.0f;
    armed_ = 0;
    state_ = INIT;

    ROS_INFO("FlightStatusPublisher initialized");
}

FlightStatusPublisher::~FlightStatusPublisher() {
    stopPublishing();
}

void FlightStatusPublisher::updateStatus(const std::string& flight_mode, float current_altitude,
                                        float target_altitude, float latitude, float longitude,
                                        uint8_t armed, FlightState state, const std::string& message) {
    flight_mode_ = flight_mode;
    current_altitude_ = current_altitude;
    target_altitude_ = target_altitude;
    armed_ = armed;
    state_ = state;
}

void FlightStatusPublisher::startPublishing(double rate) {
    if (publishing_) {
        ROS_WARN("FlightStatusPublisher already publishing");
        return;
    }

    publishing_ = true;
    double period = 1.0 / rate;
    publish_timer_ = nh_.createTimer(ros::Duration(period), &FlightStatusPublisher::publishTimerCallback, this);
    ROS_INFO("FlightStatusPublisher started publishing at %.2f Hz", rate);
}

void FlightStatusPublisher::stopPublishing() {
    if (!publishing_) {
        return;
    }

    publishing_ = false;
    publish_timer_.stop();
    ROS_INFO("FlightStatusPublisher stopped publishing");
}

void FlightStatusPublisher::publishTimerCallback(const ros::TimerEvent& event) {
    if (takeoff_controller_ != nullptr) {
        current_altitude_ = takeoff_controller_->getCurrentAltitude();
        state_ = takeoff_controller_->getCurrentState();
    }

    std_msgs::String msg;
    msg.data = "FlightMode: " + flight_mode_ +
                ", Alt: " + std::to_string(current_altitude_) +
                ", Target: " + std::to_string(target_altitude_) +
                ", Armed: " + std::to_string(armed_) +
                ", State: " + std::to_string(state_);
    status_pub_.publish(msg);
}

}
