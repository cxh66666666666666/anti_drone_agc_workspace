#ifndef FLIGHT_STATUS_PUBLISHER_H
#define FLIGHT_STATUS_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <drone_launch_controller/takeoff_controller.h>

namespace drone_launch_controller {

class FlightStatusPublisher {
public:
    explicit FlightStatusPublisher(ros::NodeHandle& nh, TakeoffController* takeoff_controller);
    ~FlightStatusPublisher();

    void updateStatus(const std::string& flight_mode, float current_altitude,
                     float target_altitude, float latitude, float longitude,
                     uint8_t armed, FlightState state, const std::string& message);

    void startPublishing(double rate = 10.0);
    void stopPublishing();

private:
    void publishTimerCallback(const ros::TimerEvent& event);

    ros::NodeHandle& nh_;
    ros::Publisher status_pub_;
    ros::Timer publish_timer_;

    TakeoffController* takeoff_controller_;

    std::string flight_mode_;
    float current_altitude_;
    float target_altitude_;
    uint8_t armed_;
    FlightState state_;
    bool publishing_;
};

}

#endif
