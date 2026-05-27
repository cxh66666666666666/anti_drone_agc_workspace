#ifndef TAKEOFF_CONTROLLER_H
#define TAKEOFF_CONTROLLER_H

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <mutex>

namespace drone_launch_controller {

enum FlightState {
    INIT = 0,
    CONNECTED = 1,
    HEALTH_CHECK = 2,
    ARMED = 3,
    TAKEOFF = 4,
    HOVER = 5,
    LAND = 6,
    EMERGENCY = 7
};

class TakeoffController {
public:
    explicit TakeoffController(ros::NodeHandle& nh);
    ~TakeoffController();

    bool takeoff(float target_altitude, float timeout);
    bool cancelTakeoff();
    bool pauseTakeoff();
    bool resumeTakeoff();

    float getCurrentAltitude() const;
    bool isInProgress() const;

    bool checkObstacle();
    bool checkGPSLoss();
    bool checkControlTimeout();

    FlightState getCurrentState() const;
    void setState(FlightState state);

private:
    void takeoffLoop();
    void hoverMaintainLoop();
    void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void publishSetpoint(float altitude);
    bool setFlightMode(const std::string& mode);
    void stopTakeoffThread();
    void startHoverMaintainThread();
    void stopHoverMaintainThread();

    ros::NodeHandle& nh_;
    ros::ServiceClient set_mode_client_;
    ros::Publisher local_pos_pub_;
    ros::Subscriber local_pos_sub_;

    geometry_msgs::PoseStamped current_pose_;
    float target_altitude_;
    float takeoff_timeout_;
    bool takeoff_in_progress_;
    FlightState current_state_;
    ros::Time takeoff_start_time_;
    std::thread takeoff_thread_;

    bool paused_;
    mutable std::mutex pose_mutex_;
    mutable std::mutex state_mutex_;

    // Hover maintain thread
    std::thread hover_maintain_thread_;
    bool hover_maintain_running_;
};

}

#endif
