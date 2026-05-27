#ifndef DRONE_CONNECTOR_H
#define DRONE_CONNECTOR_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <thread>
#include <mutex>
#include <functional>

namespace drone_launch_controller {

class DroneConnector {
public:
    DroneConnector(ros::NodeHandle& nh, double connection_timeout = 30.0);
    ~DroneConnector();

    bool connect();
    void disconnect();
    bool isConnected() const;
    uint8_t getSystemId() const;

    void startHeartbeat();
    void stopHeartbeat();

    void registerConnectionCallback(std::function<void(bool)> callback);

private:
    void stateCallback(const mavros_msgs::State::ConstPtr& state);
    void heartbeatLoop();

    ros::NodeHandle nh_;
    mavros_msgs::State current_state_;
    ros::Subscriber state_sub_;

    bool connected_;
    uint8_t system_id_;
    ros::Time last_heartbeat_time_;
    ros::Duration connection_timeout_;

    std::function<void(bool)> connection_callback_;

    bool heartbeat_running_;
    std::mutex state_mutex_;
    std::thread heartbeat_thread_;
};

}
#endif
