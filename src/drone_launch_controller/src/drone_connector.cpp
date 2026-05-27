#include <drone_launch_controller/drone_connector.h>

namespace drone_launch_controller {

DroneConnector::DroneConnector(ros::NodeHandle& nh, double connection_timeout)
    : nh_(nh)
    , connected_(false)
    , system_id_(0)
    , connection_timeout_(connection_timeout)
    , heartbeat_running_(false)
{
    current_state_.connected = false;
    current_state_.armed = false;
    current_state_.guided = false;
    current_state_.mode = "";

    last_heartbeat_time_ = ros::Time::now();

    state_sub_ = nh_.subscribe("/mavros/state", 10, &DroneConnector::stateCallback, this);

    ROS_INFO("[DroneConnector] Initialized, waiting for connection...");
}

DroneConnector::~DroneConnector()
{
    disconnect();
    stopHeartbeat();
}

bool DroneConnector::connect()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (connected_) {
        ROS_WARN("[DroneConnector] Already connected");
        return true;
    }

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);

    ROS_INFO("[DroneConnector] Waiting for FCU connection, timeout: %.1f seconds", connection_timeout_.toSec());

    while (ros::ok()) {
        if (connected_) {
            ROS_INFO("[DroneConnector] FCU connected, system ID: %d", system_id_);
            if (connection_callback_) {
                connection_callback_(true);
            }
            return true;
        }

        if ((ros::Time::now() - start_time) > connection_timeout_) {
            ROS_ERROR("[DroneConnector] Connection timeout");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return false;
}

void DroneConnector::disconnect()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!connected_) {
        return;
    }

    connected_ = false;
    ROS_INFO("[DroneConnector] Disconnected from FCU");

    if (connection_callback_) {
        connection_callback_(false);
    }
}

bool DroneConnector::isConnected() const
{
    return connected_;
}

uint8_t DroneConnector::getSystemId() const
{
    return system_id_;
}

void DroneConnector::startHeartbeat()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (heartbeat_running_) {
            ROS_WARN("[DroneConnector] Heartbeat monitor already running");
            return;
        }

        heartbeat_running_ = true;
    }

    heartbeat_thread_ = std::thread(&DroneConnector::heartbeatLoop, this);

    ROS_INFO("[DroneConnector] Heartbeat monitor thread started");
}

void DroneConnector::stopHeartbeat()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!heartbeat_running_) {
            return;
        }

        heartbeat_running_ = false;
    }

    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }

    ROS_INFO("[DroneConnector] Heartbeat monitor thread stopped");
}

void DroneConnector::registerConnectionCallback(std::function<void(bool)> callback)
{
    connection_callback_ = callback;
}

void DroneConnector::stateCallback(const mavros_msgs::State::ConstPtr& state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    bool was_connected = connected_;

    current_state_ = *state;
    connected_ = state->connected;
    system_id_ = 1;

    last_heartbeat_time_ = ros::Time::now();

    if (!was_connected && connected_) {
        ROS_INFO("[DroneConnector] Connection established, system ID: %d", system_id_);
    } else if (was_connected && !connected_) {
        ROS_WARN("[DroneConnector] Connection lost");
        if (connection_callback_) {
            connection_callback_(false);
        }
    }
}

void DroneConnector::heartbeatLoop()
{
    ROS_INFO("[DroneConnector] Heartbeat monitor loop started");

    ros::Rate rate(1);

    while (heartbeat_running_ && ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            ros::Time now = ros::Time::now();
            ros::Duration time_since_heartbeat = now - last_heartbeat_time_;

            if (connected_ && time_since_heartbeat > connection_timeout_) {
                ROS_WARN("[DroneConnector] Heartbeat timeout (%.1f seconds)", time_since_heartbeat.toSec());
            }
        }

        rate.sleep();
    }

    ROS_INFO("[DroneConnector] Heartbeat monitor loop ended");
}

}
