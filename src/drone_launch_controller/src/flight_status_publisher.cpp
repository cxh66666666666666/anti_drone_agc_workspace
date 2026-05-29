/**
 * @file flight_status_publisher.cpp
 * @brief 无人机飞行状态发布器类实现
 * @author 陈鑫豪
 * @date 2026-05-27
 */
#include <drone_launch_controller/flight_status_publisher.h>
#include <ros/ros.h>

namespace drone_launch_controller {

/**
 * @brief 构造函数
 * @param nh ROS节点句柄
 * @param takeoff_controller 无人机起飞控制器指针
 */
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

    ROS_INFO("FlightStatusPublisher initialized");
}

/**
 * @brief 析构函数
 */
FlightStatusPublisher::~FlightStatusPublisher() {
    stopPublishing();
}

/**
 * @brief 更新无人机飞行状态
 * @param flight_mode 飞行模式
 * @param current_altitude 当前高度
 * @param target_altitude 目标高度
 * @param latitude 纬度
 * @param longitude 经度
 * @param armed 是否已起飞
 * @param state 飞行状态
 * @param message 状态消息
 */
void FlightStatusPublisher::updateStatus(const std::string& flight_mode, float current_altitude,
                                        float target_altitude, float latitude, float longitude,
                                        uint8_t armed, FlightState state, const std::string& message) {
    flight_mode_ = flight_mode;
    current_altitude_ = current_altitude;
    target_altitude_ = target_altitude;
    armed_ = armed;
    state_ = state;
}

/**
 * @brief 开始发布无人机飞行状态
 * @param rate 发布频率（单位：Hz）
 */
void FlightStatusPublisher::startPublishing(double rate) {
    if (publishing_) {
        ROS_WARN("无人机飞行状态已发布");
        return;
    }

    publishing_ = true;
    double period = 1.0 / rate;
    publish_timer_ = nh_.createTimer(ros::Duration(period), &FlightStatusPublisher::publishTimerCallback, this);
    ROS_INFO("无人机飞行状态发布器开始发布，频率 %.2f Hz", rate);
}

/**
 * @brief 停止发布无人机飞行状态
 */
void FlightStatusPublisher::stopPublishing() {
    if (!publishing_) {
        ROS_WARN("无人机飞行状态未发布，无需停止");
        return;
    }

    publishing_ = false;
    publish_timer_.stop();
    ROS_INFO("无人机飞行状态发布器停止发布");
}

/**
 * @brief 定时器回调函数，用于发布无人机飞行状态
 * @param event ROS定时器事件
 */
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
