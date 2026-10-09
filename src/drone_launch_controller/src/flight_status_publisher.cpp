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
 * @param state_ptr MAVROS状态指针
 */
FlightStatusPublisher::FlightStatusPublisher(ros::NodeHandle& nh, TakeoffController* takeoff_controller,
                                           const mavros_msgs::State* state_ptr)
    : nh_(nh)
    , takeoff_controller_(takeoff_controller)
    , state_ptr_(state_ptr)
    , publishing_(false)
{
    status_pub_ = nh_.advertise<drone_launch_controller::FlightStatus>("/drone_launch_controller/flight_status", 10);

    ROS_INFO("[FlightStatusPublisher] 初始化完成");
}

/**
 * @brief 析构函数
 */
FlightStatusPublisher::~FlightStatusPublisher() {
    stopPublishing();
}

/**
 * @brief 开始发布无人机飞行状态
 * @param rate 发布频率（单位：Hz）
 */
void FlightStatusPublisher::startPublishing(double rate) {
    if (publishing_) {
        ROS_WARN("[FlightStatusPublisher] 已发布");
        return;
    }

    publishing_ = true;
    double period = 1.0 / rate;
    publish_timer_ = nh_.createTimer(ros::Duration(period), &FlightStatusPublisher::publishTimerCallback, this);
    ROS_INFO("[FlightStatusPublisher] 开始发布，频率 %.2f Hz", rate);
}

/**
 * @brief 停止发布无人机飞行状态
 */
void FlightStatusPublisher::stopPublishing() {
    if (!publishing_) {
        ROS_WARN("[FlightStatusPublisher] 未发布，无需停止");
        return;
    }

    publishing_ = false;
    publish_timer_.stop();
    ROS_INFO("[FlightStatusPublisher] 停止发布");
}

/**
 * @brief 定时器回调函数，用于发布无人机飞行状态
 * @param event ROS定时器事件
 */
void FlightStatusPublisher::publishTimerCallback(const ros::TimerEvent& event) 
{
    drone_launch_controller::FlightStatus msg;
    msg.flight_mode = state_ptr_->mode;
    msg.armed = state_ptr_->armed ? 1 : 0;
    msg.current_altitude = takeoff_controller_->getCurrentAltitude();
    msg.state = static_cast<uint8_t>(takeoff_controller_->getCurrentState());
    status_pub_.publish(msg);
}

}
