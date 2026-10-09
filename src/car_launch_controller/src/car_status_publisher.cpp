/**
 * @file car_status_publisher.cpp
 * @brief 地面小车状态发布器类实现
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#include <car_launch_controller/car_status_publisher.h>

#include <cmath>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace car_launch_controller {

/**
 * @brief 构造函数
 * @param nh ROS节点句柄
 * @param motion_controller 运动控制器指针
 * @param health_checker 健康检查器指针
 */
CarStatusPublisher::CarStatusPublisher(ros::NodeHandle& nh, MotionController* motion_controller,
                                       HealthChecker* health_checker)
    : nh_(nh)
    , motion_controller_(motion_controller)
    , health_checker_(health_checker)
    , publishing_(false)
{
    status_pub_ = nh_.advertise<car_launch_controller::CarStatus>("/car_launch_controller/car_status", 10);

    ROS_INFO("[CarStatusPublisher] 初始化完成");
}

/**
 * @brief 析构函数
 */
CarStatusPublisher::~CarStatusPublisher()
{
    stopPublishing();
}

/**
 * @brief 开始发布小车状态
 * @param rate 发布频率（单位：Hz）
 */
void CarStatusPublisher::startPublishing(double rate)
{
    if (publishing_)
    {
        ROS_WARN("[CarStatusPublisher] 已发布");
        return;
    }

    publishing_ = true;
    double period = 1.0 / rate;
    publish_timer_ = nh_.createTimer(ros::Duration(period), &CarStatusPublisher::publishTimerCallback, this);
    ROS_INFO("[CarStatusPublisher] 开始发布，频率 %.2f Hz", rate);
}

/**
 * @brief 停止发布小车状态
 */
void CarStatusPublisher::stopPublishing()
{
    if (!publishing_)
    {
        ROS_WARN("[CarStatusPublisher] 未发布，无需停止");
        return;
    }

    publishing_ = false;
    publish_timer_.stop();
    ROS_INFO("[CarStatusPublisher] 停止发布");
}

/**
 * @brief 定时器回调函数，用于发布小车状态
 * @param event ROS定时器事件
 */
void CarStatusPublisher::publishTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    car_launch_controller::CarStatus msg;
    msg.control_mode = motion_controller_->getControlMode();
    msg.state = static_cast<uint8_t>(motion_controller_->getCurrentState());
    msg.motion_enabled = motion_controller_->isMotionEnabled() ? 1 : 0;
    msg.linear_speed = motion_controller_->getLinearSpeed();

    geometry_msgs::Pose pose;
    if (health_checker_->getCurrentPose(pose))
    {
        msg.position_x = pose.position.x;
        msg.position_y = pose.position.y;
        // 四元数转 yaw
        const double qx = pose.orientation.x;
        const double qy = pose.orientation.y;
        const double qz = pose.orientation.z;
        const double qw = pose.orientation.w;
        msg.yaw = static_cast<float>(std::atan2(2.0 * (qw * qz + qx * qy),
                                                1.0 - 2.0 * (qy * qy + qz * qz)));
    }
    else
    {
        msg.position_x = 0.0f;
        msg.position_y = 0.0f;
        msg.yaw = 0.0f;
    }

    status_pub_.publish(msg);
}

}