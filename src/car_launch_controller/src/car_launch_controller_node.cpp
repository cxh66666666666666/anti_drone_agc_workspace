/**
 * @file car_launch_controller_node.cpp
 * @brief 地面小车启动控制器节点实现
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#include <clocale>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <car_launch_controller/car_launch_controller.h>

namespace car_launch_controller {

/**
 * @brief 地面小车启动控制器节点构造函数
 */
CarLaunchControllerNode::CarLaunchControllerNode()
    : nh_("~")
    , health_checker_(nh_)
    , motion_controller_(nh_, &health_checker_)
    , status_publisher_(nh_, &motion_controller_, &health_checker_)
{
    connection_service_ = nh_.advertiseService(
        "/car_launch_controller/connection_status",
        &CarLaunchControllerNode::connectionStatusCallback, this);
    health_check_service_ = nh_.advertiseService(
        "/car_launch_controller/car_health_check",
        &CarLaunchControllerNode::healthCheckCallback, this);
    enable_motion_service_ = nh_.advertiseService(
        "/car_launch_controller/enable_motion",
        &CarLaunchControllerNode::enableMotionCallback, this);
    drive_service_ = nh_.advertiseService(
        "/car_launch_controller/drive",
        &CarLaunchControllerNode::driveCallback, this);
    release_control_service_ = nh_.advertiseService(
        "/car_launch_controller/release_control",
        &CarLaunchControllerNode::releaseControlCallback, this);

    ROS_INFO("[CarLaunchControllerNode] 初始化完成");
}

/**
 * @brief 地面小车启动控制器节点运行函数
 */
void CarLaunchControllerNode::run()
{
    status_publisher_.startPublishing(10.0);
    ros::spin();
}

/**
 * @brief 处理连接状态服务回调
 * @param req 连接状态请求
 * @param res 连接状态响应
 * @return true：成功，false：失败
 */
bool CarLaunchControllerNode::connectionStatusCallback(car_launch_controller::ConnectionStatus::Request& req,
                                                       car_launch_controller::ConnectionStatus::Response& res)
{
    (void)req;
    CarHealthStatus h = health_checker_.performFullCheck();
    res.connected = h.model_present && h.pose_fresh;
    res.system_status = static_cast<uint8_t>(h.error_code);
    if (res.connected)
    {
        res.message = "已连接 - 车模: ground_car, 控制权: " + motion_controller_.getControlMode();
    }
    else
    {
        res.message = "未连接";
    }
    ROS_INFO("[Connection] 状态: %s", res.message.c_str());
    return true;
}

/**
 * @brief 处理健康检查服务回调
 * @param req 健康检查请求
 * @param res 健康检查响应
 * @return true：成功，false：失败
 */
bool CarLaunchControllerNode::healthCheckCallback(car_launch_controller::CarHealthCheck::Request& req,
                                                  car_launch_controller::CarHealthCheck::Response& res)
{
    (void)req;
    ROS_INFO("[Health Check] 健康检查服务被调用");

    CarHealthStatus h = health_checker_.performFullCheck();
    res.success = (h.error_code == 0);
    res.result = static_cast<uint8_t>(h.error_code);
    res.health_status = h;
    res.message = h.status_message;

    ROS_INFO("[Health Check] 结果: %s (error_code: %d)",
             res.success ? "PASS" : "FAIL", h.error_code);
    return true;
}

/**
 * @brief 处理使能运动控制服务回调
 * @param req 使能运动控制请求
 * @param res 使能运动控制响应
 * @return true：成功，false：失败
 */
bool CarLaunchControllerNode::enableMotionCallback(car_launch_controller::EnableMotion::Request& req,
                                                   car_launch_controller::EnableMotion::Response& res)
{
    (void)req;
    ROS_INFO("[EnableMotion] 使能运动控制服务被调用");

    res.success = false;
    res.result = 0;

    if (!health_checker_.isModelPresent())
    {
        ROS_ERROR("[EnableMotion] Gazebo未连接，无法使能运动控制");
        res.message = "Gazebo未连接";
        return true;
    }

    if (motion_controller_.enableMotion())
    {
        ROS_INFO("[EnableMotion] 运动控制已使能");
        res.success = true;
        res.result = 1;
        res.message = "运动控制已使能";
    }
    else
    {
        ROS_ERROR("[EnableMotion] 车模位姿不可用，使能失败");
        res.message = "车模位姿不可用";
    }
    return true;
}

/**
 * @brief 处理行驶服务回调
 * @param req 行驶请求
 * @param res 行驶响应
 * @return true：成功，false：失败
 */
bool CarLaunchControllerNode::driveCallback(car_launch_controller::Drive::Request& req,
                                            car_launch_controller::Drive::Response& res)
{
    ROS_INFO("========================================");
    ROS_INFO("[Drive] 行驶服务被调用");

    res.success = false;
    res.result = 0;
    res.achieved_distance = 0.0f;

    if (!motion_controller_.isMotionEnabled())
    {
        ROS_ERROR("[Drive] 未使能运动控制，无法行驶");
        res.message = "未使能运动控制";
        return true;
    }

    float achieved = 0.0f;
    bool drive_success = motion_controller_.drive(req.distance, req.speed, req.timeout, achieved);
    res.achieved_distance = achieved;

    if (drive_success)
    {
        ROS_INFO("[Drive] 行驶完成，实际距离：%.2f米", achieved);
        res.success = true;
        res.result = 1;
        res.message = "行驶完成";
    }
    else
    {
        ROS_ERROR("[Drive] 行驶超时，实际距离：%.2f米", achieved);
        res.message = "行驶超时";
    }

    ROS_INFO("========================================");
    return true;
}

/**
 * @brief 处理释放控制权服务回调
 * @param req 释放控制权请求
 * @param res 释放控制权响应
 * @return true：成功，false：失败
 */
bool CarLaunchControllerNode::releaseControlCallback(std_srvs::Trigger::Request& req,
                                                     std_srvs::Trigger::Response& res)
{
    (void)req;
    ROS_INFO("[ReleaseControl] 收到释放控制权请求");
    motion_controller_.releaseControl();
    res.success = true;
    res.message = "控制权已释放";
    return true;
}

}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "car_launch_controller");
    ROS_INFO("========================================");
    ROS_INFO("[CarLaunchControllerNode] 地面小车启动控制器节点已启动");
    ROS_INFO("========================================");

    car_launch_controller::CarLaunchControllerNode node;
    node.run();

    return 0;
}