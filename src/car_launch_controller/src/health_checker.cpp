/**
 * @file health_checker.cpp
 * @brief 地面小车健康检查器类实现
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#include <car_launch_controller/health_checker.h>

namespace car_launch_controller {

/**
 * @brief 构造函数
 * @param nh ROS节点句柄
 */
HealthChecker::HealthChecker(ros::NodeHandle& nh)
    : nh_(nh)
    , pose_received_(false)
    , sectors_received_(false)
    , model_present_(false)
    , model_ok_(false)
    , pose_ok_(false)
    , perception_ok_(false)
    , control_service_ok_(false)
{
    link_states_sub_ = nh_.subscribe("/ground_gazebo/link_states", 10,
                                     &HealthChecker::linkStatesCallback, this);
    sectors_sub_ = nh_.subscribe("/ground_obstacle_sectors", 10,
                                 &HealthChecker::sectorsCallback, this);
}

/**
 * @brief link_states 回调，提取车模位姿
 * @param msg 链接状态消息指针
 */
void HealthChecker::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "ground_car::base_link")
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = msg->pose[i];
            pose_received_ = true;
            // 小车栈 use_sim_time=false，用墙上时间做新鲜度判断
            pose_stamp_ = ros::Time::now();
            model_present_ = true;
            return;
        }
    }
}

/**
 * @brief 感知扇区回调，记录最后收到时间
 * @param msg 扇区距离消息指针
 */
void HealthChecker::sectorsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    sectors_received_ = !msg->data.empty();
    sectors_stamp_ = ros::Time::now();
}

/**
 * @brief 车模是否存在
 * @return true 表示 link_states 中出现过 ground_car::base_link
 */
bool HealthChecker::isModelPresent() const
{
    return model_present_;
}

/**
 * @brief 获取最近一次收到的车模位姿（world 系）
 * @param pose 输出位姿
 * @return true 表示已收到，false 表示尚无数据
 */
bool HealthChecker::getCurrentPose(geometry_msgs::Pose& pose) const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!pose_received_)
    {
        return false;
    }
    pose = current_pose_;
    return true;
}

/**
 * @brief 检查车模是否存在
 * @return true 表示存在
 */
bool HealthChecker::checkModelPresent()
{
    if (!model_present_)
    {
        ROS_ERROR("[HealthChecker] 未检测到车模: ground_car::base_link");
        model_ok_ = false;
        return false;
    }
    model_ok_ = true;
    return true;
}

/**
 * @brief 检查车模位姿数据是否新鲜
 * @return true 表示新鲜
 */
bool HealthChecker::checkPoseFresh()
{
    bool received = false;
    ros::Time stamp;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        received = pose_received_;
        stamp = pose_stamp_;
    }

    if (!received)
    {
        ROS_ERROR("[HealthChecker] 未收到车模位姿数据");
        pose_ok_ = false;
        return false;
    }

    ros::Duration age = ros::Time::now() - stamp;
    if (age > ros::Duration(1.0))
    {
        ROS_ERROR("[HealthChecker] 车模位姿数据过期 - 时间间隔: %.2f秒", age.toSec());
        pose_ok_ = false;
        return false;
    }

    pose_ok_ = true;
    return true;
}

/**
 * @brief 检查感知数据是否正常
 * @return true 表示正常
 */
bool HealthChecker::checkPerception()
{
    if (sectors_stamp_.isZero())
    {
        ROS_ERROR("[HealthChecker] 从未收到感知数据: /ground_obstacle_sectors");
        perception_ok_ = false;
        return false;
    }

    if (!sectors_received_)
    {
        ROS_ERROR("[HealthChecker] 感知数据为空");
        perception_ok_ = false;
        return false;
    }

    ros::Duration age = ros::Time::now() - sectors_stamp_;
    if (age > ros::Duration(1.0))
    {
        ROS_ERROR("[HealthChecker] 感知数据过期 - 时间间隔: %.2f秒", age.toSec());
        perception_ok_ = false;
        return false;
    }

    perception_ok_ = true;
    return true;
}

/**
 * @brief 检查控制服务是否可用
 * @return true 表示可用
 * @note 使用非阻塞 exists 查询，避免 waitForService 阻塞服务回调
 */
bool HealthChecker::checkControlService()
{
    if (!ros::service::exists("/ground_gazebo/set_model_state", false))
    {
        ROS_ERROR("[HealthChecker] 控制服务不可用: /ground_gazebo/set_model_state");
        control_service_ok_ = false;
        return false;
    }
    control_service_ok_ = true;
    return true;
}

/**
 * @brief 执行完整的健康检查
 * @return 车况结构体
 */
CarHealthStatus HealthChecker::performFullCheck()
{
    CarHealthStatus status;
    status.car_name = "ground_car";
    status.error_code = 0;

    if (!checkModelPresent())
    {
        status.error_code |= 0x01;
    }
    if (!checkPoseFresh())
    {
        status.error_code |= 0x02;
    }
    if (!checkPerception())
    {
        status.error_code |= 0x04;
    }
    if (!checkControlService())
    {
        status.error_code |= 0x08;
    }

    status.model_present = model_ok_;
    status.pose_fresh = pose_ok_;
    status.perception_fresh = perception_ok_;
    status.control_service_available = control_service_ok_;

    if (status.error_code == 0)
    {
        status.status_message = "所有健康检查通过";
        ROS_INFO("[HealthChecker] 所有健康检查通过");
    }
    else
    {
        std::string failed_items;
        if (!model_ok_) failed_items += "Model ";
        if (!pose_ok_) failed_items += "Pose ";
        if (!perception_ok_) failed_items += "Perception ";
        if (!control_service_ok_) failed_items += "ControlService ";
        status.status_message = "健康检查失败: " + failed_items;
        ROS_ERROR("[HealthChecker] 健康检查失败: %s", failed_items.c_str());
    }

    return status;
}

}