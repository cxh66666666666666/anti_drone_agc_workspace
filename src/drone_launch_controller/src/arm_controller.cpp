/**
 * @file arm_controller.cpp
 * @brief 无人机电机控制类函数实现
 * @author 陈鑫豪
 * @date 2026-05-27
 */
#include <drone_launch_controller/arm_controller.h>

namespace drone_launch_controller {

/**
 * @details 初始化无人机电机控制类, 超时时间60秒，最大高度120米，最大速度20米/秒，地理围栏半径1000米
 * @param nh：ROS句柄引用
 */
ArmController::ArmController(ros::NodeHandle& nh)
    : nh_(nh)
    , current_armed_state_(false)
    , arm_timeout_seconds_(60.0)
    , max_altitude_limit_(120.0)
    , max_speed_limit_(20.0)
    , geofence_radius_(1000.0)
{
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    state_sub_ = nh_.subscribe("/mavros/state", 10, &ArmController::stateCallback, this);

    arm_cmd_.request.value = false;

    ros::Duration(1.0).sleep();
    ROS_INFO("ArmController 初始化完成");
}

/**
 * @details 解锁无人机电机
 * @param force：是否强制解锁（默认false）
 */
bool ArmController::armMotors(bool force)
{
    if (!force) 
    {
        if (!checkGeofence()) 
        {
            ROS_WARN("无人机超出地理围栏范围");
            return false;
        }
        if (!checkAltitudeLimit()) 
        {
            ROS_WARN("高度限制超出");
            return false;
        }
        if (!checkSpeedLimit()) 
        {
            ROS_WARN("速度限制超出");
            return false;
        }
    }

    arm_cmd_.request.value = true;

    if (!arming_client_.waitForExistence(ros::Duration(5.0))) 
    {
        ROS_ERROR("解锁无人机电机服务不可用");
        return false;
    }

    if (arming_client_.call(arm_cmd_)) 
    {
        if (arm_cmd_.response.success) 
        {
            ROS_INFO("无人机电机解锁成功");
            return true;
        }
        else 
        {
            ROS_ERROR("解锁无人机电机失败: result=%d", arm_cmd_.response.result);
            return false;
        }
    } 
    else 
    {
        ROS_ERROR("解锁无人机电机服务调用失败");
        return false;
    }
}

/**
 * @details 上锁无人机电机
 */
bool ArmController::disarmMotors()
{
    arm_cmd_.request.value = false;

    if (!arming_client_.waitForExistence(ros::Duration(5.0))) 
    {
        ROS_ERROR("上锁无人机电机服务不可用");
        return false;
    }

    if (arming_client_.call(arm_cmd_)) 
    {
        if (arm_cmd_.response.success) 
        {
            ROS_INFO("无人机电机上锁成功");
            return true;
        } 
        else 
        {
            ROS_ERROR("上锁无人机电机失败: result=%d", arm_cmd_.response.result);
            return false;
        }
    } 
    else 
    {
        ROS_ERROR("上锁无人机电机服务调用失败");
        return false;
    }
}

/**
 * @details 检查无人机电机是否已上锁
 * @return true：已上锁，false：未上锁
 */
bool ArmController::isArmed() const
{
    return current_armed_state_;
}

/**
 * @details 设置无人机电机解锁超时时间
 * @param seconds：超时时间（秒）
 */
void ArmController::setArmTimeout(double seconds)
{
    arm_timeout_seconds_ = seconds;
    ROS_INFO("电机解锁超时时间设置为 %.1f 秒", seconds);
}

/**
 * @details 检查无人机是否超出地理围栏范围，distance = √[(Δlat)² + (Δlon)²] × 111319.9
 * @return true：超出范围，false：未超出范围
 */
bool ArmController::checkGeofence() const
{
    ros::NodeHandle nh(nh_);
    double current_lat = 0.0, current_lon = 0.0;
    double home_lat = 0.0, home_lon = 0.0;
    
    nh.getParam("/mavros/global_position/global/lat", current_lat);
    nh.getParam("/mavros/global_position/global/lon", current_lon);
    nh.getParam("/mavros/home_position/home/home_lat", home_lat);
    nh.getParam("/mavros/home_position/home/home_lon", home_lon);

    double distance = std::sqrt(
        std::pow(current_lat - home_lat, 2) +
        std::pow(current_lon - home_lon, 2)
    ) * 111319.9;

    if (distance > geofence_radius_) 
    {
        ROS_WARN("无人机超出地理围栏范围:距离为 %.2f m，超出 %.2f m 限制", distance, geofence_radius_);
        return false;
    }
    return true;
}

/**
 * @details 检查无人机高度超出限制
 * @return true：超出限制，false：未超出限制
 */
bool ArmController::checkAltitudeLimit() const
{
    ros::NodeHandle nh(nh_);
    double current_altitude;
    nh.getParam("/mavros/global_position/rel_alt", current_altitude);

    if (current_altitude > max_altitude_limit_) 
    {
        ROS_WARN("高度限制超出: %.2f m > %.2f m", current_altitude, max_altitude_limit_);
        return false;
    }
    return true;
}

/**
 * @details 检查无人机速度超出限制
 * @return true：超出限制，false：未超出限制
 */
bool ArmController::checkSpeedLimit() const
{
    ros::NodeHandle nh(nh_);
    double current_speed;
    nh.getParam("/mavros/global_position/vel/xyz", current_speed);

    if (current_speed > max_speed_limit_) 
    {
        ROS_WARN("速度限制超出: %.2f m/s > %.2f m/s", current_speed, max_speed_limit_);
        return false;
    }
    return true;
}

/**
 * @details 状态回调函数
 * @param state：Mavros状态消息指针
 */
void ArmController::stateCallback(const mavros_msgs::State::ConstPtr& state)
{
    current_armed_state_ = state->armed;
}
}