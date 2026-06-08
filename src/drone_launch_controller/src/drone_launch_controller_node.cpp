/**
 * @file drone_launch_controller_node.cpp
 * @brief 无人机启动控制器节点实现
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#include <drone_launch_controller/drone_launch_controller.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>

namespace drone_launch_controller {

/**
 * @brief 无人机启动控制器节点构造函数
 */
DroneLaunchControllerNode::DroneLaunchControllerNode()
    : nh_("~")
    , takeoff_controller_(&current_pose_, local_pos_pub_, pose_mutex_)
    , status_publisher_(nh_, &takeoff_controller_, &current_state_)
    , health_checker_(nh_)
{
    state_sub_ = nh_.subscribe("/mavros/state", 10, &DroneLaunchControllerNode::stateCallback, this);
    local_pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &DroneLaunchControllerNode::localPosCallback, this);
    takeoff_service_ = nh_.advertiseService("/drone_launch_controller/takeoff", &DroneLaunchControllerNode::takeoffCallback, this);
    health_check_service_ = nh_.advertiseService("/drone_launch_controller/drone_health_check", &DroneLaunchControllerNode::healthCheckCallback, this);
    arm_service_ = nh_.advertiseService("/drone_launch_controller/arm_motors", &DroneLaunchControllerNode::armMotorsCallback, this);
    connection_service_ = nh_.advertiseService("/drone_launch_controller/connection_status", &DroneLaunchControllerNode::connectionStatusCallback, this);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    param_set_client_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    ROS_INFO("[DroneLaunchControllerNode] 初始化完成");
}

/**
 * @brief 无人机启动控制器节点运行函数
 */
void DroneLaunchControllerNode::run() 
{
    status_publisher_.startPublishing(10.0);
    ros::spin();
}

/**
 * @brief 处理无人机状态回调
 * @param state 无人机状态指针
 */
void DroneLaunchControllerNode::stateCallback(const mavros_msgs::State::ConstPtr& state) 
{
    current_state_ = *state;
    state_received_ = true;
}

/**
 * @brief 处理无人机本地位置回调
 * @param pose 无人机本地位置指针
 */
void DroneLaunchControllerNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) 
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = *pose;
    pose_received_ = true;
}

/**
 * @brief 发布无人机目标位置
 * @param pose 无人机目标位置
 */
void DroneLaunchControllerNode::publishSetpoint(const geometry_msgs::PoseStamped& pose) 
{
    geometry_msgs::PoseStamped setpoint = pose;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    local_pos_pub_.publish(setpoint);
}

/**
 * @brief 发送无人机目标位置前的预热
 * @param duration_seconds 预热持续时间（秒）
 * @param rate_hz 预热频率（Hz）
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::sendSetpointsBeforeOffboard(double duration_seconds, double rate_hz) 
{
    ROS_INFO("[Setpoint Pre-send] 开始发送目标位置预热，持续热时间%.1f秒，频率%.1fHz", duration_seconds, rate_hz);
    
    if (!pose_received_) 
    {
        ROS_ERROR("[Setpoint Pre-send] 未收到本地位置，无法发送预热目标位置");
        return false;
    }

    ros::Rate rate(rate_hz);
    int num_setpoints = static_cast<int>(duration_seconds * rate_hz);
    
    for (int i = 0; i < num_setpoints && ros::ok(); ++i) 
    {
        publishSetpoint(current_pose_);
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("[Setpoint Pre-send] 目标位置预热预热完成");
    return true;
}

/**
 * @brief 禁用无人机的RTL和RC失败安全
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::disableRTLAndFailsafe() 
{
    ROS_INFO("[PX4 Config] 开始禁用RTL失败安全和RC失败安全...");
    
    mavros_msgs::ParamSet param_set;
    param_set.request.param_id = "NAV_RCL_ACT";
    param_set.request.value.integer = 0;
    param_set.request.value.real = 0.0;
    
    if (param_set_client_.call(param_set)) 
    {
        if (param_set.response.success) 
        {
            ROS_INFO("[PX4 Config] RTL失败安全已禁用");
        } 
        else 
        {
            ROS_WARN("[PX4 Config] 失败禁用RTL失败安全");
        }
    } 
    else 
    {
        ROS_WARN("[PX4 Config] 调用param/set服务失败");
    }
    
    param_set.request.param_id = "NAV_DLL_ACT";
    param_set.request.value.integer = 0;
    param_set.request.value.real = 0.0;
    
    if (param_set_client_.call(param_set)) 
    {
        if (param_set.response.success) 
        {
            ROS_INFO("[PX4 Config] RC失败安全已禁用");
        } 
        else 
        {
            ROS_WARN("[PX4 Config] 失败禁用RC失败安全");
        }
    }
    
    param_set.request.param_id = "COM_RCL_EXCEPT";
    param_set.request.value.integer = 4;
    param_set.request.value.real = 4.0;
    
    if (param_set_client_.call(param_set)) 
    {
        if (param_set.response.success) 
        {
            ROS_INFO("[PX4 Config] 允许在无RC下起飞");
        } 
        else 
        {
            ROS_WARN("[PX4 Config] 失败设置允许在无RC下起飞");
        }
    }
    
    ros::Duration(0.5).sleep();
    return true;
}

/**
 * @brief 等待无人机进入OFFBOARD模式
 * @param timeout_seconds 超时时间（秒）
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::waitForOffboardMode(double timeout_seconds) 
{
    ROS_INFO("[Offboard Mode] 开始等待OFFBOARD模式确认...");
    ros::Time start = ros::Time::now();
    ros::Rate rate(10);
    
    while (ros::ok()) 
    {
        ros::spinOnce();
        
        if (current_state_.mode == "OFFBOARD") 
        {
            ROS_INFO("[Offboard Mode] 已确认进入OFFBOARD模式");
            return true;
        }
        
        ros::Duration elapsed = ros::Time::now() - start;
        if (elapsed.toSec() > timeout_seconds) 
        {
            ROS_ERROR("[Offboard Mode] 等待OFFBOARD模式超时，当前模式：%s", 
                      current_state_.mode.c_str());
            return false;
        }
        
        rate.sleep();
    }
    return false;
}

/**
 * @brief 设置无人机进入OFFBOARD模式
 * @param max_retries 最大重试次数
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::setOffboardMode(int max_retries) 
{
    ROS_INFO("[Offboard Mode] 开始设置OFFBOARD模式...");
    
    for (int attempt = 1; attempt <= max_retries; ++attempt) 
    {
        ROS_INFO("[Offboard Mode] 第%d/%d次尝试", attempt, max_retries);
        
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = "OFFBOARD";
        
        if (set_mode_client_.call(set_mode_srv)) 
        {
            if (set_mode_srv.response.mode_sent) 
            {
                ROS_INFO("[Offboard Mode] 模式切换请求已发送，等待确认...");
                if (waitForOffboardMode(3.0)) 
                {
                    ros::Duration(0.5).sleep();
                    return true;
                }
                ROS_WARN("[Offboard Mode] 模式切换请求已发送，但未改变模式");
            } 
            else 
            {
                ROS_WARN("[Offboard Mode] 模式切换请求已被拒绝（第%d次尝试）", attempt);
            }
        } 
        else 
        {
            ROS_ERROR("[Offboard Mode] 第%d次尝试调用set_mode服务失败", attempt);
        }
        
        if (attempt < max_retries) 
        {
            ROS_INFO("[Offboard Mode] 1秒后重试...");
            ros::Duration(1.0).sleep();
        }
    }
    
    ROS_ERROR("[Offboard Mode] 设置OFFBOARD模式失败");
    return false;
}

/**
 * @brief 检查并启动无人机电机
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::armMotorsWithCheck() {
    ROS_INFO("[Arming] 检查并启动电机前的条件，等待确认电机已启动...");
    
    if (!state_received_ || !current_state_.connected) 
    {
        ROS_ERROR("[Arming] MAVROS未连接，无法启动电机。");
        return false;
    }
    
    if (current_state_.mode != "OFFBOARD") 
    {
        ROS_ERROR("[Arming] 当前模式：%s，不是OFFBOARD模式，无法启动电机。", current_state_.mode.c_str());
        return false;
    }
    
    if (!pose_received_) 
    {
        ROS_ERROR("[Arming] 未收到本地位置信息，无法启动电机。");
        return false;
    }
    
    ROS_INFO("[Arming] 执行启动电机命令...");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if (arming_client_.call(arm_cmd)) 
    {
        if (arm_cmd.response.success) 
        {
            ROS_INFO("[Arming] 电机已成功启动");
            ros::Duration(0.5).sleep();
            return true;
        } 
        else 
        {
            ROS_ERROR("[Arming] 启动电机失败，结果码：%d", arm_cmd.response.result);
            return false;
        }
    } 
    else 
    {
        ROS_ERROR("[Arming] 调用arming服务失败");
        return false;
    }
}

/**
 * @brief 处理起飞服务回调
 * @param req 服务请求
 * @param res 服务响应
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::takeoffCallback(drone_launch_controller::Takeoff::Request& req,
                     drone_launch_controller::Takeoff::Response& res) 
{
    ROS_INFO("========================================");
    ROS_INFO("[Takeoff] 起飞服务被调用");

    res.success = false;
    res.result = 0;
    res.achieved_altitude = 0.0f;

    if (!state_received_ || !current_state_.connected) 
    {
        ROS_ERROR("[Takeoff] MAVROS未连接，无法起飞。");
        res.message = "MAVROS未连接";
        return true;
    }

    ROS_INFO("[Takeoff] MAVROS 已连接，当前模式：%s", current_state_.mode.c_str());
    ROS_INFO("[Takeoff] 是否已启动电机：%s", current_state_.armed ? "Yes" : "No");

    disableRTLAndFailsafe();

    if (!sendSetpointsBeforeOffboard(2.0, 20.0)) 
    {
        ROS_ERROR("[Takeoff] 发送预设点失败，无法起飞。");
        res.message = "发送预设点失败";
        return true;
    }

    if (current_state_.mode != "OFFBOARD") 
    {
        if (!setOffboardMode(3)) 
        {
            ROS_ERROR("[Takeoff] 设置OFFBOARD模式失败，无法起飞。");
            res.message = "设置OFFBOARD模式失败";
            return true;
        }
    } 
    else 
    {
        ROS_INFO("[Takeoff] 已进入OFFBOARD模式");
    }

    if (!current_state_.armed) 
    {
        if (!armMotorsWithCheck()) 
        {
            ROS_ERROR("[Takeoff] 启动电机失败，无法起飞。");
            res.message = "启动电机失败";
            return true;
        }
    } 
    else 
    {
        ROS_INFO("[Takeoff] 已启动电机");
    }

    float target_altitude = req.altitude > 0.0f ? req.altitude : 2.0f;
    float timeout = req.timeout > 0.0f ? req.timeout : 30.0f;

    ROS_INFO("[Takeoff] 开始起飞到高度：%.2f米，超时时间：%.1f秒", target_altitude, timeout);
    bool takeoff_success = takeoff_controller_.takeoff(target_altitude, timeout);

    if (takeoff_success) 
    {
        ros::Rate rate(10);
        int timeout_count = 0;
        int max_timeout = static_cast<int>(timeout * 10);

        while (ros::ok() && takeoff_controller_.isInProgress() && timeout_count < max_timeout) 
        {
            ros::spinOnce();
            rate.sleep();
            timeout_count++;
        }

        FlightState state = takeoff_controller_.getCurrentState();
        float final_altitude = takeoff_controller_.getCurrentAltitude();
        if (state == HOVER) 
        {
            ROS_INFO("[Takeoff] 成功起飞到高度：%.2f米", final_altitude);
            res.success = true;
            res.result = 1;
            res.achieved_altitude = final_altitude;
            res.message = "起飞成功";
        } 
        else if (state == EMERGENCY) 
        {
            ROS_ERROR("[Takeoff] 紧急状态检测到");
            res.message = "紧急状态检测到";
            res.achieved_altitude = final_altitude;
        } 
        else 
        {
            res.message = "未知状态: " + std::to_string(state);
            res.achieved_altitude = final_altitude;
        }
    } 
    else 
    {
        ROS_ERROR("[Takeoff] 起飞失败，未成功起飞到目标高度");
        res.message = "起飞失败";
    }

    ROS_INFO("========================================");
    return true;
}

/**
 * @brief 处理健康检查服务回调
 * @param req 服务请求
 * @param res 服务响应
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::healthCheckCallback(drone_launch_controller::DroneHealthCheck::Request& req,
                         drone_launch_controller::DroneHealthCheck::Response& res) 
{
    ROS_INFO("[Health Check] 健康检查服务被调用");

    DroneHealthStatus health = health_checker_.performFullCheck();
    res.success = (health.error_code == 0);
    res.result = health.error_code;
    res.health_status = health;
    res.message = health.status_message;

    ROS_INFO("[Health Check] 结果: %s (error_code: %d)", 
             res.success ? "PASS" : "FAIL", health.error_code);
    return true;
}

/**
 * @brief 处理解锁电机服务回调
 * @param req 服务请求
 * @param res 服务响应
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::armMotorsCallback(drone_launch_controller::ArmMotors::Request& req,
                       drone_launch_controller::ArmMotors::Response& res) 
{
    ROS_INFO("[Arm Service] 启动电机服务被调用, (force: %s)", req.force ? "true" : "false");

    res.success = false;
    res.result = 0;

    if (!state_received_ || !current_state_.connected) 
    {
        ROS_ERROR("[Arm Service] MAVROS未连接，无法启动电机。");
        res.message = "MAVROS未连接";
        return true;
    }

    disableRTLAndFailsafe();

    if (!sendSetpointsBeforeOffboard(2.0, 20.0)) 
    {
        ROS_ERROR("[Arm Service] 发送预设点失败，无法启动电机。");
        res.message = "发送预设点失败";
        return true;
    }

    if (current_state_.mode != "OFFBOARD") 
    {
        if (!setOffboardMode(3)) 
        {
            ROS_ERROR("[Arm Service] 设置OFFBOARD模式失败，无法启动电机。");
            res.message = "设置OFFBOARD模式失败";
            return true;
        }
    }

    if (armMotorsWithCheck()) 
    {
        ROS_INFO("[Arm Service] 启动电机成功");
        res.success = true;
        res.result = 1;
        res.message = "启动电机成功";
    } 
    else 
    {
        ROS_ERROR("[Arm Service] 启动电机失败");
        res.message = "启动电机失败";
    }

    return true;
}

/**
 * @brief 处理连接状态服务回调
 * @param req 服务请求
 * @param res 服务响应
 * @return true：成功，false：失败
 */
bool DroneLaunchControllerNode::connectionStatusCallback(drone_launch_controller::ConnectionStatus::Request& req,
                               drone_launch_controller::ConnectionStatus::Response& res) 
{
    res.connected = current_state_.connected;
    res.system_status = current_state_.system_status;
    if (current_state_.connected) 
    {
        res.message = "已连接 -模式: " + current_state_.mode +
                      ", 解锁: " + (current_state_.armed ? "是" : "否");
    } 
    else 
    {
        res.message = "未连接";
    }
    ROS_INFO("[Connection] 状态: %s", res.message.c_str());
    return true;
}

}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "drone_launch_controller");
    ROS_INFO("========================================");
    ROS_INFO("[DroneLaunchControllerNode] 无人机起飞控制器节点已启动");
    ROS_INFO("========================================");

    drone_launch_controller::DroneLaunchControllerNode node;
    node.run();

    return 0;
}
