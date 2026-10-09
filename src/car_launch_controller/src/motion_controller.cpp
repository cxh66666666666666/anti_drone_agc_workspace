/**
 * @file motion_controller.cpp
 * @brief 地面小车运动控制器类实现
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#include <car_launch_controller/motion_controller.h>

#include <algorithm>

namespace car_launch_controller {

// 参数缺省值
static constexpr float kDefaultReferenceSpeed = 1.0f;
static constexpr double kDefaultControlRate = 20.0;
static constexpr float kDefaultTimeout = 30.0f;

/**
 * @brief 构造函数
 * @param nh ROS节点句柄
 * @param health_checker 健康检查器指针
 */
MotionController::MotionController(ros::NodeHandle& nh, HealthChecker* health_checker)
    : nh_(nh)
    , health_checker_(health_checker)
    , current_state_(INIT)
    , linear_speed_(0.0f)
    , model_name_("ground_car")
    , control_rate_(kDefaultControlRate)
{
    // 读取私有参数（读不到则用默认值）
    std::string model_name;
    if (ros::param::get("~model_name", model_name))
    {
        model_name_ = model_name;
    }
    double control_rate = 0.0;
    if (ros::param::get("~control_rate", control_rate) && control_rate > 0.0)
    {
        control_rate_ = control_rate;
    }

    set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/ground_gazebo/set_model_state");

    ROS_INFO("[MotionController] 初始化完成");
}

/**
 * @brief 析构函数
 */
MotionController::~MotionController()
{
    hold_timer_.stop();
    ROS_INFO("[MotionController] 销毁完成");
}

/**
 * @brief 使能运动控制（接管车模）
 * @return true 表示使能成功
 */
bool MotionController::enableMotion()
{
    if (isMotionEnabled())
    {
        ROS_WARN("[MotionController] 运动控制已使能");
        return true;
    }

    if (!health_checker_->isModelPresent())
    {
        ROS_ERROR("[MotionController] 车模不存在，无法使能运动控制");
        return false;
    }

    geometry_msgs::Pose pose;
    if (!health_checker_->getCurrentPose(pose))
    {
        ROS_ERROR("[MotionController] 无法读取车模位姿，使能失败");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        initial_pose_ = pose;
        hold_pose_ = pose;
    }

    setState(ENABLED);

    // 启动保持定时器：仅在 HOLDING 时写保持位姿
    hold_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                  &MotionController::holdTimerCallback, this);

    ROS_INFO("[MotionController] 运动控制已使能，初始位姿 x=%.2f y=%.2f",
             pose.position.x, pose.position.y);
    return true;
}

/**
 * @brief 沿 world +x 行驶指定距离
 * @param distance 目标距离（米）
 * @param speed 线速度（米/秒）
 * @param timeout 超时（秒）
 * @param achieved_distance 输出实际行驶距离
 * @return true 表示到达目标距离
 */
bool MotionController::drive(float distance, float speed, float timeout, float& achieved_distance)
{
    achieved_distance = 0.0f;

    CarState state = getCurrentState();
    if (state != ENABLED && state != HOLDING)
    {
        ROS_ERROR("[MotionController] 未使能运动控制，无法驱动");
        return false;
    }

    if (distance <= 0.0f)
    {
        ROS_ERROR("[MotionController] 目标距离无效: %.2f", distance);
        return false;
    }

    if (speed <= 0.0f)
    {
        double param_speed = 0.0;
        if (ros::param::get("~reference_speed", param_speed))
        {
            speed = static_cast<float>(param_speed);
        }
        else
        {
            speed = kDefaultReferenceSpeed;
        }
    }
    if (timeout <= 0.0f)
    {
        timeout = kDefaultTimeout;
    }

    geometry_msgs::Pose start_pose;
    if (!health_checker_->getCurrentPose(start_pose))
    {
        ROS_ERROR("[MotionController] 无法读取车模位姿，驱动失败");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        linear_speed_ = speed;
    }
    setState(DRIVING);

    ROS_INFO("[MotionController] 开始行驶: distance=%.2f m, speed=%.2f m/s, timeout=%.1f s",
             distance, speed, timeout);

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(control_rate_);

    float traveled = 0.0f;
    bool reached = false;
    geometry_msgs::Pose target_pose = start_pose;

    while (ros::ok())
    {
        double elapsed = (ros::Time::now() - start_time).toSec();
        if (elapsed > timeout)
        {
            ROS_WARN("[MotionController] 行驶超时，停在已行进位置");
            break;
        }

        // 目标位姿 = 起点沿 world +x 平移 min(speed*elapsed, distance)
        traveled = std::min(static_cast<float>(speed * elapsed), distance);
        target_pose = start_pose;
        target_pose.position.x = start_pose.position.x + traveled;
        // y、z、姿态保持起点不变
        writeModelState(target_pose);

        if (traveled >= distance)
        {
            reached = true;
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hold_pose_ = target_pose;
        linear_speed_ = 0.0f;
    }
    setState(HOLDING);

    achieved_distance = reached ? distance : traveled;
    if (reached)
    {
        ROS_INFO("[MotionController] 已到达目标距离: %.2f m", achieved_distance);
    }

    return reached;
}

/**
 * @brief 释放控制权（可重复调用）
 */
void MotionController::releaseControl()
{
    hold_timer_.stop();

    bool already_released = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        already_released = (current_state_ == CONNECTED);
        if (!already_released)
        {
            linear_speed_ = 0.0f;
        }
    }

    if (already_released)
    {
        ROS_INFO("[MotionController] 控制权已释放，无需重复释放");
        return;
    }

    setState(CONNECTED);
    ROS_INFO("[MotionController] 控制权已释放");
}

/**
 * @brief 获取当前状态
 * @return 当前 CarState
 */
CarState MotionController::getCurrentState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

/**
 * @brief 是否已使能（ENABLED 及之后）
 * @return true 表示已使能
 */
bool MotionController::isMotionEnabled() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_ >= ENABLED;
}

/**
 * @brief 获取当前线速度，非 DRIVING 时为 0
 * @return 线速度（米/秒）
 */
float MotionController::getLinearSpeed() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_state_ != DRIVING)
    {
        return 0.0f;
    }
    return linear_speed_;
}

/**
 * @brief 获取控制模式
 * @return "NONE" 或 "CONTROLLER"
 */
std::string MotionController::getControlMode() const
{
    return isMotionEnabled() ? "CONTROLLER" : "NONE";
}

/**
 * @brief 设置当前状态并记录变化
 * @param state 新状态
 */
void MotionController::setState(CarState state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_state_ != state)
    {
        ROS_INFO("[MotionController] 状态变化: %d -> %d",
                 static_cast<int>(current_state_), static_cast<int>(state));
        current_state_ = state;
    }
}

/**
 * @brief 写入车模位姿到 set_model_state
 * @param pose 目标位姿（world 系）
 */
void MotionController::writeModelState(const geometry_msgs::Pose& pose)
{
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = model_name_;
    srv.request.model_state.reference_frame = "world";
    srv.request.model_state.pose = pose;

    if (!set_model_state_client_.call(srv))
    {
        ROS_WARN("[MotionController] set_model_state 调用失败");
    }
}

/**
 * @brief 保持定时器回调，仅 HOLDING 时写保持位姿
 * @param event ROS定时器事件
 */
void MotionController::holdTimerCallback(const ros::TimerEvent& event)
{
    (void)event;

    if (getCurrentState() != HOLDING)
    {
        return;
    }

    geometry_msgs::Pose pose;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pose = hold_pose_;
    }
    writeModelState(pose);
}

}