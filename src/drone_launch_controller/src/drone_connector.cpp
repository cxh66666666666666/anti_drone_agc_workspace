/**
 * @file drone_connector.cpp
 * @brief 无人机连接类实现
 * @author 陈鑫豪
 * @date 2026-05-27
 */
#include <drone_launch_controller/drone_connector.h>

namespace drone_launch_controller {

/**
 * @brief 构造函数
 * @param nh ROS节点句柄
 * @param connection_timeout 连接超时时间（秒）
 */
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

    ROS_INFO("【DroneConnector】等待无人机连接...");
}

/**
 * @brief 析构函数
 */
DroneConnector::~DroneConnector()
{
    disconnect();
    stopHeartbeat();
}

/**
 * @brief 连接无人机
 * @return true 如果连接成功
 * @return false 如果连接超时
 */
bool DroneConnector::connect()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (connected_) 
    {
        ROS_WARN("【DroneConnector】已连接");
        return true;
    }

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);

    ROS_INFO("【DroneConnector】等待无人机连接，超时时间：%.1f秒", connection_timeout_.toSec());

    while (ros::ok()) 
    {
        if (connected_) 
        {
            ROS_INFO("【DroneConnector】无人机连接成功，系统ID：%d", system_id_);
            if (connection_callback_) {
                connection_callback_(true);
            }
            return true;
        }

        if ((ros::Time::now() - start_time) > connection_timeout_) 
        {
            ROS_ERROR("【DroneConnector】连接超时");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return false;
}

/**
 * @brief 断开与无人机的连接
 * @return true 如果断开成功
 * @return false 如果断开超时或ROS节点关闭
 */
bool DroneConnector::disconnect()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!connected_) 
        {
            ROS_WARN("【DroneConnector】未连接，无需断开");
            return true;
        }

        connected_ = false;
        ROS_INFO("【DroneConnector】开始断开连接...");
    }

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);
    int wait_count = 0;

    while (ros::ok()) 
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!connected_) 
            {
                ROS_INFO("【DroneConnector】连接已断开");
                if (connection_callback_) 
                {
                    connection_callback_(false);
                }
                return true;
            }
        }

        if ((ros::Time::now() - start_time) > connection_timeout_) 
        {
            ROS_ERROR("【DroneConnector】断开连接超时");
            if (connection_callback_) {
                connection_callback_(false);
            }
            return false;
        }

        if (wait_count % 10 == 0) 
        {
            ROS_INFO("【DroneConnector】等待断开连接...");
        }
        wait_count++;
        ros::spinOnce();
        rate.sleep();
    }

    if (!ros::ok()) 
    {
        ROS_ERROR("【DroneConnector】ROS节点已关闭");
        if (connection_callback_)
        {
            connection_callback_(false);
        }
        return false;
    }
}

/**
 * @brief 检查是否已连接
 * @return true 如果已连接
 * @return false 如果未连接
 */
bool DroneConnector::isConnected() const
{
    return connected_;
}

/**
 * @brief 获取无人机系统ID
 * @return uint8_t 无人机系统ID
 */
uint8_t DroneConnector::getSystemId() const
{
    return system_id_;
}

/**
 * @brief 启动心跳检测线程
 */
void DroneConnector::startHeartbeat()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (heartbeat_running_) 
        {
            ROS_WARN("【DroneConnector】心跳检测线程已运行");
            return;
        }

        heartbeat_running_ = true;
    }

    heartbeat_thread_ = std::thread(&DroneConnector::heartbeatLoop, this);
    ROS_INFO("【DroneConnector】心跳监控线程已启动");
}

/**
 * @brief 停止心跳检测线程
 */
void DroneConnector::stopHeartbeat()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!heartbeat_running_) 
        {
            ROS_WARN("【DroneConnector】心跳检测线程未运行，无需停止");
            return;
        }

        heartbeat_running_ = false;
    }

    if (heartbeat_thread_.joinable()) 
    {
        heartbeat_thread_.join();
    }

    ROS_INFO("【DroneConnector】心跳监控线程已停止");
}

/**
 * @brief 注册连接状态回调函数
 * @param callback 连接状态回调函数
 */
void DroneConnector::registerConnectionCallback(std::function<void(bool)> callback)
{
    connection_callback_ = callback;
}

/**
 * @brief 处理无人机状态回调
 * @param state 无人机状态指针
 */
void DroneConnector::stateCallback(const mavros_msgs::State::ConstPtr& state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    bool was_connected = connected_;

    current_state_ = *state;
    connected_ = state->connected;
    system_id_ = 1;

    last_heartbeat_time_ = ros::Time::now();

    if (!was_connected && connected_) 
    {
        ROS_INFO("【DroneConnector】连接已建立，系统ID：%d", system_id_);
    }
    else if (was_connected && !connected_) 
    {
        ROS_WARN("【DroneConnector】连接已丢失");
        if (connection_callback_) 
        {
            connection_callback_(false);
        }
    }
}

/**
 * @brief 心跳检测线程循环
 */
void DroneConnector::heartbeatLoop()
{
    ROS_INFO("【DroneConnector】心跳监控循环已启动");

    ros::Rate rate(1);

    while (heartbeat_running_ && ros::ok()) 
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            ros::Time now = ros::Time::now();
            ros::Duration time_since_heartbeat = now - last_heartbeat_time_;

            if (connected_ && time_since_heartbeat > connection_timeout_) 
            {
                ROS_WARN("【DroneConnector】心跳超时（%.1f秒）", time_since_heartbeat.toSec());
                connected_ = false;
                if (connection_callback_) 
                {
                    connection_callback_(false);
                }
            }
        }

        rate.sleep();
    }
    ROS_INFO("【DroneConnector】心跳监控循环已结束");
}

}
