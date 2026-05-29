/**
 * @file drone_connector.h
 * @brief 无人机连接类
 * @details 该类负责无人机的连接与断开，包括心跳检测与回调注册。
 * @author 陈鑫豪
 * @date 2026-05-27
 * @note 该类依赖于Mavros消息类型和ROS服务客户端。
 */
#ifndef DRONE_CONNECTOR_H
#define DRONE_CONNECTOR_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <thread>
#include <mutex>
#include <functional>

namespace drone_launch_controller {

/**
 * @brief DroneConnector类
 * @details 该类负责无人机的连接与断开，包括心跳检测与回调注册。
 */
class DroneConnector {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param connection_timeout 连接超时时间（秒）
     */
    DroneConnector(ros::NodeHandle& nh, double connection_timeout = 30.0);
    /**
     * @brief 析构函数
     */
    ~DroneConnector();

    /**
     * @brief 连接无人机
     * @return true 如果连接成功，否则返回false
     */
    bool connect();

    /**
     * @brief 断开无人机连接
     * @return true 如果断开成功，否则返回false（超时或ROS节点关闭）
     */
    bool disconnect();

    /**
     * @brief 检查无人机是否已连接
     * @return true 如果已连接，否则返回false
     */
    bool isConnected() const;

    /**
     * @brief 获取无人机系统ID
     * @return 无人机系统ID
     */
    uint8_t getSystemId() const;

    /**
     * @brief 启动心跳检测线程
     */
    void startHeartbeat();

    /**
     * @brief 停止心跳检测线程
     */
    void stopHeartbeat();
    
    /**
     * @brief 注册连接状态回调函数
     * @param callback 连接状态回调函数指针
     */
    void registerConnectionCallback(std::function<void(bool)> callback);

private:
    /**
     * @brief 状态回调函数
     * @param state Mavros状态消息指针
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& state);
    
    /**
     * @brief 心跳检测循环
     */
    void heartbeatLoop();

    ros::NodeHandle nh_;
    mavros_msgs::State current_state_;
    ros::Subscriber state_sub_;

    bool connected_;
    uint8_t system_id_;
    ros::Time last_heartbeat_time_;
    ros::Duration connection_timeout_;

    std::function<void(bool)> connection_callback_;

    bool heartbeat_running_;
    std::mutex state_mutex_;
    std::thread heartbeat_thread_;
};

}
#endif
