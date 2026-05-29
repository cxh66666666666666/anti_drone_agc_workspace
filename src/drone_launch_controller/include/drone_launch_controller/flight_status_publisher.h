/**
 * @file flight_status_publisher.h
 * @brief 无人机飞行状态发布器类
 * @details 该类负责发布无人机的飞行状态消息。
 * @author 陈鑫豪
 * @date 2026-05-27
 * @note 该类依赖于Mavros消息类型和ROS服务客户端。
 */
#ifndef FLIGHT_STATUS_PUBLISHER_H
#define FLIGHT_STATUS_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <drone_launch_controller/takeoff_controller.h>

namespace drone_launch_controller {

class FlightStatusPublisher {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param takeoff_controller 无人机起飞控制器指针
     */
    explicit FlightStatusPublisher(ros::NodeHandle& nh, TakeoffController* takeoff_controller);
    /**
     * @brief 析构函数
     */
    ~FlightStatusPublisher();

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
    void updateStatus(const std::string& flight_mode, float current_altitude,
                     float target_altitude, float latitude, float longitude,
                     uint8_t armed, FlightState state, const std::string& message);
    
    /**
     * @brief 开始发布无人机飞行状态
     * @param rate 发布频率（单位：Hz）
     */
    void startPublishing(double rate = 10.0);

    /**
     * @brief 停止发布无人机飞行状态
     */ 
    void stopPublishing();

private:
    /**
     * @brief 定时器回调函数，用于发布无人机飞行状态
     * @param event ROS定时器事件
     */
    void publishTimerCallback(const ros::TimerEvent& event);

    ros::NodeHandle& nh_;
    ros::Publisher status_pub_;
    ros::Timer publish_timer_;

    TakeoffController* takeoff_controller_;

    std::string flight_mode_;
    float current_altitude_;
    float target_altitude_;
    uint8_t armed_;
    FlightState state_;
    bool publishing_;
};

}

#endif
