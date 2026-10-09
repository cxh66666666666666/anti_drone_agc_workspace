/**
 * @file health_checker.h
 * @brief 地面小车健康检查器类
 * @details 负责检查车模存在性、位姿新鲜度、感知数据新鲜度和控制服务可用性。
 * @author 陈鑫豪
 * @date 2026-09-19
 */
#ifndef HEALTH_CHECKER_H
#define HEALTH_CHECKER_H

#include <mutex>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <car_launch_controller/CarHealthStatus.h>

namespace car_launch_controller {

class HealthChecker {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit HealthChecker(ros::NodeHandle& nh);

    /**
     * @brief 执行完整的健康检查
     * @return 车况结构体（含位掩码 error_code）
     */
    CarHealthStatus performFullCheck();

    /**
     * @brief 车模是否存在
     * @return true 表示 link_states 中出现过 ground_car::base_link
     */
    bool isModelPresent() const;

    /**
     * @brief 获取最近一次收到的车模位姿（world 系）
     * @param pose 输出位姿
     * @return true 表示已收到，false 表示尚无数据
     */
    bool getCurrentPose(geometry_msgs::Pose& pose) const;

private:
    /**
     * @brief link_states 回调，提取车模位姿
     * @param msg 链接状态消息指针
     */
    void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);

    /**
     * @brief 感知扇区回调，记录最后收到时间
     * @param msg 扇区距离消息指针
     */
    void sectorsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    bool checkModelPresent();
    bool checkPoseFresh();
    bool checkPerception();
    bool checkControlService();

    ros::NodeHandle& nh_;
    ros::Subscriber link_states_sub_;
    ros::Subscriber sectors_sub_;

    mutable std::mutex pose_mutex_;
    geometry_msgs::Pose current_pose_;
    bool pose_received_;
    ros::Time pose_stamp_;

    bool sectors_received_;
    ros::Time sectors_stamp_;

    bool model_present_;
    bool model_ok_;
    bool pose_ok_;
    bool perception_ok_;
    bool control_service_ok_;
};

}

#endif