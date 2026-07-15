/**
 * @file lqr_tracker_node.h
 * @brief LQR轨迹跟踪节点类
 * @author 陈鑫豪
 * @date 2026-06-04
 */

#ifndef LQR_TRAJECTORY_TRACKER_LQR_TRACKER_NODE_H
#define LQR_TRAJECTORY_TRACKER_LQR_TRACKER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/State.h>

#include "lqr_trajectory_tracker/lqr_controller.h"
#include "lqr_trajectory_tracker/trajectory_generator.h"

namespace lqr_trajectory_tracker {

struct TrackingError {
    double position_error;
    double velocity_error;
    Eigen::Vector3d position_error_vec;
    Eigen::Vector3d velocity_error_vec;
};

class LQRTrackerNode {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param pnh 私有节点句柄
     */
    LQRTrackerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /**
     * @brief 析构函数
     */
    ~LQRTrackerNode();

    /**
     * @brief 启动跟踪控制循环
     */
    void run();

private:
    /**
     * @brief 回调函数：处理无人机位姿和状态订阅消息
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief 回调函数：处理无人机状态订阅消息
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    /**
     * @brief 回调函数：处理强化学习局部避障偏移量
     */
    void localOffsetCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    /**
     * @brief 发布函数：发布轨迹可视化数据
     */
    void publishTrajectoryVisualization();

    /**
     * @brief 发布函数：发布位置设定点数据
     */
    void publishPositionSetpoint(const Eigen::Vector3d& position);

    /**
     * @brief 发布函数：发布跟踪误差数据
     */
    void publishTrackingError(const TrackingError& error);

    /**
     * @brief 记录函数：记录跟踪统计信息
     */
    void logTrackingStats();

    /**
     * @brief 核心控制函数：计算LQR控制量
     */
    Eigen::Vector3d computeLQRControl(const State& current, const TrajectoryPoint& ref);

    /**
     * @brief 核心控制函数：计算跟踪误差
     */
    TrackingError computeTrackingError(const State& current, const TrajectoryPoint& ref);  // 计算跟踪误差

    /**
     * @brief 初始化函数：从参数服务器加载所有配置参数
     */
    void loadParameters();

    /**
     * @brief 初始化函数：生成跟踪轨迹
     */
    void generateTrajectory();

    /**
     * @brief 主函数：跟踪控制循环
     */
    void trackingLoop();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber pose_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber local_offset_sub_;

    // 发布者：发送控制指令和可视化数据
    ros::Publisher pos_pub_;
    ros::Publisher accel_pub_;
    ros::Publisher ref_path_pub_;
    ros::Publisher adjusted_ref_pub_;
    ros::Publisher flown_path_pub_;
    ros::Publisher pos_error_pub_;
    ros::Publisher vel_error_pub_;
    ros::Publisher total_error_pub_;

    // 控制器和轨迹生成器
    LQRController lqr_controller_;
    TrajectoryGenerator traj_generator_;
    std::vector<TrajectoryPoint> trajectory_;

    // 当前无人机状态
    State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    bool has_state_;
    bool has_mavros_state_;
    mavros_msgs::State mavros_state_;

    // 强化学习局部避障偏移量
    Eigen::Vector3d rl_offset_;
    bool has_rl_offset_;
    ros::Time last_rl_offset_time_;  // 最后收到偏移消息的时间戳，用于超时清零
    double rl_speed_scale_;   // 障碍物检测时的速度缩放系数（0~1）
    double rl_velocity_gain_; // 避障偏移量转为速度分量的增益（Hz）

    // 飞行轨迹可视化
    visualization_msgs::Marker flown_path_marker_;

    // 轨迹参数
    std::string trajectory_type_;
    double trajectory_duration_;
    double control_rate_;
    double dt_;

    // LQR权重参数
    double q_pos_xy_;
    double q_pos_z_;
    double q_vel_xy_;
    double q_vel_z_;
    double r_accel_;

    // 跟踪误差统计
    std::vector<double> position_errors_;
    std::vector<double> velocity_errors_;
    double max_position_error_;
    double mean_position_error_;

    // 轨迹形状参数
    Eigen::Vector3d start_point_;
    Eigen::Vector3d end_point_;
    Eigen::Vector3d center_point_;
    double radius_;
    double height_;
    double angular_velocity_;

    // 固定水平8字形轨迹参数
    FixedHorizontalParams fixed_horizontal_params_;
};

}

#endif
