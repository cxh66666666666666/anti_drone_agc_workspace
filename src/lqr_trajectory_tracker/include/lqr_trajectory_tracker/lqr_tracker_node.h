#ifndef LQR_TRAJECTORY_TRACKER_LQR_TRACKER_NODE_H
#define LQR_TRAJECTORY_TRACKER_LQR_TRACKER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>

#include "lqr_trajectory_tracker/lqr_controller.h"
#include "lqr_trajectory_tracker/trajectory_generator.h"

namespace lqr_trajectory_tracker {

// 跟踪误差结构体：存储位置和速度误差的标量及向量形式
struct TrackingError {
    double position_error;          // 位置误差标量（欧氏距离）
    double velocity_error;           // 速度误差标量（速度差模值）
    Eigen::Vector3d position_error_vec;  // 位置误差向量
    Eigen::Vector3d velocity_error_vec;   // 速度误差向量
};

class LQRTrackerNode {
public:
    LQRTrackerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);  // 构造函数，初始化ROS节点句柄
    ~LQRTrackerNode();  // 析构函数

    void run();  // 启动跟踪控制循环

private:
    // 回调函数：处理无人机位姿和状态订阅消息
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);  // 位姿订阅回调
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);  // MAVROS状态订阅回调

    // 发布函数：发布控制指令和可视化数据
    void publishTrajectoryVisualization();  // 发布轨迹可视化
    void publishPositionSetpoint(const Eigen::Vector3d& position);  // 发布位置设定点
    void publishTrackingError(const TrackingError& error);  // 发布跟踪误差
    void logTrackingStats();  // 记录跟踪统计信息

    // 核心控制函数：计算LQR控制量和跟踪误差
    Eigen::Vector3d computeLQRControl(const State& current, const TrajectoryPoint& ref);  // 计算LQR控制输出
    TrackingError computeTrackingError(const State& current, const TrajectoryPoint& ref);  // 计算跟踪误差

    // 模式切换与安全保护函数
    bool disableRTLAndFailsafe();  // 禁用RTL和故障保护
    bool sendSetpointsBeforeOffboard(double duration_seconds, double rate_hz);  // Offboard前发送设定点
    bool waitForOffboardMode(double timeout_seconds);  // 等待进入Offboard模式
    bool setOffboardMode(int max_retries = 3);  // 设置Offboard模式
    bool armMotorsWithCheck();  // 检查并解锁电机

    // 初始化与轨迹生成函数
    void loadParameters();  // 从参数服务器加载参数
    void generateTrajectory();  // 生成跟踪轨迹
    void trackingLoop();  // 主跟踪循环

    // ROS节点句柄
    ros::NodeHandle nh_;  // 公共节点句柄
    ros::NodeHandle pnh_;  // 私有节点句柄

    // 订阅者：接收无人机位姿和状态
    ros::Subscriber pose_sub_;  // 位姿订阅者
    ros::Subscriber state_sub_;  // MAVROS状态订阅者

    // 发布者：发送控制指令和可视化数据
    ros::Publisher pos_pub_;  // 位置设定点发布者
    ros::Publisher accel_pub_;  // 加速度指令发布者
    ros::Publisher ref_path_pub_;  // 参考轨迹发布者
    ros::Publisher pos_error_pub_;  // 位置误差发布者
    ros::Publisher vel_error_pub_;  // 速度误差发布者
    ros::Publisher total_error_pub_;  // 总误差发布者

    // 服务客户端：模式切换和电机解锁
    ros::ServiceClient set_mode_client_;  // 模式设置服务客户端
    ros::ServiceClient arming_client_;  // 电机解锁服务客户端
    ros::ServiceClient param_set_client_;  // 参数设置服务客户端

    // 控制器和轨迹生成器
    LQRController lqr_controller_;  // LQR控制器实例
    TrajectoryGenerator traj_generator_;  // 轨迹生成器实例
    std::vector<TrajectoryPoint> trajectory_;  // 跟踪轨迹点集合

    // 当前无人机状态
    State current_state_;  // 当前状态（位置、速度等）
    geometry_msgs::PoseStamped current_pose_;  // 当前位姿消息
    bool has_state_;  // 是否已接收状态标志
    bool has_mavros_state_;  // 是否已接收MAVROS状态标志
    mavros_msgs::State mavros_state_;  // MAVROS状态消息

    // 轨迹参数
    std::string trajectory_type_;  // 轨迹类型
    double trajectory_duration_;  // 轨迹总时长
    double control_rate_;  // 控制频率
    double dt_;  // 控制时间步长

    // LQR权重参数
    double q_pos_xy_;  // XY位置误差权重
    double q_pos_z_;  // Z位置误差权重
    double q_vel_xy_;  // XY速度误差权重
    double q_vel_z_;  // Z速度误差权重
    double r_accel_;  // 加速度控制权重

    // 跟踪误差统计
    std::vector<double> position_errors_;  // 历史位置误差记录
    std::vector<double> velocity_errors_;  // 历史速度误差记录
    double max_position_error_;  // 最大位置误差
    double mean_position_error_;  // 平均位置误差

    // 轨迹形状参数
    Eigen::Vector3d start_point_;  // 轨迹起始点
    Eigen::Vector3d end_point_;  // 轨迹结束点
    Eigen::Vector3d center_point_;  // 圆形轨迹圆心
    double radius_;  // 圆形轨迹半径
    double height_;  // 轨迹高度
    double angular_velocity_;  // 角速度（圆形轨迹）

    // 随机轨迹参数
    RandomTrajectoryParams random_params_;  // 随机轨迹参数

    // 固定水平8字形轨迹参数
    FixedHorizontalParams fixed_horizontal_params_;  // 固定水平轨迹参数
};

}

#endif
