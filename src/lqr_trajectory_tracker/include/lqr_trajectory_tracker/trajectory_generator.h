#ifndef LQR_TRAJECTORY_TRACKER_TRAJECTORY_GENERATOR_H
#define LQR_TRAJECTORY_TRACKER_TRAJECTORY_GENERATOR_H

#include <vector>
#include <Eigen/Dense>

namespace lqr_trajectory_tracker {

// 轨迹类型枚举
enum class TrajectoryType {
    LINEAR,    // 直线轨迹
    CIRCULAR,  // 圆周轨迹
    HELICAL,   // 螺旋轨迹
    RANDOM,    // 随机轨迹
    FIXED_HORIZONTAL  // 固定水平8字形轨迹
};

// 固定水平8字形轨迹参数结构体
struct FixedHorizontalParams {
    double left_center_x = -3.0;   // 左圆圆心X
    double left_center_y = 0.0;    // 左圆圆心Y
    double right_center_x = 3.0;   // 右圆圆心X
    double right_center_y = 0.0;   // 右圆圆心Y
    double radius = 3.0;           // 圆半径
    double height = 2.0;           // 固定飞行高度
    double angular_velocity = 0.5;  // 角速度
};

// 随机轨迹参数结构体
struct RandomTrajectoryParams {
    int num_waypoints = 10;       // 航迹点数量
    double space_min_x = -20.0;    // 空间范围 X最小值
    double space_max_x = 20.0;    // 空间范围 X最大值
    double space_min_y = -20.0;   // 空间范围 Y最小值
    double space_max_y = 20.0;    // 空间范围 Y最大值
    double space_min_z = 2.0;     // 空间范围 Z最小值
    double space_max_z = 10.0;    // 空间范围 Z最大值
    double max_velocity = 5.0;     // 最大速度 m/s
    bool closed_loop = false;      // 是否闭合轨迹
    int seed = 42;                // 随机种子，0表示每次不同
};

// 轨迹点结构体，包含时间戳、位置、速度和加速度信息
struct TrajectoryPoint {
    double timestamp;       // 时间戳
    Eigen::Vector3d position;      // 位置
    Eigen::Vector3d velocity;       // 速度
    Eigen::Vector3d acceleration;  // 加速度
};

// 轨迹生成器类，用于生成直线、圆周和螺旋轨迹
class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    // 生成直线轨迹
    std::vector<TrajectoryPoint> generateLinearTrajectory(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        double duration,
        double dt);

    // 生成圆周轨迹
    std::vector<TrajectoryPoint> generateCircularTrajectory(
        const Eigen::Vector3d& center,
        double radius,
        double height,
        double angular_velocity,
        double duration,
        double dt);

    // 生成螺旋轨迹
    std::vector<TrajectoryPoint> generateHelicalTrajectory(
        const Eigen::Vector3d& center,
        double radius,
        double height_start,
        double height_end,
        double angular_velocity,
        double duration,
        double dt);

    // 生成随机轨迹
    std::vector<TrajectoryPoint> generateRandomTrajectory(
        const RandomTrajectoryParams& params,
        double duration,
        double dt);

    // 生成固定水平8字形轨迹
    std::vector<TrajectoryPoint> generateFixedHorizontalTrajectory(
        const FixedHorizontalParams& params,
        double duration,
        double dt);

    // 根据时间获取轨迹上的点
    TrajectoryPoint getPointAtTime(const std::vector<TrajectoryPoint>& trajectory, double time) const;

private:
    // 计算速度剖面
    double computeVelocityProfile(double t, double duration) const;
};

}

#endif
