/**
 * @file trajectory_generator.h
 * @brief 轨迹生成器类接口
 * @author 陈鑫豪
 * @date 2026-06-08
 */

#ifndef LQR_TRAJECTORY_TRACKER_TRAJECTORY_GENERATOR_H
#define LQR_TRAJECTORY_TRACKER_TRAJECTORY_GENERATOR_H

#include <vector>
#include <Eigen/Dense>

namespace lqr_trajectory_tracker {

/**
 * @brief 轨迹类型枚举
 */
enum class TrajectoryType {
    LINEAR,
    CIRCULAR,
    HELICAL,
    FIXED_HORIZONTAL
};

/**
 * @brief 固定水平8字形轨迹参数结构体
 */
struct FixedHorizontalParams {
    double left_center_x = -3.0;
    double left_center_y = 0.0;
    double right_center_x = 3.0;
    double right_center_y = 0.0;
    double radius = 3.0;
    double height = 2.0;
    double angular_velocity = 0.5;
};

/**
 * @brief 轨迹点结构体，包含时间戳、位置、速度和加速度信息
 */
struct TrajectoryPoint {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
};

class TrajectoryGenerator {
public:

    /**
     * @brief 构造函数
     */
    TrajectoryGenerator();
    
    /**
     * @brief 析构函数
     */
    ~TrajectoryGenerator();

    /**
     * @brief 生成直线轨迹
     * @param start 起始点
     * @param end 结束点
     * @param duration 轨迹持续时间
     * @param dt 时间步长
     * @return std::vector<TrajectoryPoint> 直线轨迹点向量
     */
    std::vector<TrajectoryPoint> generateLinearTrajectory(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        double duration,
        double dt);

    /**
     * @brief 生成圆周轨迹
     * @param center 圆心
     * @param radius 半径
     * @param height 高度
     * @param angular_velocity 角速度
     * @param duration 轨迹持续时间
     * @param dt 时间步长
     * @return std::vector<TrajectoryPoint> 圆周轨迹点向量
     */
    std::vector<TrajectoryPoint> generateCircularTrajectory(
        const Eigen::Vector3d& center,
        double radius,
        double height,
        double angular_velocity,
        double duration,
        double dt);

    /**
     * @brief 生成螺旋轨迹
     * @param center 螺旋中心
     * @param radius 螺旋轨迹半径
     * @param height_start 螺旋轨迹起高度
     * @param height_end 螺旋轨迹终高度
     * @param angular_velocity 螺旋轨迹角速度
     * @param duration 轨迹持续时间
     * @param dt 时间步长
     * @return std::vector<TrajectoryPoint> 螺旋轨迹点向量
     */
    std::vector<TrajectoryPoint> generateHelicalTrajectory(
        const Eigen::Vector3d& center,
        double radius,
        double height_start,
        double height_end,
        double angular_velocity,
        double duration,
        double dt);

    /**
     * @brief 生成固定水平8字形轨迹
     * @param params 固定水平8字形轨迹参数
     * @param duration 轨迹持续时间
     * @param dt 时间步长
     * @return std::vector<TrajectoryPoint> 固定水平8字形轨迹点向量
     */
    std::vector<TrajectoryPoint> generateFixedHorizontalTrajectory(
        const FixedHorizontalParams& params,
        double duration,
        double dt);

    /**
     * @brief 根据时间获取轨迹上的点
     * @param trajectory 轨迹点向量
     * @param time 时间戳
     * @return TrajectoryPoint 轨迹点
     */
    TrajectoryPoint getPointAtTime(const std::vector<TrajectoryPoint>& trajectory, double time) const;

private:
    /**
     * @brief 计算速度剖面
     * @param t 时间戳
     * @param duration 轨迹持续时间
     * @return double 速度剖面
     */
    double computeVelocityProfile(double t, double duration) const;
};

}

#endif
