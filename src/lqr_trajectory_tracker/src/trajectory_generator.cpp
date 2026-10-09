/**
 * @file trajectory_generator.cpp
 * @brief 轨迹生成器类实现
 * @author 陈鑫豪
 * @date 2026-06-08
 */
#include "lqr_trajectory_tracker/trajectory_generator.h"
#include <cmath>
#include <algorithm>

namespace lqr_trajectory_tracker {

/**
 * @brief 轨迹生成器类构造函数
 */
TrajectoryGenerator::TrajectoryGenerator() {}

/**
 * @brief 轨迹生成器类析构函数
 */
TrajectoryGenerator::~TrajectoryGenerator() {}

/**
 * @brief 计算速度剖面
 * @param t 时间戳
 * @param duration 轨迹持续时间
 * @return double 速度剖面
 */
double TrajectoryGenerator::computeVelocityProfile(double t, double duration) const 
{
    double normalized_time = std::max(0.0, std::min(1.0, t / duration));
    
    if (normalized_time < 0.1) 
    {
        return normalized_time / 0.1;
    }
    else if (normalized_time > 0.9) 
    {
        return (1.0 - normalized_time) / 0.1;
    }
    return 1.0;
}

// Integral of computeVelocityProfile over normalized time.  The profile area
// is 0.9 (0.05 + 0.8 + 0.05), so callers can normalize it to the requested
// displacement.
static double integratedVelocityProfile(double normalized_time)
{
    const double u = std::max(0.0, std::min(1.0, normalized_time));
    if (u < 0.1)
        return 5.0 * u * u;
    if (u <= 0.9)
        return u - 0.05;
    return 10.0 * u - 5.0 * u * u - 4.10;
}

/**
 * @brief 生成线性轨迹
 * @param start 起始点
 * @param end 终点
 * @param duration 轨迹持续时间
 * @param dt 时间步长
 * @return std::vector<TrajectoryPoint> 线性轨迹点向量
 */
std::vector<TrajectoryPoint> TrajectoryGenerator::generateLinearTrajectory(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double duration,
    double dt) 
{
    
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;
    
    Eigen::Vector3d direction = end - start;
    double distance = direction.norm();
    
    if (distance < 1e-6) 
    {
        TrajectoryPoint point;
        point.timestamp = 0;
        point.position = start;
        point.velocity = Eigen::Vector3d::Zero();
        point.acceleration = Eigen::Vector3d::Zero();
        trajectory.push_back(point);
        return trajectory;
    }
    
    direction.normalize();
    const double profile_area = 0.9;
    double max_velocity = distance / (duration * profile_area);
    
    for (int i = 0; i < num_points; ++i) 
    {
        double t = i * dt;
        TrajectoryPoint point;
        point.timestamp = t;
        
        double s = std::max(0.0, std::min(1.0, t / duration));
        double progress = integratedVelocityProfile(s) / profile_area;
        point.position = start + progress * (end - start);
        
        double vel_profile = computeVelocityProfile(t, duration);
        point.velocity = direction * max_velocity * vel_profile;
        
        double accel_profile = 0.0;
        if (s < 0.1) 
        {
            accel_profile = max_velocity / (0.1 * duration);
        } 
        else if (s > 0.9) {
            accel_profile = -max_velocity / (0.1 * duration);
        }
        point.acceleration = direction * accel_profile;
        
        trajectory.push_back(point);
    }
    
    return trajectory;
}

/**
 * @brief 生成圆形轨迹
 * @param center 圆心
 * @param radius 半径
 * @param height 高度
 * @param angular_velocity 角速度
 * @param duration 轨迹持续时间
 * @param dt 时间步长
 * @return std::vector<TrajectoryPoint> 圆形轨迹点向量
 */
std::vector<TrajectoryPoint> TrajectoryGenerator::generateCircularTrajectory(
    const Eigen::Vector3d& center,
    double radius,
    double height,
    double angular_velocity,
    double duration,
    double dt) 
{
    
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;
    
    for (int i = 0; i < num_points; ++i) 
    {
        double t = i * dt;
        double theta = angular_velocity * t;
        
        TrajectoryPoint point;
        point.timestamp = t;
        
        point.position.x() = center.x() + radius * std::cos(theta);
        point.position.y() = center.y() + radius * std::sin(theta);
        point.position.z() = height;
        
        point.velocity.x() = -radius * angular_velocity * std::sin(theta);
        point.velocity.y() = radius * angular_velocity * std::cos(theta);
        point.velocity.z() = 0.0;
        
        point.acceleration.x() = -radius * angular_velocity * angular_velocity * std::cos(theta);
        point.acceleration.y() = -radius * angular_velocity * angular_velocity * std::sin(theta);
        point.acceleration.z() = 0.0;
        
        trajectory.push_back(point);
    }
    
    return trajectory;
}

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
std::vector<TrajectoryPoint> TrajectoryGenerator::generateHelicalTrajectory(
    const Eigen::Vector3d& center,
    double radius,
    double height_start,
    double height_end,
    double angular_velocity,
    double duration,
    double dt) 
{
    
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;
    double height_diff = height_end - height_start;
    double vertical_velocity = height_diff / duration;
    
    for (int i = 0; i < num_points; ++i) 
    {
        double t = i * dt;
        double theta = angular_velocity * t;
        
        TrajectoryPoint point;
        point.timestamp = t;
        
        point.position.x() = center.x() + radius * std::cos(theta);
        point.position.y() = center.y() + radius * std::sin(theta);
        point.position.z() = height_start + vertical_velocity * t;
        
        point.velocity.x() = -radius * angular_velocity * std::sin(theta);
        point.velocity.y() = radius * angular_velocity * std::cos(theta);
        point.velocity.z() = vertical_velocity;
        
        point.acceleration.x() = -radius * angular_velocity * angular_velocity * std::cos(theta);
        point.acceleration.y() = -radius * angular_velocity * angular_velocity * std::sin(theta);
        point.acceleration.z() = 0.0;
        
        trajectory.push_back(point);
    }

    return trajectory;
}

/**
 * @brief 获取指定时间点的轨迹点
 * @param trajectory 轨迹点向量
 * @param time 时间戳
 * @return TrajectoryPoint 轨迹点
 */
TrajectoryPoint TrajectoryGenerator::getPointAtTime(
    const std::vector<TrajectoryPoint>& trajectory, 
    double time) const 
{
    
    if (trajectory.empty()) 
    {
        return TrajectoryPoint();
    }
    
    if (time <= trajectory.front().timestamp) 
    {
        return trajectory.front();
    }
    
    if (time >= trajectory.back().timestamp) 
    {
        return trajectory.back();
    }
    
    for (size_t i = 0; i < trajectory.size() - 1; ++i) 
    {
        if (time >= trajectory[i].timestamp && time <= trajectory[i + 1].timestamp) 
        {
            double alpha = (time - trajectory[i].timestamp) / 
                          (trajectory[i + 1].timestamp - trajectory[i].timestamp);
            
            TrajectoryPoint point;
            point.timestamp = time;
            point.position = (1 - alpha) * trajectory[i].position + 
                            alpha * trajectory[i + 1].position;
            point.velocity = (1 - alpha) * trajectory[i].velocity + 
                            alpha * trajectory[i + 1].velocity;
            point.acceleration = (1 - alpha) * trajectory[i].acceleration +
                                alpha * trajectory[i + 1].acceleration;
            return point;
        }
    }

    return trajectory.back();
}

/**
 * @brief 生成历史固定水平跑道形轨迹（右半圆→上直线→左半圆→下直线）
 * @param params 固定水平跑道形轨迹参数
 * @param duration 轨迹持续时间（仅决定采样点数，不改变角速率）
 * @param dt 时间步长
 * @return std::vector<TrajectoryPoint> 固定水平8字形轨迹点向量
 */
std::vector<TrajectoryPoint> TrajectoryGenerator::generateFixedHorizontalTrajectory(
    const FixedHorizontalParams& params,
    double duration,
    double dt) 
{
    std::vector<TrajectoryPoint> trajectory;

    Eigen::Vector3d left_center(params.left_center_x, params.left_center_y, params.height);
    Eigen::Vector3d right_center(params.right_center_x, params.right_center_y, params.height);

    double omega = params.angular_velocity;
    double r = params.radius;
    double linear_speed = omega * r;  // 恒定线速度

    // 各段边界点（圆弧与直线交接处）
    Eigen::Vector3d top_right(right_center.x(), right_center.y() + r, params.height);
    Eigen::Vector3d top_left(left_center.x(),  left_center.y()  + r, params.height);
    Eigen::Vector3d bot_left(left_center.x(),  left_center.y()  - r, params.height);
    Eigen::Vector3d bot_right(right_center.x(), right_center.y() - r, params.height);

    // 各段持续时间：半圆弧 = π/ω，直线段 = 2r/(rω) = 2/ω
    double T_arc = M_PI / omega;
    double T_straight = 2.0 / omega;
    double T_total = 2.0 * (T_arc + T_straight);

    // 至少生成一个完整周期，确保4段轨迹都存在
    double effective_duration = std::max(duration, T_total);
    int num_points = static_cast<int>(effective_duration / dt) + 1;

    for (int i = 0; i < num_points; ++i) 
    {
        double t = i * dt;
        double t_cycle = std::fmod(t, T_total);

        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;

        if (t_cycle < T_arc)
        {
            // 段1: 右半圆 (θ: -π/2 → π/2)，顺时针
            double theta = -M_PI / 2.0 + omega * t_cycle;
            position.x() = right_center.x() + r * std::cos(theta);
            position.y() = right_center.y() + r * std::sin(theta);
            position.z() = params.height;

            velocity.x() = -r * omega * std::sin(theta);
            velocity.y() =  r * omega * std::cos(theta);
            velocity.z() = 0.0;

            acceleration.x() = -r * omega * omega * std::cos(theta);
            acceleration.y() = -r * omega * omega * std::sin(theta);
            acceleration.z() = 0.0;
        }
        else if (t_cycle < T_arc + T_straight)
        {
            // 段2: 上直线 (3,3) → (-3,3)，向左匀速
            double progress = (t_cycle - T_arc) / T_straight;
            position = top_right + progress * (top_left - top_right);
            velocity = Eigen::Vector3d(-linear_speed, 0.0, 0.0);
            acceleration = Eigen::Vector3d::Zero();
        }
        else if (t_cycle < 2.0 * T_arc + T_straight)
        {
            // 段3: 左半圆 (θ: π/2 → 3π/2)，顺时针
            double t_local = t_cycle - (T_arc + T_straight);
            double theta = M_PI / 2.0 + omega * t_local;
            position.x() = left_center.x() + r * std::cos(theta);
            position.y() = left_center.y() + r * std::sin(theta);
            position.z() = params.height;

            velocity.x() = -r * omega * std::sin(theta);
            velocity.y() =  r * omega * std::cos(theta);
            velocity.z() = 0.0;

            acceleration.x() = -r * omega * omega * std::cos(theta);
            acceleration.y() = -r * omega * omega * std::sin(theta);
            acceleration.z() = 0.0;
        }
        else
        {
            // 段4: 下直线 (-3,-3) → (3,-3)，向右匀速
            double progress = (t_cycle - (2.0 * T_arc + T_straight)) / T_straight;
            position = bot_left + progress * (bot_right - bot_left);
            velocity = Eigen::Vector3d(linear_speed, 0.0, 0.0);
            acceleration = Eigen::Vector3d::Zero();
        }

        TrajectoryPoint point;
        point.timestamp = t;
        point.position = position;
        point.velocity = velocity;
        point.acceleration = acceleration;
        trajectory.push_back(point);
    }

    return trajectory;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateFigureEightTrajectory(
    const FigureEightParams& params,
    double duration,
    double dt)
{
    std::vector<TrajectoryPoint> trajectory;
    const int num_points = static_cast<int>(duration / dt) + 1;
    const double omega = params.angular_velocity;

    for (int i = 0; i < num_points; ++i)
    {
        const double t = i * dt;
        const double theta = omega * t;
        TrajectoryPoint point;
        point.timestamp = t;
        point.position.x() = params.center_x + params.radius_x * std::sin(theta);
        point.position.y() = params.center_y + params.radius_y * std::sin(theta) * std::cos(theta);
        point.position.z() = params.height;
        point.velocity.x() = params.radius_x * omega * std::cos(theta);
        point.velocity.y() = params.radius_y * omega * std::cos(2.0 * theta);
        point.velocity.z() = 0.0;
        point.acceleration.x() = -params.radius_x * omega * omega * std::sin(theta);
        point.acceleration.y() = -2.0 * params.radius_y * omega * omega * std::sin(2.0 * theta);
        point.acceleration.z() = 0.0;
        trajectory.push_back(point);
    }
    return trajectory;
}

/**
 * @brief 生成悬停轨迹（所有点位置相同，零速度/零加速度）
 * @param position 悬停位置
 * @param duration 轨迹持续时间
 * @param dt 时间步长
 * @return std::vector<TrajectoryPoint> 悬停轨迹点向量
 */
std::vector<TrajectoryPoint> TrajectoryGenerator::generateStepTrajectory(
    const Eigen::Vector3d& position,
    double duration,
    double dt)
{
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;

    for (int i = 0; i < num_points; ++i)
    {
        TrajectoryPoint point;
        point.timestamp = i * dt;
        point.position = position;
        point.velocity = Eigen::Vector3d::Zero();
        point.acceleration = Eigen::Vector3d::Zero();
        trajectory.push_back(point);
    }

    return trajectory;
}

/**
 * @brief 生成航点轨迹（按弧长等速重采样，末点速度置零）
 */
std::vector<TrajectoryPoint> TrajectoryGenerator::generateWaypointTrajectory(
    const std::vector<Eigen::Vector3d>& waypoints,
    double speed,
    double dt)
{
    std::vector<TrajectoryPoint> trajectory;
    if (waypoints.size() < 2 || speed <= 0.0 || dt <= 0.0)
        return trajectory;

    std::vector<double> cumulative(waypoints.size(), 0.0);
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        cumulative[i] = cumulative[i - 1] + (waypoints[i] - waypoints[i - 1]).norm();
    }
    const double total_length = cumulative.back();
    if (total_length < 1e-6)
        return trajectory;

    const double duration = total_length / speed;
    const int num_points = static_cast<int>(duration / dt) + 2;
    size_t segment = 0;

    for (int i = 0; i < num_points; ++i)
    {
        const double s = std::min(static_cast<double>(i) * dt * speed, total_length);
        while (segment + 1 < waypoints.size() && cumulative[segment + 1] < s)
            ++segment;

        TrajectoryPoint point;
        point.timestamp = i * dt;
        point.acceleration = Eigen::Vector3d::Zero();
        if (segment + 1 < waypoints.size())
        {
            const double seg_len = cumulative[segment + 1] - cumulative[segment];
            const double ratio = seg_len > 1e-9
                ? (s - cumulative[segment]) / seg_len : 0.0;
            point.position = waypoints[segment]
                + ratio * (waypoints[segment + 1] - waypoints[segment]);
            point.velocity = speed
                * (waypoints[segment + 1] - waypoints[segment]).normalized();
        }
        else
        {
            point.position = waypoints.back();
            point.velocity = Eigen::Vector3d::Zero();
        }
        trajectory.push_back(point);
    }
    return trajectory;
}

}
