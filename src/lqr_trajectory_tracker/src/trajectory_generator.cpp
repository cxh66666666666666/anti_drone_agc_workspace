/**
 * @file trajectory_generator.cpp
 * @brief 轨迹生成器类实现
 * @author 陈鑫豪
 * @date 2026-06-08
 */
#include "lqr_trajectory_tracker/trajectory_generator.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <iostream>

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
    double normalized_time = t / duration;
    
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
    double max_velocity = distance / (duration * 0.8);
    
    for (int i = 0; i < num_points; ++i) 
    {
        double t = i * dt;
        TrajectoryPoint point;
        point.timestamp = t;
        
        double s = t / duration;
        point.position = start + s * (end - start);
        
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
 * @brief Catmull-Rom样条插值计算中间点
 * @param p0 第一个点
 * @param p1 第二个点
 * @param p2 第三个点
 * @param p3 第四个点
 * @param t 插值参数
 * @return Eigen::Vector3d 插值结果
 */
Eigen::Vector3d catmullRomSpline(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3,
    double t) 
{
    double t2 = t * t;
    double t3 = t2 * t;

    Eigen::Vector3d result = 0.5 * (
        (2.0 * p1) +
        (-p0 + p2) * t +
        (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
        (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    );

    return result;
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
 * @brief 生成固定水平8字形轨迹
 * @param params 固定水平8字形轨迹参数
 * @param duration 轨迹持续时间
 * @param dt 时间步长
 * @return std::vector<TrajectoryPoint> 固定水平8字形轨迹点向量
 */
std::vector<TrajectoryPoint> TrajectoryGenerator::generateFixedHorizontalTrajectory(
    const FixedHorizontalParams& params,
    double duration,
    double dt) 
{
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;

    Eigen::Vector3d left_center(params.left_center_x, params.left_center_y, params.height);
    Eigen::Vector3d right_center(params.right_center_x, params.right_center_y, params.height);

    double omega = params.angular_velocity;

    for (int i = 0; i < num_points; ++i) 
    {
        double t = i * dt;
        double normalized_t = t / duration;
        double theta_total = 2.0 * M_PI * normalized_t;

        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;

        double linear_vel = omega * params.radius;

        if (theta_total <= M_PI) 
        {
            double theta_right = -M_PI / 2.0 + theta_total;
            position.x() = right_center.x() + params.radius * std::cos(theta_right);
            position.y() = right_center.y() + params.radius * std::sin(theta_right);
            position.z() = params.height;

            velocity.x() = -params.radius * omega * std::sin(theta_right);
            velocity.y() = params.radius * omega * std::cos(theta_right);
            velocity.z() = 0.0;

            acceleration.x() = -params.radius * omega * omega * std::cos(theta_right);
            acceleration.y() = -params.radius * omega * omega * std::sin(theta_right);
            acceleration.z() = 0.0;
        } 
        else 
        {
            double theta_left = M_PI / 2.0 + (theta_total - M_PI);
            position.x() = left_center.x() + params.radius * std::cos(theta_left);
            position.y() = left_center.y() + params.radius * std::sin(theta_left);
            position.z() = params.height;

            velocity.x() = -params.radius * omega * std::sin(theta_left);
            velocity.y() = params.radius * omega * std::cos(theta_left);
            velocity.z() = 0.0;

            acceleration.x() = -params.radius * omega * omega * std::cos(theta_left);
            acceleration.y() = -params.radius * omega * omega * std::sin(theta_left);
            acceleration.z() = 0.0;
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

}
