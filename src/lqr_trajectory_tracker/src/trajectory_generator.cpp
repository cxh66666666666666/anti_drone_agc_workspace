#include "lqr_trajectory_tracker/trajectory_generator.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <iostream>

namespace lqr_trajectory_tracker {

TrajectoryGenerator::TrajectoryGenerator() {}

TrajectoryGenerator::~TrajectoryGenerator() {}

// computeVelocityProfile: 计算速度配置文件
double TrajectoryGenerator::computeVelocityProfile(double t, double duration) const {
    double normalized_time = t / duration;
    
    if (normalized_time < 0.1) {
        return normalized_time / 0.1;
    } else if (normalized_time > 0.9) {
        return (1.0 - normalized_time) / 0.1;
    }
    return 1.0;
}

// generateLinearTrajectory: 生成线性轨迹
std::vector<TrajectoryPoint> TrajectoryGenerator::generateLinearTrajectory(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double duration,
    double dt) {
    
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;
    
    Eigen::Vector3d direction = end - start;
    double distance = direction.norm();
    
    if (distance < 1e-6) {
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
    
    for (int i = 0; i < num_points; ++i) {
        double t = i * dt;
        TrajectoryPoint point;
        point.timestamp = t;
        
        double s = t / duration;
        point.position = start + s * (end - start);
        
        double vel_profile = computeVelocityProfile(t, duration);
        point.velocity = direction * max_velocity * vel_profile;
        
        double accel_profile = 0.0;
        if (s < 0.1) {
            accel_profile = max_velocity / (0.1 * duration);
        } else if (s > 0.9) {
            accel_profile = -max_velocity / (0.1 * duration);
        }
        point.acceleration = direction * accel_profile;
        
        trajectory.push_back(point);
    }
    
    return trajectory;
}

// generateCircularTrajectory: 生成圆形轨迹
std::vector<TrajectoryPoint> TrajectoryGenerator::generateCircularTrajectory(
    const Eigen::Vector3d& center,
    double radius,
    double height,
    double angular_velocity,
    double duration,
    double dt) {
    
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;
    
    for (int i = 0; i < num_points; ++i) {
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

// generateHelicalTrajectory: 生成螺旋轨迹
std::vector<TrajectoryPoint> TrajectoryGenerator::generateHelicalTrajectory(
    const Eigen::Vector3d& center,
    double radius,
    double height_start,
    double height_end,
    double angular_velocity,
    double duration,
    double dt) {
    
    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;
    double height_diff = height_end - height_start;
    double vertical_velocity = height_diff / duration;
    
    for (int i = 0; i < num_points; ++i) {
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

// catmullRomSpline: Catmull-Rom样条插值计算中间点
Eigen::Vector3d catmullRomSpline(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3,
    double t) {
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

// generateRandomTrajectory: 生成随机轨迹
std::vector<TrajectoryPoint> TrajectoryGenerator::generateRandomTrajectory(
    const RandomTrajectoryParams& params,
    double duration,
    double dt) {

    std::vector<TrajectoryPoint> trajectory;
    std::mt19937 gen;
    if (params.seed > 0) {
        gen.seed(params.seed);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    std::uniform_real_distribution<> dis_x(params.space_min_x, params.space_max_x);
    std::uniform_real_distribution<> dis_y(params.space_min_y, params.space_max_y);
    std::uniform_real_distribution<> dis_z(params.space_min_z, params.space_max_z);

    int num_waypoints = params.num_waypoints;
    if (num_waypoints < 4) {
        num_waypoints = 4;
    }

    std::vector<Eigen::Vector3d> waypoints;
    for (int i = 0; i < num_waypoints; ++i) {
        Eigen::Vector3d wp;
        wp.x() = dis_x(gen);
        wp.y() = dis_y(gen);
        wp.z() = dis_z(gen);
        waypoints.push_back(wp);
    }

    if (params.closed_loop) {
        waypoints.push_back(waypoints.front());
    }

    std::vector<Eigen::Vector3d> smoothed_positions;
    const int interpolation_points = 10;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const Eigen::Vector3d& p0 = waypoints[(i - 1 + waypoints.size()) % waypoints.size()];
        const Eigen::Vector3d& p1 = waypoints[i];
        const Eigen::Vector3d& p2 = waypoints[(i + 1) % waypoints.size()];
        const Eigen::Vector3d& p3 = waypoints[(i + 2) % waypoints.size()];

        for (int j = 0; j < interpolation_points; ++j) {
            double t = static_cast<double>(j) / interpolation_points;
            Eigen::Vector3d pos = catmullRomSpline(p0, p1, p2, p3, t);
            smoothed_positions.push_back(pos);
        }
    }

    std::vector<double> segment_distances;
    double total_distance = 0.0;
    for (size_t i = 0; i < smoothed_positions.size(); ++i) {
        size_t next = (i + 1) % smoothed_positions.size();
        double dist = (smoothed_positions[next] - smoothed_positions[i]).norm();
        segment_distances.push_back(dist);
        total_distance += dist;
    }

    int num_points = static_cast<int>(duration / dt) + 1;
    double velocity = params.max_velocity;

    for (int i = 0; i < num_points; ++i) {
        double t = i * dt;
        double normalized_t = t / duration;
        double cumulative_distance = normalized_t * total_distance;

        double current_dist = 0.0;
        size_t segment_idx = 0;
        for (size_t j = 0; j < segment_distances.size(); ++j) {
            if (current_dist + segment_distances[j] >= cumulative_distance) {
                segment_idx = j;
                break;
            }
            current_dist += segment_distances[j];
        }

        size_t next_idx = (segment_idx + 1) % smoothed_positions.size();
        double segment_t = 0.0;
        if (segment_distances[segment_idx] > 1e-6) {
            segment_t = (cumulative_distance - current_dist) / segment_distances[segment_idx];
        }

        TrajectoryPoint point;
        point.timestamp = t;
        point.position = smoothed_positions[segment_idx] +
                         segment_t * (smoothed_positions[next_idx] - smoothed_positions[segment_idx]);

        Eigen::Vector3d next_pos = smoothed_positions[(segment_idx + 1) % smoothed_positions.size()];
        Eigen::Vector3d prev_pos = smoothed_positions[(segment_idx - 1 + smoothed_positions.size()) % smoothed_positions.size()];
        Eigen::Vector3d tangent = (next_pos - prev_pos) / 2.0;
        point.velocity = tangent.normalized() * velocity;

        Eigen::Vector3d next_tangent = smoothed_positions[(segment_idx + 2) % smoothed_positions.size()] -
                                       smoothed_positions[segment_idx];
        point.acceleration = (next_tangent.normalized() * velocity - point.velocity) / dt;

        trajectory.push_back(point);
    }

    return trajectory;
}

// getPointAtTime: 获取指定时间点的轨迹点
TrajectoryPoint TrajectoryGenerator::getPointAtTime(
    const std::vector<TrajectoryPoint>& trajectory, 
    double time) const {
    
    if (trajectory.empty()) {
        return TrajectoryPoint();
    }
    
    if (time <= trajectory.front().timestamp) {
        return trajectory.front();
    }
    
    if (time >= trajectory.back().timestamp) {
        return trajectory.back();
    }
    
    for (size_t i = 0; i < trajectory.size() - 1; ++i) {
        if (time >= trajectory[i].timestamp && time <= trajectory[i + 1].timestamp) {
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

std::vector<TrajectoryPoint> TrajectoryGenerator::generateFixedHorizontalTrajectory(
    const FixedHorizontalParams& params,
    double duration,
    double dt) {

    std::vector<TrajectoryPoint> trajectory;
    int num_points = static_cast<int>(duration / dt) + 1;

    Eigen::Vector3d left_center(params.left_center_x, params.left_center_y, params.height);
    Eigen::Vector3d right_center(params.right_center_x, params.right_center_y, params.height);

    double omega = params.angular_velocity;

    for (int i = 0; i < num_points; ++i) {
        double t = i * dt;
        double normalized_t = t / duration;
        double theta_total = 2.0 * M_PI * normalized_t;

        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;

        double linear_vel = omega * params.radius;

        if (theta_total <= M_PI) {
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
        } else {
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
