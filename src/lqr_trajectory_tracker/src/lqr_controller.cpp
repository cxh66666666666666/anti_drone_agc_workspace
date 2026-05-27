#include "lqr_trajectory_tracker/lqr_controller.h"
#include <iostream>

namespace lqr_trajectory_tracker {

LQRController::LQRController() 
    : dt_(0.01) // 控制周期0.01秒，100hz
    , is_initialized_(false)
{
    initializeSystemMatrices();
    
    Q_ = Eigen::Matrix<double, 6, 6>::Identity();
    Q_.topLeftCorner<3, 3>() *= 10.0; // 位置权重10
    Q_.bottomRightCorner<3, 3>() *= 1.0; // 速度权重1
    
    R_ = Eigen::Matrix3d::Identity() * 0.1; // 控制权重0.1
}

LQRController::~LQRController() {}

/* A = [ I  dt·I ]    B = [ 0 ]
       [ 0   I  ]        [ dt·I ] */
void LQRController::initializeSystemMatrices() {
    A_ = Eigen::Matrix<double, 6, 6>::Identity();
    // 右上角3x3 = dt*I（速度对位置的积分）
    A_.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * dt_;
    
    B_ = Eigen::Matrix<double, 6, 3>::Zero();
    // 下3行 = dt*I（加速度对速度的积分）
    B_.bottomRows<3>() = Eigen::Matrix3d::Identity() * dt_;
}

/* Q = [ q_pos(3x3)      0(3x3)  ]
       [ 0(3x3)        q_vel(3x3) ] */
void LQRController::setQ(const Eigen::Matrix3d& q_pos, const Eigen::Matrix3d& q_vel) {
    Q_.topLeftCorner<3, 3>() = q_pos;
    Q_.bottomRightCorner<3, 3>() = q_vel;
}

void LQRController::setR(const Eigen::Matrix3d& r) {
    R_ = r;
}

void LQRController::setDt(double dt) {
    dt_ = dt;
    initializeSystemMatrices();
    is_initialized_ = false;
}

// 求解Riccati方程
bool LQRController::solveRiccati() {
    const int max_iterations = 1000; // 最多迭代数1000次
    const double tolerance = 1e-6;   // 收敛容差1e-6
    
    Eigen::Matrix<double, 6, 6> P = Q_;
    Eigen::Matrix<double, 6, 6> P_new;
    
    for (int i = 0; i < max_iterations; ++i) {
        // P_new = Q + A'·P·A - A'·P·B·(R + B'·P·B)^(-1)·B'·P·A
        P_new = Q_ + A_.transpose() * P * A_ 
                - A_.transpose() * P * B_ 
                * (R_ + B_.transpose() * P * B_).inverse() 
                * B_.transpose() * P * A_;
        
        if ((P_new - P).norm() < tolerance) {
            P = P_new;
            break;
        }
        P = P_new;
    }
    
    // 最优反馈增益K = (R + B'·P·B)^(-1)·B'·P·A
    K_ = (R_ + B_.transpose() * P * B_).inverse() * B_.transpose() * P * A_;
    is_initialized_ = true;
    
    return true;
}

ControlOutput LQRController::computeControl(const State& current, const State& reference) {
    ControlOutput output;
    
    // 如果K未初始化，先求解Riccati
    if (!is_initialized_) {
        solveRiccati();
    }
    
    // 构造误差向量（6x1）
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = current.position - reference.position;
    error.tail<3>() = current.velocity - reference.velocity;
    
    // u = -K·error
    output.acceleration = -K_ * error;
    
    // 限制最大加速度（±10 m/s²）
    const double max_accel = 10.0;
    output.acceleration = output.acceleration.cwiseMax(-max_accel).cwiseMin(max_accel);
    
    return output;
}

}
