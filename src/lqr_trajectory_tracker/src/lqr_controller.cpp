/**
 * @file lqr_controller.cpp
 * @brief LQR控制器实现
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#include "lqr_trajectory_tracker/lqr_controller.h"
#include <iostream>

namespace lqr_trajectory_tracker {

/**
 * @brief 构造函数
 * @details 速度权重10，加速度权重1，控制权重0.1
 * @param dt 控制周期，默认0.01秒，100hz
 */
LQRController::LQRController() 
    : dt_(0.01)
    , is_initialized_(false)
{
    initializeSystemMatrices();
    
    Q_ = Eigen::Matrix<double, 6, 6>::Identity();
    Q_.topLeftCorner<3, 3>() *= 10.0;
    Q_.bottomRightCorner<3, 3>() *= 1.0;
    
    R_ = Eigen::Matrix3d::Identity() * 0.1;
}

/**
 * @brief 析构函数
 */
LQRController::~LQRController() {}

/**
 * @brief 初始化系统矩阵A和B
 * @details A = [ I  dt·I ]    B = [ 0 ]
 *              [ 0   I  ]        [ dt·I ]
 */
void LQRController::initializeSystemMatrices() 
{
    A_ = Eigen::Matrix<double, 6, 6>::Identity();
    // 右上角3x3 = dt*I（速度对位置的积分）
    A_.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * dt_;
    
    B_ = Eigen::Matrix<double, 6, 3>::Zero();
    // 下3行 = dt*I（加速度对速度的积分）
    B_.bottomRows<3>() = Eigen::Matrix3d::Identity() * dt_;
}

/**
 * @brief 设置状态权重矩阵Q
 * @details Q = [ q_pos(3x3)      0(3x3)  ]
 *              [ 0(3x3)        q_vel(3x3) ]
 * @param q_pos 位置权重矩阵
 * @param q_vel 速度权重矩阵
 */
void LQRController::setQ(const Eigen::Matrix3d& q_pos, const Eigen::Matrix3d& q_vel) 
{
    Q_.topLeftCorner<3, 3>() = q_pos;
    Q_.bottomRightCorner<3, 3>() = q_vel;
}

/**
 * @brief 设置控制权重矩阵R
 * @details R = [ 0(3x3) 0(3x3) ]
 * @param r 控制权重矩阵
 */
void LQRController::setR(const Eigen::Matrix3d& r) 
{
    R_ = r;
}

/**
 * @brief 设置控制周期
 * @param dt 控制周期，默认0.01秒，100hz
 */
void LQRController::setDt(double dt) 
{
    dt_ = dt;
    initializeSystemMatrices();
    is_initialized_ = false;
}

/**
 * @brief 求解Riccati方程
 * @return 是否成功求解
 */
bool LQRController::solveRiccati() 
{
    const int max_iterations = 1000;
    // 收敛容差1e-6
    const double tolerance = 1e-6;
    
    Eigen::Matrix<double, 6, 6> P = Q_;
    Eigen::Matrix<double, 6, 6> P_new;
    
    for (int i = 0; i < max_iterations; ++i) 
    {
        // P_new = Q + A'·P·A - A'·P·B·(R + B'·P·B)^(-1)·B'·P·A
        P_new = Q_ + A_.transpose() * P * A_ 
                - A_.transpose() * P * B_ 
                * (R_ + B_.transpose() * P * B_).inverse() 
                * B_.transpose() * P * A_;
        
        if ((P_new - P).norm() < tolerance) 
        {
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

/**
 * @brief 计算控制输入
 * @param current 当前状态
 * @param reference 参考状态
 * @return 控制输出
 */
ControlOutput LQRController::computeControl(const State& current, const State& reference) 
{
    ControlOutput output;
    
    if (!is_initialized_) 
    {
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