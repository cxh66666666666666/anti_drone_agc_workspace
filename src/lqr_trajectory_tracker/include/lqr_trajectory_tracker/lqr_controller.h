/**
 * @file lqr_controller.h
 * @brief LQR 控制器头文件
 * @author 陈鑫豪
 * @date 2026-05-29
 */
#ifndef LQR_TRAJECTORY_TRACKER_LQR_CONTROLLER_H
#define LQR_TRAJECTORY_TRACKER_LQR_CONTROLLER_H

#include <Eigen/Dense>

namespace lqr_trajectory_tracker {

struct State 
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;

    State() : position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()) {}
};

struct ControlOutput 
{
    Eigen::Vector3d acceleration;
};

class LQRController 
{
public:
    /**
     * @brief 构造函数
     */
    LQRController();

    /**
     * @brief 析构函数
     */
    ~LQRController();

    /**
     * @brief 设置状态权重矩阵
     * @param q_pos 位置权重矩阵
     * @param q_vel 速度权重矩阵
     */
    void setQ(const Eigen::Matrix3d& q_pos, const Eigen::Matrix3d& q_vel);

    /**
     * @brief 设置控制权重矩阵
     * @param r 加速度权重矩阵
     */
    void setR(const Eigen::Matrix3d& r);

    /**
     * @brief 设置离散时间步长
     * @param dt 离散时间步长
     */
    void setDt(double dt);

    /**
     * @brief 求解黎卡提方程，计算反馈增益矩阵K
     */
    void solveRiccati();

    /**
     * @brief 计算LQR控制输出
     * @param current 当前状态
     * @param reference 参考状态
     * @return ControlOutput 控制输出
     */
    ControlOutput computeControl(const State& current, const State& reference);

private:
    /**
     * @brief 初始化系统矩阵A和B
     */
    void initializeSystemMatrices();

    double dt_;

    Eigen::Matrix<double, 6, 6> A_;
    Eigen::Matrix<double, 6, 3> B_;
    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix3d R_;
    Eigen::Matrix<double, 3, 6> K_;

    bool is_initialized_;
};

}

#endif