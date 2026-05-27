#ifndef LQR_TRAJECTORY_TRACKER_LQR_CONTROLLER_H
#define LQR_TRAJECTORY_TRACKER_LQR_CONTROLLER_H

#include <Eigen/Dense>

namespace lqr_trajectory_tracker {

struct State {
    Eigen::Vector3d position;  // 位置 [x, y, z]
    Eigen::Vector3d velocity;  // 速度 [vx, vy, vz]

    State() : position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()) {}
};

struct ControlOutput {
    Eigen::Vector3d acceleration;
};

class LQRController {
public:
    LQRController();
    ~LQRController();

    void setQ(const Eigen::Matrix3d& q_pos, const Eigen::Matrix3d& q_vel);  // 设置状态权重矩阵
    void setR(const Eigen::Matrix3d& r);  // 设置控制权重矩阵
    void setDt(double dt);  // 设置离散时间步长

    bool solveRiccati();  // 求解黎卡提方程，计算反馈增益矩阵K

    ControlOutput computeControl(const State& current, const State& reference);  // 计算LQR控制输出
    
    Eigen::Matrix<double, 6, 6> getA() const { return A_; }
    Eigen::Matrix<double, 6, 3> getB() const { return B_; }
    Eigen::Matrix<double, 3, 6> getK() const { return K_; }

private:
    double dt_;  // 离散时间步长

    Eigen::Matrix<double, 6, 6> A_;  // 系统状态矩阵
    Eigen::Matrix<double, 6, 3> B_;  // 系统输入矩阵
    Eigen::Matrix<double, 6, 6> Q_;  // 状态权重矩阵
    Eigen::Matrix3d R_;  // 控制权重矩阵
    Eigen::Matrix<double, 3, 6> K_;  // LQR反馈增益矩阵

    bool is_initialized_;  // 初始化标志

    void initializeSystemMatrices();  // 初始化系统矩阵A和B
};

}

#endif
