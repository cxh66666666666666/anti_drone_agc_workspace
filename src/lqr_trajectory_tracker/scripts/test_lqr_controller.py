#!/usr/bin/env python3
"""
测试LQR控制器
模拟轨迹跟踪过程
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def solve_riccati(A, B, Q, R, max_iter=1000, tol=1e-6):
    """求解离散代数Riccati方程"""
    P = Q.copy()
    
    for i in range(max_iter):
        P_new = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
        
        if np.linalg.norm(P_new - P) < tol:
            P = P_new
            break
        P = P_new
    
    K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
    return K


def generate_circular_trajectory(center, radius, height, angular_velocity, duration, dt):
    """生成圆形轨迹"""
    num_points = int(duration / dt) + 1
    trajectory = []
    
    for i in range(num_points):
        t = i * dt
        theta = angular_velocity * t
        
        point = {
            'timestamp': t,
            'position': np.array([
                center[0] + radius * np.cos(theta),
                center[1] + radius * np.sin(theta),
                height
            ]),
            'velocity': np.array([
                -radius * angular_velocity * np.sin(theta),
                radius * angular_velocity * np.cos(theta),
                0.0
            ]),
            'acceleration': np.array([
                -radius * angular_velocity**2 * np.cos(theta),
                -radius * angular_velocity**2 * np.sin(theta),
                0.0
            ])
        }
        trajectory.append(point)
    
    return trajectory


def simulate_lqr_tracking(trajectory, dt, noise_std=0.01):
    """模拟LQR轨迹跟踪"""
    # 系统矩阵 (双积分器模型)
    A = np.eye(6)
    A[0:3, 3:6] = np.eye(3) * dt
    
    B = np.zeros((6, 3))
    B[3:6, 0:3] = np.eye(3) * dt
    
    # LQR权重矩阵
    Q = np.eye(6)
    Q[0:3, 0:3] *= 10.0  # 位置权重
    Q[3:6, 3:6] *= 1.0   # 速度权重
    
    R = np.eye(3) * 0.1  # 控制权重
    
    # 求解LQR增益
    K = solve_riccati(A, B, Q, R)
    print(f"LQR增益矩阵 K:\n{K}")
    
    # 初始状态 (有偏差)
    x = np.zeros(6)
    x[0:3] = trajectory[0]['position'] + np.array([1.0, -0.5, 0.3])  # 初始位置偏差
    x[3:6] = np.array([0.1, 0.1, 0.0])  # 初始速度
    
    # 记录数据
    actual_positions = []
    actual_velocities = []
    ref_positions = []
    ref_velocities = []
    controls = []
    errors = []
    timestamps = []
    
    for i, ref_point in enumerate(trajectory):
        t = ref_point['timestamp']
        x_ref = np.zeros(6)
        x_ref[0:3] = ref_point['position']
        x_ref[3:6] = ref_point['velocity']
        
        # 计算误差
        error = x - x_ref
        
        # LQR控制律 u = -K * error
        u = -K @ error
        
        # 限制控制量
        u = np.clip(u, -10.0, 10.0)
        
        # 添加测量噪声
        noise = np.random.normal(0, noise_std, 6)
        x_measured = x + noise
        
        # 记录数据
        timestamps.append(t)
        actual_positions.append(x_measured[0:3].copy())
        actual_velocities.append(x_measured[3:6].copy())
        ref_positions.append(ref_point['position'].copy())
        ref_velocities.append(ref_point['velocity'].copy())
        controls.append(u.copy())
        errors.append(np.linalg.norm(error[0:3]))
        
        # 状态更新 (双积分器模型)
        x[0:3] = x[0:3] + x[3:6] * dt + 0.5 * u * dt**2
        x[3:6] = x[3:6] + u * dt
    
    return {
        'timestamps': np.array(timestamps),
        'actual_positions': np.array(actual_positions),
        'actual_velocities': np.array(actual_velocities),
        'ref_positions': np.array(ref_positions),
        'ref_velocities': np.array(ref_velocities),
        'controls': np.array(controls),
        'errors': np.array(errors)
    }


def visualize_tracking_results(results):
    """可视化跟踪结果"""
    timestamps = results['timestamps']
    actual_pos = results['actual_positions']
    ref_pos = results['ref_positions']
    actual_vel = results['actual_velocities']
    ref_vel = results['ref_velocities']
    controls = results['controls']
    errors = results['errors']
    
    fig = plt.figure(figsize=(16, 12))
    
    # 3D轨迹对比
    ax1 = fig.add_subplot(3, 3, 1, projection='3d')
    ax1.plot(ref_pos[:, 0], ref_pos[:, 1], ref_pos[:, 2], 
             'b--', linewidth=2, label='Reference')
    ax1.plot(actual_pos[:, 0], actual_pos[:, 1], actual_pos[:, 2], 
             'r-', linewidth=1, label='Actual')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Trajectory Tracking')
    ax1.legend()
    ax1.grid(True)
    
    # 位置跟踪误差
    ax2 = fig.add_subplot(3, 3, 2)
    ax2.plot(timestamps, actual_pos[:, 0] - ref_pos[:, 0], 'r-', label='X error')
    ax2.plot(timestamps, actual_pos[:, 1] - ref_pos[:, 1], 'g-', label='Y error')
    ax2.plot(timestamps, actual_pos[:, 2] - ref_pos[:, 2], 'b-', label='Z error')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position Error [m]')
    ax2.set_title('Position Tracking Error')
    ax2.legend()
    ax2.grid(True)
    
    # 位置误差范数
    ax3 = fig.add_subplot(3, 3, 3)
    ax3.plot(timestamps, errors, 'm-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Position Error Norm [m]')
    ax3.set_title('Total Position Error')
    ax3.grid(True)
    
    # X位置对比
    ax4 = fig.add_subplot(3, 3, 4)
    ax4.plot(timestamps, ref_pos[:, 0], 'b--', linewidth=2, label='Reference')
    ax4.plot(timestamps, actual_pos[:, 0], 'r-', linewidth=1, label='Actual')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('X Position [m]')
    ax4.set_title('X Position Tracking')
    ax4.legend()
    ax4.grid(True)
    
    # Y位置对比
    ax5 = fig.add_subplot(3, 3, 5)
    ax5.plot(timestamps, ref_pos[:, 1], 'b--', linewidth=2, label='Reference')
    ax5.plot(timestamps, actual_pos[:, 1], 'r-', linewidth=1, label='Actual')
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Y Position [m]')
    ax5.set_title('Y Position Tracking')
    ax5.legend()
    ax5.grid(True)
    
    # Z位置对比
    ax6 = fig.add_subplot(3, 3, 6)
    ax6.plot(timestamps, ref_pos[:, 2], 'b--', linewidth=2, label='Reference')
    ax6.plot(timestamps, actual_pos[:, 2], 'r-', linewidth=1, label='Actual')
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Z Position [m]')
    ax6.set_title('Z Position Tracking')
    ax6.legend()
    ax6.grid(True)
    
    # 速度对比
    ax7 = fig.add_subplot(3, 3, 7)
    ax7.plot(timestamps, ref_vel[:, 0], 'b--', linewidth=2, label='Ref Vx')
    ax7.plot(timestamps, actual_vel[:, 0], 'r-', linewidth=1, label='Actual Vx')
    ax7.plot(timestamps, ref_vel[:, 1], 'c--', linewidth=2, label='Ref Vy')
    ax7.plot(timestamps, actual_vel[:, 1], 'm-', linewidth=1, label='Actual Vy')
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('Velocity [m/s]')
    ax7.set_title('Velocity Tracking (X & Y)')
    ax7.legend()
    ax7.grid(True)
    
    # 控制输出
    ax8 = fig.add_subplot(3, 3, 8)
    ax8.plot(timestamps, controls[:, 0], 'r-', label='Ax')
    ax8.plot(timestamps, controls[:, 1], 'g-', label='Ay')
    ax8.plot(timestamps, controls[:, 2], 'b-', label='Az')
    ax8.set_xlabel('Time [s]')
    ax8.set_ylabel('Acceleration [m/s²]')
    ax8.set_title('Control Output')
    ax8.legend()
    ax8.grid(True)
    
    # XY平面轨迹
    ax9 = fig.add_subplot(3, 3, 9)
    ax9.plot(ref_pos[:, 0], ref_pos[:, 1], 'b--', linewidth=2, label='Reference')
    ax9.plot(actual_pos[:, 0], actual_pos[:, 1], 'r-', linewidth=1, label='Actual')
    ax9.set_xlabel('X [m]')
    ax9.set_ylabel('Y [m]')
    ax9.set_title('XY Plane Trajectory')
    ax9.legend()
    ax9.grid(True)
    ax9.axis('equal')
    
    plt.tight_layout()
    plt.savefig('/tmp/lqr_tracking_results.png', dpi=150)
    plt.show()
    
    # 打印统计信息
    print("\n" + "=" * 50)
    print("LQR轨迹跟踪统计")
    print("=" * 50)
    print(f"最大位置误差: {np.max(errors):.4f} m")
    print(f"平均位置误差: {np.mean(errors):.4f} m")
    print(f"最终位置误差: {errors[-1]:.4f} m")
    print(f"最大控制输出: {np.max(np.abs(controls)):.4f} m/s²")
    print(f"平均控制输出: {np.mean(np.abs(controls)):.4f} m/s²")


def main():
    print("=" * 50)
    print("LQR控制器测试")
    print("=" * 50)
    
    # 参数设置
    duration = 10.0
    dt = 0.02
    
    # 生成圆形轨迹
    print("\n生成参考轨迹...")
    trajectory = generate_circular_trajectory(
        center=[5, 0, 5],
        radius=5.0,
        height=5.0,
        angular_velocity=0.5,
        duration=duration,
        dt=dt
    )
    print(f"轨迹点数: {len(trajectory)}")
    
    # 模拟LQR跟踪
    print("\n模拟LQR轨迹跟踪...")
    np.random.seed(42)  # 设置随机种子以便复现
    results = simulate_lqr_tracking(trajectory, dt, noise_std=0.02)
    
    # 可视化结果
    print("\n生成可视化图表...")
    visualize_tracking_results(results)
    
    print("\n" + "=" * 50)
    print("测试完成！")
    print("=" * 50)


if __name__ == "__main__":
    main()
