#!/usr/bin/env python3
"""
测试轨迹生成器
可视化生成的轨迹
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def generate_linear_trajectory(start, end, duration, dt):
    """生成直线轨迹"""
    num_points = int(duration / dt) + 1
    trajectory = []
    
    direction = np.array(end) - np.array(start)
    distance = np.linalg.norm(direction)
    
    if distance < 1e-6:
        return [{
            'timestamp': 0,
            'position': np.array(start),
            'velocity': np.zeros(3),
            'acceleration': np.zeros(3)
        }]
    
    direction = direction / distance
    max_velocity = distance / (duration * 0.8)
    
    for i in range(num_points):
        t = i * dt
        s = t / duration
        
        # 速度曲线
        if s < 0.1:
            vel_profile = s / 0.1
        elif s > 0.9:
            vel_profile = (1.0 - s) / 0.1
        else:
            vel_profile = 1.0
        
        # 加速度曲线
        if s < 0.1:
            accel_profile = max_velocity / (0.1 * duration)
        elif s > 0.9:
            accel_profile = -max_velocity / (0.1 * duration)
        else:
            accel_profile = 0.0
        
        point = {
            'timestamp': t,
            'position': np.array(start) + s * (np.array(end) - np.array(start)),
            'velocity': direction * max_velocity * vel_profile,
            'acceleration': direction * accel_profile
        }
        trajectory.append(point)
    
    return trajectory


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


def generate_helical_trajectory(center, radius, height_start, height_end, 
                                angular_velocity, duration, dt):
    """生成螺旋轨迹"""
    num_points = int(duration / dt) + 1
    trajectory = []
    height_diff = height_end - height_start
    vertical_velocity = height_diff / duration
    
    for i in range(num_points):
        t = i * dt
        theta = angular_velocity * t
        
        point = {
            'timestamp': t,
            'position': np.array([
                center[0] + radius * np.cos(theta),
                center[1] + radius * np.sin(theta),
                height_start + vertical_velocity * t
            ]),
            'velocity': np.array([
                -radius * angular_velocity * np.sin(theta),
                radius * angular_velocity * np.cos(theta),
                vertical_velocity
            ]),
            'acceleration': np.array([
                -radius * angular_velocity**2 * np.cos(theta),
                -radius * angular_velocity**2 * np.sin(theta),
                0.0
            ])
        }
        trajectory.append(point)
    
    return trajectory


def visualize_trajectory(trajectory, title="Trajectory"):
    """可视化轨迹"""
    positions = np.array([p['position'] for p in trajectory])
    velocities = np.array([p['velocity'] for p in trajectory])
    accelerations = np.array([p['acceleration'] for p in trajectory])
    timestamps = np.array([p['timestamp'] for p in trajectory])
    
    fig = plt.figure(figsize=(15, 10))
    
    # 3D轨迹
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
    ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                c='g', marker='o', s=100, label='Start')
    ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                c='r', marker='s', s=100, label='End')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title(f'{title} - 3D Path')
    ax1.legend()
    ax1.grid(True)
    
    # 位置曲线
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(timestamps, positions[:, 0], 'r-', label='X')
    ax2.plot(timestamps, positions[:, 1], 'g-', label='Y')
    ax2.plot(timestamps, positions[:, 2], 'b-', label='Z')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # 速度曲线
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(timestamps, velocities[:, 0], 'r-', label='Vx')
    ax3.plot(timestamps, velocities[:, 1], 'g-', label='Vy')
    ax3.plot(timestamps, velocities[:, 2], 'b-', label='Vz')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Velocity [m/s]')
    ax3.set_title('Velocity vs Time')
    ax3.legend()
    ax3.grid(True)
    
    # 加速度曲线
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(timestamps, accelerations[:, 0], 'r-', label='Ax')
    ax4.plot(timestamps, accelerations[:, 1], 'g-', label='Ay')
    ax4.plot(timestamps, accelerations[:, 2], 'b-', label='Az')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Acceleration [m/s²]')
    ax4.set_title('Acceleration vs Time')
    ax4.legend()
    ax4.grid(True)
    
    # XY平面
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
    ax5.scatter(positions[0, 0], positions[0, 1], c='g', marker='o', s=100)
    ax5.scatter(positions[-1, 0], positions[-1, 1], c='r', marker='s', s=100)
    ax5.set_xlabel('X [m]')
    ax5.set_ylabel('Y [m]')
    ax5.set_title('XY Plane Projection')
    ax5.grid(True)
    ax5.axis('equal')
    
    # XZ平面
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=2)
    ax6.scatter(positions[0, 0], positions[0, 2], c='g', marker='o', s=100)
    ax6.scatter(positions[-1, 0], positions[-1, 2], c='r', marker='s', s=100)
    ax6.set_xlabel('X [m]')
    ax6.set_ylabel('Z [m]')
    ax6.set_title('XZ Plane Projection')
    ax6.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'/tmp/{title.lower().replace(" ", "_")}.png', dpi=150)
    plt.show()


def main():
    print("=" * 50)
    print("LQR轨迹生成器测试")
    print("=" * 50)
    
    duration = 10.0
    dt = 0.02
    
    # 测试直线轨迹
    print("\n1. 生成直线轨迹...")
    linear_traj = generate_linear_trajectory(
        start=[0, 0, 0],
        end=[10, 5, 3],
        duration=duration,
        dt=dt
    )
    print(f"   生成了 {len(linear_traj)} 个轨迹点")
    visualize_trajectory(linear_traj, "Linear Trajectory")
    
    # 测试圆形轨迹
    print("\n2. 生成圆形轨迹...")
    circular_traj = generate_circular_trajectory(
        center=[5, 0, 5],
        radius=5.0,
        height=5.0,
        angular_velocity=0.5,
        duration=duration,
        dt=dt
    )
    print(f"   生成了 {len(circular_traj)} 个轨迹点")
    visualize_trajectory(circular_traj, "Circular Trajectory")
    
    # 测试螺旋轨迹
    print("\n3. 生成螺旋轨迹...")
    helical_traj = generate_helical_trajectory(
        center=[5, 0, 0],
        radius=5.0,
        height_start=0.0,
        height_end=10.0,
        angular_velocity=0.5,
        duration=duration,
        dt=dt
    )
    print(f"   生成了 {len(helical_traj)} 个轨迹点")
    visualize_trajectory(helical_traj, "Helical Trajectory")
    
    print("\n" + "=" * 50)
    print("测试完成！")
    print("=" * 50)


if __name__ == "__main__":
    main()
