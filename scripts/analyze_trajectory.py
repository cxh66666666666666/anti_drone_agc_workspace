#!/usr/bin/env python3
"""
分析固定种子生成的随机轨迹
输出安全的障碍物放置位置（确保不与轨迹碰撞）
"""
import numpy as np

def catmull_rom_spline(p0, p1, p2, p3, t):
    """Catmull-Rom样条插值"""
    t2 = t * t
    t3 = t2 * t
    result = 0.5 * (
        (2.0 * p1) +
        (-p0 + p2) * t +
        (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
        (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    )
    return result

def generate_random_trajectory_fixed(seed, num_waypoints, space_min_x, space_max_x,
                                     space_min_y, space_max_y, space_min_z, space_max_z,
                                     duration, dt):
    """使用固定种子生成随机轨迹（与C++版本相同算法）"""
    np.random.seed(seed)  # 使用numpy的随机种子

    waypoints = []
    for _ in range(num_waypoints):
        wp = np.array([
            np.random.uniform(space_min_x, space_max_x),
            np.random.uniform(space_min_y, space_max_y),
            np.random.uniform(space_min_z, space_max_z)
        ])
        waypoints.append(wp)

    waypoints = np.array(waypoints)

    smoothed_positions = []
    interpolation_points = 10

    for i in range(len(waypoints) - 1):
        p0 = waypoints[(i - 1) % len(waypoints)]
        p1 = waypoints[i]
        p2 = waypoints[(i + 1) % len(waypoints)]
        p3 = waypoints[(i + 2) % len(waypoints)]

        for j in range(interpolation_points):
            t = j / interpolation_points
            pos = catmull_rom_spline(p0, p1, p2, p3, t)
            smoothed_positions.append(pos)

    smoothed_positions = np.array(smoothed_positions)

    segment_distances = []
    total_distance = 0.0
    for i in range(len(smoothed_positions)):
        next_idx = (i + 1) % len(smoothed_positions)
        dist = np.linalg.norm(smoothed_positions[next_idx] - smoothed_positions[i])
        segment_distances.append(dist)
        total_distance += dist

    velocities = []
    for i in range(len(smoothed_positions)):
        next_idx = (i + 1) % len(smoothed_positions)
        prev_idx = (i - 1) % len(smoothed_positions)
        tangent = (smoothed_positions[next_idx] - smoothed_positions[prev_idx]) / 2.0
        velocities.append(tangent / np.linalg.norm(tangent) * 5.0)

    return smoothed_positions, velocities

def analyze_trajectory_and_suggest_obstacles(seed=42, num_waypoints=10,
                                             space_min_x=-10.0, space_max_x=10.0,
                                             space_min_y=-10.0, space_max_y=10.0,
                                             space_min_z=1.5, space_max_z=3.0):
    """分析轨迹并建议安全的障碍物位置"""

    trajectory, velocities = generate_random_trajectory_fixed(
        seed, num_waypoints,
        space_min_x, space_max_x,
        space_min_y, space_max_y,
        space_min_z, space_max_z,
        duration=30.0, dt=0.02
    )

    print("=" * 60)
    print("轨迹分析结果")
    print("=" * 60)

    print(f"\n轨迹点数量: {len(trajectory)}")
    print(f"\n轨迹空间范围:")
    print(f"  X: {trajectory[:, 0].min():.2f} ~ {trajectory[:, 0].max():.2f}")
    print(f"  Y: {trajectory[:, 1].min():.2f} ~ {trajectory[:, 1].max():.2f}")
    print(f"  Z: {trajectory[:, 2].min():.2f} ~ {trajectory[:, 2].max():.2f}")

    margin = 2.0  # 安全距离

    safe_regions = []

    print(f"\n{'='*60}")
    print("安全障碍物放置建议（距离轨迹至少{:.1f}m）".format(margin))
    print("="*60)

    print("\n--- 障碍物位置建议 ---")
    print("格式: obstacles.append((\"name\", spawn_xxx(...), x, y, z))")

    suggestions = []

    x_min, x_max = trajectory[:, 0].min(), trajectory[:, 0].max()
    y_min, y_max = trajectory[:, 1].min(), trajectory[:, 1].max()
    z_min, z_max = trajectory[:, 2].min(), trajectory[:, 2].max()

    potential_positions = [
        (x_max + margin, y_max + margin),
        (x_min - margin, y_min - margin),
        (x_max + margin, y_min - margin),
        (x_min - margin, y_max + margin),
        (x_max + margin, 0),
        (x_min - margin, 0),
        (0, y_max + margin),
        (0, y_min - margin),
    ]

    for x, y in potential_positions:
        if (space_min_x <= x <= space_max_x and
            space_min_y <= y <= space_max_y):
            z_center = (z_min + z_max) / 2
            suggestions.append((x, y, z_center))
            print(f"  ({x:.1f}, {y:.1f}, {z_center:.1f})")

    print(f"\n--- 生成障碍物代码 ---")
    print("obstacles = []")
    for i, (x, y, z) in enumerate(suggestions):
        print(f'obloacles.append(("obstacle_{i+1}", spawn_box({x:.1f}, {y:.1f}, {z:.1f}, 2, 2, 2, "obstacle_{i+1}"), {x:.1f}, {y:.1f}, 0))')

    return trajectory, suggestions

if __name__ == "__main__":
    trajectory, suggestions = analyze_trajectory_and_suggest_obstacles(
        seed=42,
        num_waypoints=10,
        space_min_x=-10.0,
        space_max_x=10.0,
        space_min_y=-10.0,
        space_max_y=10.0,
        space_min_z=1.5,
        space_max_z=3.0
    )
