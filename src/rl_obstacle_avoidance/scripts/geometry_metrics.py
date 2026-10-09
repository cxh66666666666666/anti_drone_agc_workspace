#!/usr/bin/env python3
"""纯几何碰撞代理（不依赖 ROS/Gazebo）。

障碍物使用 Gazebo spawn_model 当前采用的轴对齐 box 近似；无人机使用
保守的轴对齐包络。返回值是“无人机包络到障碍物表面”的有符号净空：
正值表示分离，零表示相切，负值表示包络重叠。该量是几何代理，不是
Gazebo 物理接触事件。
"""

import math


DEFAULT_OBSTACLE_SIZE = (0.3, 0.3, 3.0)
DEFAULT_DRONE_HALF_EXTENTS = (0.35, 0.35, 0.06)


def obstacle_surface_clearance(point, obstacle,
                               drone_half_extents=DEFAULT_DRONE_HALF_EXTENTS):
    """Return clearance to a scenario obstacle dict or legacy tuple.

    Walls/boxes are evaluated in their local yaw frame. Cylinders use a
    conservative circular horizontal envelope, which is exact for the
    supported upright cylinders and remains compatible with old tuple input.
    """
    if isinstance(obstacle, dict):
        center = obstacle["center"]
        size = obstacle["size"]
        obstacle_type = obstacle.get("type", "pillar_box")
        yaw = float(obstacle.get("yaw", 0.0))
    else:
        center, size = obstacle
        obstacle_type, yaw = "pillar_box", 0.0
    px, py, pz = (float(value) for value in point)
    cx, cy, cz = (float(value) for value in center)
    if obstacle_type == "cylinder":
        radius = 0.5 * max(float(size[0]), float(size[1]))
        horizontal = math.hypot(px - cx, py - cy) - radius - math.hypot(
            float(drone_half_extents[0]), float(drone_half_extents[1]))
        vertical = abs(pz - cz) - 0.5 * float(size[2]) - float(drone_half_extents[2])
        if horizontal > 0.0 or vertical > 0.0:
            return math.hypot(max(horizontal, 0.0), max(vertical, 0.0))
        return max(horizontal, vertical)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    dx, dy = px - cx, py - cy
    local_point = (cos_yaw * dx + sin_yaw * dy,
                   -sin_yaw * dx + cos_yaw * dy, pz - cz)
    return box_surface_clearance(local_point, (0.0, 0.0, 0.0), size,
                                 drone_half_extents)


def box_surface_clearance(point, box_center, box_size,
                          drone_half_extents=DEFAULT_DRONE_HALF_EXTENTS):
    """Return signed clearance from a point envelope to an axis-aligned box."""
    expanded = tuple(0.5 * float(size) + float(body)
                     for size, body in zip(box_size, drone_half_extents))
    deltas = tuple(abs(float(coord) - float(center))
                   for coord, center in zip(point, box_center))
    outside = tuple(max(delta - half, 0.0)
                    for delta, half in zip(deltas, expanded))
    if any(value > 0.0 for value in outside):
        return math.sqrt(sum(value * value for value in outside))
    # The envelope overlaps the expanded box.  Report signed penetration
    # rather than clipping at zero so minimum-clearance plots remain useful.
    return -min(half - delta for half, delta in zip(expanded, deltas))


def minimum_geometry_clearance(point, obstacles,
                               drone_half_extents=DEFAULT_DRONE_HALF_EXTENTS):
    """Return the minimum signed clearance, or ``math.inf`` if empty.

    ``obstacles`` contains ``(center_xyz, size_xyz)`` pairs.
    """
    if not obstacles:
        return math.inf
    return min(obstacle_surface_clearance(point, obstacle, drone_half_extents)
               for obstacle in obstacles)
