#!/usr/bin/env python3
"""A* 路径规划纯函数模块（无 ROS 依赖，可独立测试）。

障碍物以 (x, y) 圆柱列表或带 type/size/yaw 的场景字典表示，规划在 2D 栅格上进行：
- build_grid：栅格化 + 膨胀（柱半径 + 安全余量）
- astar_path：8 邻域 A*
- smooth_path：视线法简化（逐段碰撞复查）
- is_goal_valid：目标点合法性检查
"""

import heapq
import math


def build_grid(obstacles, bounds, resolution, inflation_radius):
    """Return the set of blocked grid cells.

    Args:
        obstacles: list of (x, y) obstacle centers.
        bounds: (xmin, xmax, ymin, ymax) world bounds.
        resolution: grid cell size in meters.
        inflation_radius: obstacle inflation radius (pole radius + margin).

    Returns:
        set of (ix, iy) blocked cells.
    """
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    blocked = set()
    xmin, xmax, ymin, ymax = bounds
    cells_radius = int(math.ceil(inflation_radius / resolution))
    for ox, oy in obstacles:
        cx, cy = world_to_grid(ox, oy, bounds, resolution)
        for dx in range(-cells_radius, cells_radius + 1):
            for dy in range(-cells_radius, cells_radius + 1):
                wx, wy = grid_to_world(cx + dx, cy + dy, bounds, resolution)
                if math.hypot(wx - ox, wy - oy) <= inflation_radius:
                    blocked.add((cx + dx, cy + dy))
    return blocked


def obstacle_clearance_2d(point, obstacle):
    """Return horizontal clearance to a typed obstacle footprint.

    A legacy ``(x, y)`` item is treated as a point, preserving the original
    planner semantics.  Typed walls/boxes use their rotated rectangle and
    cylinders use their radius.
    """
    px, py = float(point[0]), float(point[1])
    if isinstance(obstacle, dict):
        center = obstacle["center"]
        size = obstacle["size"]
        cx, cy = float(center[0]), float(center[1])
        dx, dy = px - cx, py - cy
        if obstacle.get("type", "pillar_box") == "cylinder":
            return math.hypot(dx, dy) - 0.5 * max(float(size[0]), float(size[1]))
        yaw = float(obstacle.get("yaw", 0.0))
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        outside_x = max(abs(local_x) - 0.5 * float(size[0]), 0.0)
        outside_y = max(abs(local_y) - 0.5 * float(size[1]), 0.0)
        if outside_x or outside_y:
            return math.hypot(outside_x, outside_y)
        return -min(0.5 * float(size[0]) - abs(local_x),
                    0.5 * float(size[1]) - abs(local_y))
    return math.hypot(px - float(obstacle[0]), py - float(obstacle[1]))


def build_typed_grid(obstacles, bounds, resolution, inflation_radius):
    """Rasterise typed scenario footprints with a uniform safety margin."""
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    if inflation_radius < 0:
        raise ValueError("inflation_radius must be non-negative")
    blocked = set()
    xmin, xmax, ymin, ymax = (float(value) for value in bounds)
    for obstacle in obstacles:
        if isinstance(obstacle, dict):
            center = obstacle["center"]
            size = obstacle["size"]
            radius = (0.5 * max(float(size[0]), float(size[1]))
                      if obstacle.get("type", "pillar_box") == "cylinder"
                      else 0.5 * math.hypot(float(size[0]), float(size[1])))
            cx, cy = float(center[0]), float(center[1])
        else:
            cx, cy = float(obstacle[0]), float(obstacle[1])
            radius = 0.0
        cells_radius = int(math.ceil((radius + inflation_radius) / resolution)) + 1
        gx, gy = world_to_grid(cx, cy, bounds, resolution)
        for ix in range(gx - cells_radius, gx + cells_radius + 1):
            for iy in range(gy - cells_radius, gy + cells_radius + 1):
                wx, wy = grid_to_world(ix, iy, bounds, resolution)
                if not (xmin <= wx <= xmax and ymin <= wy <= ymax):
                    continue
                if obstacle_clearance_2d((wx, wy), obstacle) <= inflation_radius:
                    blocked.add((ix, iy))
    return blocked


def typed_goal_valid(point, obstacles, min_clearance):
    """Check a goal against actual typed footprints plus ``min_clearance``."""
    return all(obstacle_clearance_2d(point, obstacle) >= float(min_clearance)
               for obstacle in obstacles)


def world_to_grid(x, y, bounds, resolution):
    xmin, xmax, ymin, ymax = bounds
    return (int(round((x - xmin) / resolution)),
            int(round((y - ymin) / resolution)))


def grid_to_world(ix, iy, bounds, resolution):
    xmin, xmax, ymin, ymax = bounds
    return (xmin + ix * resolution, ymin + iy * resolution)


def _grid_in_bounds(ix, iy, bounds, resolution):
    xmin, xmax, ymin, ymax = bounds
    return (xmin <= xmin + ix * resolution <= xmax and
            ymin <= ymin + iy * resolution <= ymax)


def nearest_free_point(blocked, point, bounds, resolution):
    """Return the nearest free grid point, or None when none is reachable."""
    start_cell = world_to_grid(point[0], point[1], bounds, resolution)
    if not _grid_in_bounds(start_cell[0], start_cell[1], bounds, resolution):
        return None
    if start_cell not in blocked:
        return point

    frontier = [(0.0, start_cell)]
    visited = set()
    while frontier:
        _, current = heapq.heappop(frontier)
        if current in visited:
            continue
        visited.add(current)
        if current not in blocked:
            return grid_to_world(current[0], current[1], bounds, resolution)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbor = (current[0] + dx, current[1] + dy)
                if (neighbor not in visited and
                        _grid_in_bounds(neighbor[0], neighbor[1],
                                        bounds, resolution)):
                    distance = math.hypot(neighbor[0] - start_cell[0],
                                          neighbor[1] - start_cell[1])
                    heapq.heappush(frontier, (distance, neighbor))
    return None


def astar_path(blocked, start, goal, bounds, resolution):
    """8-connected A* from start to goal.

    Args:
        blocked: set of blocked grid cells.
        start/goal: world (x, y) points.
        bounds, resolution: grid definition.

    Returns:
        list of world-space waypoints from start to goal, or [] if unreachable.
    """
    start_cell = world_to_grid(start[0], start[1], bounds, resolution)
    goal_cell = world_to_grid(goal[0], goal[1], bounds, resolution)
    if start_cell in blocked or goal_cell in blocked:
        return []

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap = [(heuristic(start_cell, goal_cell), 0, start_cell)]
    came_from = {}
    cost_so_far = {start_cell: 0.0}
    visited = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal_cell:
            path_cells = [current]
            while current in came_from:
                current = came_from[current]
                path_cells.append(current)
            path_cells.reverse()
            return [grid_to_world(ix, iy, bounds, resolution)
                    for ix, iy in path_cells]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbor = (current[0] + dx, current[1] + dy)
                if neighbor in blocked:
                    continue
                if not _grid_in_bounds(neighbor[0], neighbor[1],
                                       bounds, resolution):
                    continue
                step = math.hypot(dx, dy)
                new_cost = cost_so_far[current] + step
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal_cell)
                    heapq.heappush(open_heap, (priority, new_cost, neighbor))
                    came_from[neighbor] = current
    return []


def segment_blocked(p1, p2, blocked, bounds, resolution):
    """Check whether the straight segment p1→p2 passes through blocked cells."""
    x1, y1 = p1
    x2, y2 = p2
    length = math.hypot(x2 - x1, y2 - y1)
    samples = max(2, int(math.ceil(length / (resolution * 0.5))))
    for i in range(1, samples):
        t = float(i) / samples
        cell = world_to_grid(x1 + t * (x2 - x1), y1 + t * (y2 - y1),
                             bounds, resolution)
        if cell in blocked:
            return True
    return False


def smooth_path(path, blocked, bounds, resolution):
    """Greedy line-of-sight smoothing: keep waypoints whose segment is clear."""
    if len(path) <= 2:
        return path
    smoothed = [path[0]]
    anchor = 0
    index = 1
    while anchor < len(path) - 1:
        if (index >= len(path) or
                segment_blocked(path[anchor], path[index], blocked,
                                bounds, resolution)):
            smoothed.append(path[index - 1] if index > anchor + 1
                            else path[index])
            anchor = index - 1 if index > anchor + 1 else index
            index = anchor + 1
            continue
        if index == len(path) - 1:
            smoothed.append(path[index])
            break
        index += 1
    # 去重相邻点
    result = [smoothed[0]]
    for point in smoothed[1:]:
        if math.hypot(point[0] - result[-1][0],
                      point[1] - result[-1][1]) > 1e-6:
            result.append(point)
    return result


def anchor_path_start(path, start):
    """Return a path whose first waypoint is the exact requested start."""
    if not path:
        return []
    anchored = list(path)
    if math.hypot(anchored[0][0] - start[0],
                  anchored[0][1] - start[1]) <= 1e-9:
        anchored[0] = start
    else:
        anchored.insert(0, start)
    return anchored


def is_goal_valid(goal, obstacles, min_clearance):
    """A goal is valid when it keeps at least min_clearance from every obstacle."""
    for ox, oy in obstacles:
        if math.hypot(goal[0] - ox, goal[1] - oy) < min_clearance:
            return False
    return True
