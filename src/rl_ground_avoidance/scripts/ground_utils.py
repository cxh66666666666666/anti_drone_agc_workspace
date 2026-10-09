#!/usr/bin/env python3
"""Pure helpers for the ground vehicle DQN environment (no ROS imports)."""

import math

SECTOR_COUNT = 12
LASER_MAX_RANGE = 10.0
LASER_MIN_RANGE = 0.1


def sectors_from_scan(ranges, angle_min, angle_increment, num_sectors=SECTOR_COUNT,
                      max_range=LASER_MAX_RANGE, min_range=LASER_MIN_RANGE):
    """Convert a LaserScan range array into normalized sector minima.

    Sector i covers angles [-pi + i*size, -pi + (i+1)*size) in the laser frame;
    a sector with no valid returns becomes 1.0 (far).
    """
    if num_sectors <= 0:
        raise ValueError("num_sectors must be positive")
    sector_size = 2.0 * math.pi / num_sectors
    minima = [float("inf")] * num_sectors
    for index, distance in enumerate(ranges):
        if distance is None or not math.isfinite(distance):
            continue
        if distance <= min_range or distance > max_range:
            continue
        angle = angle_min + index * angle_increment
        sector = int((angle + math.pi) / sector_size) % num_sectors
        if distance < minima[sector]:
            minima[sector] = distance
    return [min(1.0, value / max_range) if math.isfinite(value) else 1.0
            for value in minima]


def forward_clearance(sectors, half_angle=math.pi / 2.0):
    """Minimum normalized clearance inside the forward cone."""
    count = len(sectors)
    if not count:
        return 1.0
    sector_size = 2.0 * math.pi / count
    distances = [float(distance) for index, distance in enumerate(sectors)
                 if abs((index + 0.5) * sector_size - math.pi) <= half_angle]
    return min(distances) if distances else 1.0


def minimum_clearance(sectors):
    return min(float(distance) for distance in sectors) if sectors else 1.0


def reference_target(elapsed, speed, offset_x, offset_y):
    """Moving reference point on the straight line y=0 at `speed` m/s."""
    return (speed * elapsed + offset_x, offset_y)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    """Return ENU yaw from a quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def virtual_sectors(pose_x, pose_y, yaw, poles, num_sectors=SECTOR_COUNT,
                    max_range=LASER_MAX_RANGE, pole_radius=0.15):
    """Analytic 2D lidar: minimum surface distance to static poles per sector.

    Poles are circles of radius `pole_radius` in the xy plane; the returned
    distances are normalized by max_range. A sector without any pole becomes
    1.0 (far). When the robot center enters a pole, all sectors collapse to 0.
    """
    if num_sectors <= 0:
        raise ValueError("num_sectors must be positive")
    sector_size = 2.0 * math.pi / num_sectors
    minima = [float("inf")] * num_sectors
    for pole_x, pole_y in poles:
        dx = pole_x - pose_x
        dy = pole_y - pose_y
        center_distance = math.hypot(dx, dy)
        if center_distance <= pole_radius:
            return [0.0] * num_sectors
        surface = center_distance - pole_radius
        bearing = wrap_angle(math.atan2(dy, dx) - yaw)
        sector = int((bearing + math.pi) / sector_size) % num_sectors
        if surface < minima[sector]:
            minima[sector] = surface
    return [min(1.0, value / max_range) if math.isfinite(value) else 1.0
            for value in minima]


def diff_drive_command(pose_x, pose_y, yaw, target_x, target_y,
                       base_speed=1.0, max_angular=2.5, position_error=0.0):
    """Steer a differential drive robot toward a target point.

    Returns (linear_x, angular_z). Speed is reduced while turning hard and
    slightly increased while trailing behind the target (catch-up term).
    """
    heading_error = wrap_angle(
        math.atan2(target_y - pose_y, target_x - pose_x) - yaw)
    angular = max(-max_angular, min(max_angular, 3.0 * heading_error))
    linear = base_speed * max(0.15, 1.0 - abs(heading_error) / 2.0)
    linear += 0.4 * min(max(position_error, 0.0), 1.0)
    return linear, angular


def tracking_error(pose_x, pose_y, target_x, target_y):
    """Euclidean distance between the robot and its moving target."""
    return math.hypot(target_x - pose_x, target_y - pose_y)


def integrate_pose(pose_x, pose_y, yaw, linear, angular, dt):
    """Kinematic unicycle integration: advance the pose by one dt.

    Returns (new_x, new_y, new_yaw).
    """
    new_yaw = wrap_angle(yaw + angular * dt)
    new_x = pose_x + linear * math.cos(new_yaw) * dt
    new_y = pose_y + linear * math.sin(new_yaw) * dt
    return new_x, new_y, new_yaw
