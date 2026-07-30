#!/usr/bin/env python3
"""Pure helpers shared by the DQN environment and training loop."""

import math


def epsilon_at_step(start, end, decay_steps, step):
    """Exponentially decay epsilon from start to end over decay_steps."""
    if not 0.0 <= end <= start <= 1.0:
        raise ValueError("epsilon must satisfy 0 <= end <= start <= 1")
    if decay_steps <= 0:
        raise ValueError("decay_steps must be positive")
    if step <= 0:
        return start
    if step >= decay_steps or start == end:
        return end
    return start * math.pow(end / start, float(step) / decay_steps)


def action_change_cost(previous_action, current_action, penalty):
    """Return the configured cost when consecutive actions differ."""
    return penalty if previous_action != current_action else 0.0


def avoidance_magnitude(distance, safe_distance, minimum, maximum):
    """Scale a lateral offset using the rule baseline's distance response."""
    if safe_distance <= 0.0:
        raise ValueError("safe_distance must be positive")
    if minimum < 0.0 or maximum < minimum:
        raise ValueError("offset limits must satisfy 0 <= minimum <= maximum")
    severity = 1.0 - max(0.0, min(distance, safe_distance)) / safe_distance
    return minimum + severity * (maximum - minimum)


def smooth_value(previous, desired, alpha):
    """Apply the same first-order offset filter as the rule baseline."""
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha must satisfy 0 < alpha <= 1")
    return (1.0 - alpha) * previous + alpha * desired


def limit_change(previous, desired, maximum_change):
    """Limit how far a command may move during one environment step."""
    if maximum_change <= 0.0:
        raise ValueError("maximum_change must be positive")
    return max(previous - maximum_change,
               min(previous + maximum_change, desired))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x, y, z, w):
    """Return ENU yaw from a quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def motion_heading(velocity_x, velocity_y, fallback_heading, minimum_speed=0.1):
    """Use horizontal velocity as forward, retaining the last heading at low speed."""
    if math.hypot(velocity_x, velocity_y) >= minimum_speed:
        return math.atan2(velocity_y, velocity_x)
    return fallback_heading


def rotate_sectors_to_heading(sectors, body_yaw, heading_world):
    """Re-index body-frame sectors so zero angle follows horizontal motion."""
    if not sectors:
        return []
    count = len(sectors)
    sector_size = 2.0 * math.pi / count
    heading_body = wrap_angle(heading_world - body_yaw)
    rotated = []
    for index in range(count):
        relative_angle = (index + 0.5) * sector_size - math.pi
        body_angle = wrap_angle(relative_angle + heading_body)
        source = int((body_angle + math.pi) / sector_size) % count
        rotated.append(float(sectors[source]))
    return rotated


def motion_frame_offset(forward, left, heading_world):
    """Convert forward/left components into ENU map-frame XY components."""
    cosine = math.cos(heading_world)
    sine = math.sin(heading_world)
    return (forward * cosine - left * sine,
            forward * sine + left * cosine)


def directional_clearance(sectors, half_angle=math.pi / 2.0):
    """Return minimum clearance inside a motion-frame forward cone."""
    if not sectors:
        return 1.0
    count = len(sectors)
    sector_size = 2.0 * math.pi / count
    distances = [float(distance) for index, distance in enumerate(sectors)
                 if abs((index + 0.5) * sector_size - math.pi) <= half_angle]
    return min(distances) if distances else 1.0


def proximity_penalty(clearance, collision_distance, emergency_distance,
                      safe_distance, safe_scale=0.1, emergency_scale=0.5):
    """Bounded dense penalty that does not make prolonged success worse than collision."""
    if clearance >= safe_distance:
        return 0.0
    if clearance < emergency_distance:
        span = max(emergency_distance - collision_distance, 1e-6)
        severity = min(1.0, max(0.0,
                               (emergency_distance - clearance) / span))
        return safe_scale + severity * (emergency_scale - safe_scale)
    span = max(safe_distance - emergency_distance, 1e-6)
    severity = (safe_distance - clearance) / span
    return safe_scale * min(1.0, max(0.0, severity))


def constrain_avoidance_action(action, locked_side, clearance, safe_distance):
    """Keep one lateral side until the obstacle has left the forward safety zone."""
    if action not in range(5):
        return 0, locked_side
    if clearance >= safe_distance:
        locked_side = 0
    side = 1 if action in (1, 3) else (-1 if action in (2, 4) else 0)
    if side and locked_side == 0:
        locked_side = side
    elif side and side != locked_side:
        if locked_side > 0:
            action = 3 if action in (3, 4) else 1
        else:
            action = 4 if action in (3, 4) else 2
    return action, locked_side


def update_avoidance_progress(encountered, clear_steps, obstacle_distance,
                              tracking_error, safe_distance,
                              tracking_tolerance, required_clear_steps):
    """Advance the encounter-then-clear success state machine by one step."""
    if required_clear_steps <= 0:
        raise ValueError("required_clear_steps must be positive")
    encountered = encountered or obstacle_distance < safe_distance
    safely_clear = (encountered and obstacle_distance >= safe_distance and
                    tracking_error <= tracking_tolerance)
    clear_steps = clear_steps + 1 if safely_clear else 0
    return encountered, clear_steps, clear_steps >= required_clear_steps


def episode_succeeded(done_reason):
    """Only an explicit completed avoidance manoeuvre counts as success."""
    return done_reason == "avoidance_complete"


def validate_distance_thresholds(min_range, collision_distance,
                                 emergency_distance, safe_distance):
    """Validate physical distance thresholds used by perception and the environment."""
    if min_range < 0.0:
        raise ValueError("lidar min_range must be non-negative")
    if collision_distance <= min_range:
        raise ValueError(
            "collision_distance must be greater than lidar min_range so collision "
            "observations are reachable")
    if not collision_distance < emergency_distance < safe_distance:
        raise ValueError(
            "distance thresholds must satisfy collision < emergency < safe")
