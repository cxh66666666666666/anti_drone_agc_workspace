#!/usr/bin/env python3
"""Pure helpers shared by the DQN environment and training loop."""

import math
from collections import deque


class NStepAccumulator:
    """Build discounted n-step transitions without changing replay storage."""

    def __init__(self, n_steps, gamma):
        if n_steps <= 0 or not 0.0 < gamma <= 1.0:
            raise ValueError("n_steps and gamma must be positive")
        self.n_steps = n_steps
        self.gamma = gamma
        self.pending = deque()

    def append(self, transition):
        """Append (s,a,r,s2,done,allowed,hint), returning ready transitions."""
        self.pending.append(transition)
        ready = []
        if len(self.pending) >= self.n_steps:
            ready.append(self._pop_left(self.n_steps))
        if transition[4]:
            while self.pending:
                ready.append(self._pop_left(len(self.pending)))
        return ready

    def _pop_left(self, length):
        items = list(self.pending)[:length]
        reward = sum((self.gamma ** index) * item[2]
                     for index, item in enumerate(items))
        first = items[0]
        last = items[-1]
        self.pending.popleft()
        return (first[0], first[1], reward, last[3], last[4], last[5],
                self.gamma ** length, max(item[6] for item in items))


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
    """Rotate body sectors into the motion frame without losing edge obstacles.

    A rotated sector generally overlaps two source sectors.  Taking both
    minima is deliberately conservative: point-sampling only the sector
    centre can make a close obstacle disappear when a heading crosses a bin
    boundary.
    """
    if not sectors:
        return []
    count = len(sectors)
    sector_size = 2.0 * math.pi / count
    heading_body = wrap_angle(heading_world - body_yaw)
    rotated = []
    for index in range(count):
        relative_angle = (index + 0.5) * sector_size - math.pi
        body_center = wrap_angle(relative_angle + heading_body)
        overlapping = []
        for source, distance in enumerate(sectors):
            source_center = (source + 0.5) * sector_size - math.pi
            if abs(wrap_angle(source_center - body_center)) < sector_size - 1e-12:
                overlapping.append(float(distance))
        rotated.append(min(overlapping))
    return rotated


def swept_heading_sectors(sectors, body_yaw, start_heading, end_heading,
                          max_heading_step=None):
    """Combine clearances across the headings swept by a curved path."""
    if not sectors:
        return []
    sector_size = 2.0 * math.pi / len(sectors)
    step = sector_size * 0.5 if max_heading_step is None else max_heading_step
    if step <= 0.0:
        raise ValueError("max_heading_step must be positive")
    delta = wrap_angle(end_heading - start_heading)
    samples = max(1, int(math.ceil(abs(delta) / step)))
    swept = [float("inf")] * len(sectors)
    for index in range(samples + 1):
        heading = start_heading + delta * index / samples
        rotated = rotate_sectors_to_heading(sectors, body_yaw, heading)
        swept = [min(old, current)
                 for old, current in zip(swept, rotated)]
    return swept


def path_swept_sectors(sectors, body_yaw, headings):
    """Combine sector clearances over every segment of a path heading list."""
    if not sectors:
        return []
    if not headings:
        raise ValueError("headings must not be empty")
    if len(headings) == 1:
        return rotate_sectors_to_heading(sectors, body_yaw, headings[0])
    swept = [float("inf")] * len(sectors)
    for start, end in zip(headings, headings[1:]):
        segment = swept_heading_sectors(sectors, body_yaw, start, end)
        swept = [min(old, current)
                 for old, current in zip(swept, segment)]
    return swept


def retain_obstacle_sectors(current, remembered, memory_age, timeout):
    """Keep a detected obstacle only inside a finite sensor-dropout window."""
    current = list(current) if current else []
    if (not remembered or memory_age is None or memory_age < 0.0 or
            memory_age > timeout):
        return current
    remembered = list(remembered)
    if not current:
        return remembered
    if len(current) != len(remembered):
        raise ValueError("current and remembered sectors must have equal length")
    return [min(value, old_value)
            for value, old_value in zip(current, remembered)]


def swept_forward_clearance(sectors, body_yaw, headings,
                            half_angle=math.pi / 3.0):
    """Return the minimum forward-cone clearance over a curved path."""
    if not sectors:
        return 1.0
    if not headings:
        raise ValueError("headings must not be empty")
    return min(directional_clearance(
        rotate_sectors_to_heading(sectors, body_yaw, heading), half_angle)
        for heading in headings)


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


def lateral_clearances(sectors, front_deadband=math.pi / 6.0):
    """Return left/right half-plane clearance outside the forward deadband."""
    if not sectors:
        return 1.0, 1.0
    count = len(sectors)
    sector_size = 2.0 * math.pi / count
    left = []
    right = []
    for index, distance in enumerate(sectors):
        angle = (index + 0.5) * sector_size - math.pi
        if angle >= front_deadband:
            left.append(float(distance))
        elif angle <= -front_deadband:
            right.append(float(distance))
    return (min(left) if left else 1.0,
            min(right) if right else 1.0)


def cone_min_clearance(sectors, center_angle, half_angle=math.pi / 6.0):
    """返回指定角度锥内的最小归一化净空。

    角度约定与 directional_clearance / lateral_clearances 完全一致：
    扇区 i 的方位角中心为 (i + 0.5) * 2*pi/count - pi，0.0 为正前方。
    锥内没有扇区或 sectors 为空时返回 1.0（视为全清）。
    """
    if not sectors:
        return 1.0
    count = len(sectors)
    sector_size = 2.0 * math.pi / count
    inside = [float(distance) for index, distance in enumerate(sectors)
              if abs(wrap_angle((index + 0.5) * sector_size - math.pi
                                - center_angle)) <= half_angle]
    return min(inside) if inside else 1.0


def path_lookahead_features(sectors, body_yaw, headings, travel_distances,
                            max_range, body_margin, scale_m=2.0):
    """返回前瞻可达性观测特征 (features, avoid_needed)。

    features 为 5 个 [0,1] 浮点数：
      [0] reach        —— 前瞻可达的正面阻挡程度
      [1] band_deficit —— 沿未来航迹的横向带宽不足程度
      [2] slack_left_norm
      [3] slack_right_norm
      [4] avoid_needed —— 0/1 标志

    第 i 个航向只与第 i 个可行距离一一配对，不跨帧取 min。
    够不到的侧方障碍（slack > d_i + body_margin）按满量程处理，视为与未来
    航迹无关，否则八字轨迹另一叶的障碍会污染判据。
    sectors 为空时三个锥均返回 1.0，等价"全清"，avoid_needed = 0。
    """
    if not headings or not travel_distances:
        raise ValueError("headings and travel_distances must not be empty")
    if len(headings) != len(travel_distances):
        raise ValueError("headings and travel_distances must have equal length")
    if max_range <= 0.0:
        raise ValueError("max_range must be positive")
    if body_margin < 0.0:
        raise ValueError("body_margin must be non-negative")
    if scale_m <= 0.0:
        raise ValueError("scale_m must be positive")

    reach = 0.0
    band_deficit = 0.0
    tightest = -1.0  # 记录 severity 最大的帧，初值保证 i=0 也会被记录
    left_at_tightest = 1.0
    right_at_tightest = 1.0
    for heading, travel in zip(headings, travel_distances):
        frame = rotate_sectors_to_heading(sectors, body_yaw, heading)
        # 三个锥的最小距离（米 = 归一化值 × 满量程）
        forward = cone_min_clearance(frame, 0.0, math.pi / 6.0) * max_range
        left = cone_min_clearance(
            frame, math.pi / 2.0, math.pi / 3.0) * max_range
        right = cone_min_clearance(
            frame, -math.pi / 2.0, math.pi / 3.0) * max_range
        reachable = travel + body_margin
        frame_reach = 0.0 if reachable <= 0.0 else max(
            0.0, min(1.0, (reachable - forward) / reachable))
        slack = min(left, right)
        gap = slack if slack <= reachable else max_range
        frame_band = 0.0 if body_margin == 0.0 else max(
            0.0, min(1.0, (body_margin - gap) / body_margin))
        reach = max(reach, frame_reach)
        band_deficit = max(band_deficit, frame_band)
        severity = max(frame_reach, frame_band)
        if severity > tightest:
            tightest = severity
            left_at_tightest = left
            right_at_tightest = right
    avoid_needed = 1.0 if (reach > 0.0 or band_deficit > 0.0) else 0.0
    features = [reach, band_deficit,
                max(0.0, min(1.0, left_at_tightest / scale_m)),
                max(0.0, min(1.0, right_at_tightest / scale_m)),
                avoid_needed]
    return features, bool(avoid_needed > 0.5)


def action_swept_clearance(action, front, left, right):
    """Return clearance in the space swept by a discrete avoidance action."""
    if action == 0:
        return front
    if action in (1, 3):
        return min(front, left)
    if action in (2, 4):
        return min(front, right)
    if action == 5:
        return min(front, left, right)
    raise ValueError("action must be in range(6)")


def clearance_change_reward(previous, current, safe_distance, scale):
    """Reward increasing swept clearance while inside the safety zone."""
    if previous is None or safe_distance <= 0.0 or scale < 0.0:
        return 0.0
    if min(previous, current) >= safe_distance:
        return 0.0
    delta = (current - previous) / safe_distance
    return scale * max(-1.0, min(1.0, delta))


def time_to_collision(clearance, closing_rate, collision_distance):
    """Return geometric TTC; infinity means the obstacle is not closing."""
    if closing_rate <= 1e-6:
        return float("inf")
    return max(0.0, clearance - collision_distance) / closing_rate


def action_reversal_cost(previous_action, current_action, switch_penalty,
                         reversal_penalty):
    """Penalize left/right reversal more strongly than an ordinary switch."""
    if previous_action == current_action:
        return 0.0
    previous_side = (1 if previous_action in (1, 3)
                     else -1 if previous_action in (2, 4) else 0)
    current_side = (1 if current_action in (1, 3)
                    else -1 if current_action in (2, 4) else 0)
    return reversal_penalty if previous_side * current_side < 0 else switch_penalty


def project_path_progress(position, path):
    """Project a 2-D position onto a polyline and return (arc length, error)."""
    if len(path) < 2:
        return 0.0, 0.0
    best_error = float("inf")
    best_progress = 0.0
    cumulative = 0.0
    px, py = position
    for start, end in zip(path, path[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_sq = dx * dx + dy * dy
        length = math.sqrt(length_sq)
        if length_sq <= 1e-12:
            continue
        ratio = ((px - start[0]) * dx + (py - start[1]) * dy) / length_sq
        ratio = max(0.0, min(1.0, ratio))
        qx = start[0] + ratio * dx
        qy = start[1] + ratio * dy
        error = math.hypot(px - qx, py - qy)
        if error < best_error:
            best_error = error
            best_progress = cumulative + ratio * length
        cumulative += length
    return best_progress, (best_error if math.isfinite(best_error) else 0.0)


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
    if action not in range(6):
        raise ValueError("action must be in range(6)")
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


def allowed_avoidance_actions(clearance, emergency_distance,
                              avoidance_distance, warning_distance,
                              locked_side, left_clearance=1.0,
                              right_clearance=1.0, omni_min=1.0):
    """Return actions for clear, warning, avoidance and emergency zones.

    ``omni_min`` is the normalized minimum distance over the full LiDAR
    frame.  When the forward cone is still clear (``clearance`` above the
    warning distance) but an obstacle is already near the body elsewhere,
    straight flight can no longer be trusted on curved trajectories, so the
    mask is unlocked toward the blockable side instead of returning [0].
    """
    if not 0.0 < emergency_distance < avoidance_distance < warning_distance:
        raise ValueError(
            "distances must satisfy 0 < emergency < avoidance < warning")
    left_open = left_clearance >= emergency_distance
    right_open = right_clearance >= emergency_distance
    if clearance >= warning_distance:
        # 前方锥尚开阔但全向已贴脸时，曲线内侧障碍会漏出前方锥，需解锁绕行。
        if omni_min < emergency_distance:
            if locked_side > 0 and left_open:
                return [1, 3]
            if locked_side < 0 and right_open:
                return [2, 4]
            actions = []
            if left_open:
                actions.extend([1, 3])
            if right_open:
                actions.extend([2, 4])
            if actions:
                return actions
            return [5]
        return [0]

    if clearance >= avoidance_distance:
        actions = [0]
        if left_open:
            actions.append(1)
        if right_open:
            actions.append(2)
        return actions

    if locked_side > 0 and left_open:
        return [3] if clearance < emergency_distance else [1, 3]
    if locked_side < 0 and right_open:
        return [4] if clearance < emergency_distance else [2, 4]

    actions = []
    if left_open:
        actions.extend([3] if clearance < emergency_distance else [1, 3])
    if right_open:
        actions.extend([4] if clearance < emergency_distance else [2, 4])
    if actions:
        return actions
    return [5] if clearance < emergency_distance else [0]


def local_control_profile(clearance, emergency_distance,
                          avoidance_distance, warning_distance):
    """Return speed scale without taking control away from the policy."""
    if not 0.0 < emergency_distance < avoidance_distance < warning_distance:
        raise ValueError(
            "distances must satisfy 0 < emergency < avoidance < warning")
    if clearance >= warning_distance:
        return 1.0, False, 0
    if clearance >= avoidance_distance:
        return 0.8, False, 0
    if clearance >= emergency_distance:
        return 0.65, False, 0
    return 0.3, False, 0


def avoidance_tail_gate(active, tail_seen, rear_clearance,
                        completion_clearance, release_clearance):
    """Separate obstacle-passed completion from the later return-to-path gate."""
    if not 0.0 < completion_clearance <= release_clearance:
        raise ValueError("tail clearances must satisfy 0 < completion <= release")
    completion_ready = tail_seen and rear_clearance >= completion_clearance
    # A front-to-rear sector transition can briefly report both cones clear.
    # Keep the offset locked until the obstacle has actually been observed in
    # the rear cone, then wait for the larger release clearance.
    release_locked = active and (
        not tail_seen or rear_clearance < release_clearance)
    return completion_ready, release_locked


def avoidance_completion_allowed(forward_progress, completion_ready,
                                 release_locked):
    """Only finish after the observed tail has cleared the release margin."""
    return (forward_progress > 0.0 and completion_ready and
            not release_locked)


def corridor_aware_clearance(directional, omni, central, left, right,
                             collision_distance, safe_distance,
                             warning_distance):
    """Keep a physically open center gap traversable between two obstacles."""
    side_gap_is_safe = (
        collision_distance <= left < safe_distance and
        collision_distance <= right < safe_distance)
    if central >= warning_distance and side_gap_is_safe:
        return central
    return min(directional, omni)


def reset_recovery_timeout(base_timeout, distance, speed):
    """Allow enough time to fly back to a new moving-trajectory start."""
    if base_timeout <= 0.0 or speed <= 0.0:
        raise ValueError("reset timeout and speed must be positive")
    return max(base_timeout, distance / speed * 2.0 + 5.0)


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


def update_avoidance_event(active, clear_steps, event_count, min_clearance,
                           last_completion_distance, obstacle_distance,
                           tracking_error, goal_distance, safe_distance,
                           tracking_tolerance, required_clear_steps,
                           max_events, rearm_progress,
                           completion_allowed=True,
                           completion_obstacle_distance=None):
    """Advance one repeatable avoidance event and emit a completion pulse."""
    if required_clear_steps <= 0 or max_events <= 0 or rearm_progress < 0.0:
        raise ValueError("avoidance event limits must be positive")

    completed = False
    if not active:
        # A new entry after a completed clear phase is a new danger cluster.
        # Goal-distance gating made dense passages practically impossible to
        # re-arm because the vehicle often meets the next pole immediately.
        if obstacle_distance < safe_distance and event_count < max_events:
            active = True
            clear_steps = 0
            min_clearance = obstacle_distance

    if active:
        min_clearance = min(min_clearance, obstacle_distance)
        # Triggering uses an early-warning cone, while completion must use the
        # strict forward cone.  A side/rear obstacle should not keep an event
        # active after the vehicle has passed it.
        clear_distance = (obstacle_distance if completion_obstacle_distance
                          is None else completion_obstacle_distance)
        if clear_distance >= safe_distance and completion_allowed:
            clear_steps += 1
        else:
            clear_steps = 0
        if clear_steps >= required_clear_steps:
            active = False
            clear_steps = 0
            event_count += 1
            completed = True
            if goal_distance is not None:
                last_completion_distance = goal_distance

    return (active, clear_steps, event_count, min_clearance,
            last_completion_distance, completed)


def avoidance_quality_reward(min_clearance, recovery_error,
                             collision_distance, emergency_distance,
                             tracking_tolerance, base_reward, quality_bonus):
    """Reward a completed avoidance event according to clearance and recovery."""
    # 紧急动作阈值(最近) < 碰撞/判失败边界(较远)
    if not emergency_distance < collision_distance:
        raise ValueError("emergency distance must be below collision distance")
    if tracking_tolerance <= 0.0 or base_reward < 0.0 or quality_bonus < 0.0:
        raise ValueError("reward inputs and tracking tolerance must be valid")
    clearance_score = ((min_clearance - emergency_distance) /
                       (collision_distance - emergency_distance))
    clearance_score = max(0.0, min(1.0, clearance_score))
    recovery_score = 1.0 - recovery_error / tracking_tolerance
    recovery_score = max(0.0, min(1.0, recovery_score))
    quality = 0.75 * clearance_score + 0.25 * recovery_score
    return base_reward + quality_bonus * quality, quality


def update_settle_progress(target, stable_steps, position, speed,
                           speed_tolerance, position_tolerance):
    """Count consecutive low-speed observations near a fixed hold point."""
    if speed_tolerance < 0.0 or position_tolerance < 0.0:
        raise ValueError("settle tolerances must be non-negative")
    stable = (speed <= speed_tolerance and
              math.hypot(position[0] - target[0],
                         position[1] - target[1]) <= position_tolerance)
    return stable_steps + 1 if stable else 0


def episode_succeeded(done_reason):
    """显式完成的避障机动、到达目标点或跑完轨迹记为成功。"""
    return done_reason in ("avoidance_complete", "goal_reached",
                           "trajectory_complete")


def episode_is_success(done_reason, collision_event_count):
    """统一的回合成功判定：训练/评估/汇总全部使用本口径。

    显式完成（avoidance_complete / goal_reached / trajectory_complete）成功；
    超时一律判失败；其余终止原因失败。

    Args:
        collision_event_count: 兼容保留的形参（调用方按位置传入），在当前口径下
            不再影响判定。
    """
    return episode_succeeded(done_reason)


def validate_distance_thresholds(min_range, collision_distance,
                                 emergency_distance, safe_distance,
                                 contact_dist=0.0):
    """Validate physical distance thresholds used by perception and the environment."""
    if min_range < 0.0:
        raise ValueError("lidar min_range must be non-negative")
    if emergency_distance <= min_range:
        raise ValueError(
            "emergency_distance must be greater than lidar min_range so emergency "
            "observations are reachable")
    if not contact_dist <= emergency_distance < collision_distance < safe_distance:
        raise ValueError(
            "distance thresholds must satisfy contact <= emergency < collision < safe")
    # 说明：包络接触阈值(最近) <= 紧急动作阈值 < 风险带阈值 < 安全边界(最远)


def epsilon_linear_at_step(start, end, decay_steps, step):
    """线性衰减 epsilon 从 start 到 end，跨越 decay_steps 步。

    Args:
        start: 初始 epsilon 值（e.g. 1.0）
        end: 最终 epsilon 值（e.g. 0.05）
        decay_steps: 衰减总步数
        step: 当前步数

    Returns:
        float: 当前 epsilon 值
    """
    # 参数验证
    if not 0.0 <= end <= start <= 1.0:
        raise ValueError("epsilon must satisfy 0 <= end <= start <= 1")
    if decay_steps <= 0:
        raise ValueError("decay_steps must be positive")
    # 边界处理
    if step <= 0:
        return start
    if step >= decay_steps:
        return end
    # 线性插值
    return start + (end - start) * (float(step) / decay_steps)


_ROLLING_FIELDS = ("success_rate", "safe_success_rate", "collision_rate",
                   "risk_entry_rate", "timeout_rate", "mean_reward")
# 新增桶率：历史评估记录（旧 checkpoint）可能缺列，缺失时宽容地按 0.0 处理，
# 不加入必需字段校验，避免旧记录直接报错。
_ROLLING_OPTIONAL_FIELDS = ("shield_infeasible_rate", "reset_failure_rate",
                            "trajectory_complete_rate")


def parse_hidden_dims(value):
    """Parse "128,128" into a tuple of positive ints; raise ValueError on bad input."""
    parts = [part.strip() for part in str(value).split(",")]
    if not parts or any(part == "" for part in parts):
        raise ValueError("hidden dims 必须是非空逗号分隔整数列表，实际为 %r" % (value,))
    dims = []
    for part in parts:
        try:
            dim = int(part)
        except ValueError:
            raise ValueError("hidden dims 必须全部为整数，实际为 %r" % (value,))
        if dim <= 0:
            raise ValueError("hidden dims 每层维度必须为正数，实际为 %r" % (value,))
        dims.append(dim)
    return tuple(dims)


def rolling_evaluation_metrics(history, window):
    """Average the last `window` evaluations in `history`.

    Args:
        history: list of evaluation metric dicts, each containing the keys in
            _ROLLING_FIELDS.
        window: positive int; when fewer than `window` entries exist, all
            available entries are used and `window_used` reports the actual
            count.

    Returns:
        dict with keys rolling_success_rate, rolling_collision_rate,
        rolling_timeout_rate, rolling_shield_infeasible_rate,
        rolling_reset_failure_rate, rolling_trajectory_complete_rate,
        rolling_mean_reward, window_used.
    """
    if window < 1:
        raise ValueError("eval window 必须为正整数，实际为 %r" % (window,))
    entries = history[-window:]
    for entry in entries:
        missing = [field for field in _ROLLING_FIELDS if field not in entry]
        if missing:
            raise ValueError("评估记录缺少字段: %s" % (missing,))
    used = len(entries)
    if used == 0:
        rolling = {"rolling_%s" % field: 0.0 for field in _ROLLING_FIELDS}
        rolling.update(
            {"rolling_%s" % field: 0.0 for field in _ROLLING_OPTIONAL_FIELDS})
        rolling["window_used"] = 0
        return rolling
    rolling = {
        "success_rate": (
            sum(e["success_rate"] for e in entries) / used),
        "safe_success_rate": (
            sum(e["safe_success_rate"] for e in entries) / used),
        "collision_rate": (
            sum(e["collision_rate"] for e in entries) / used),
        "risk_entry_rate": (
            sum(e["risk_entry_rate"] for e in entries) / used),
        "timeout_rate": (
            sum(e["timeout_rate"] for e in entries) / used),
        "mean_reward": (
            sum(e["mean_reward"] for e in entries) / used),
    }
    # 新增桶率：旧记录缺列时按 0.0 处理
    rolling.update({
        field: sum(e.get(field, 0.0) for e in entries) / used
        for field in _ROLLING_OPTIONAL_FIELDS
    })
    result = {"rolling_%s" % key: value for key, value in rolling.items()}
    result["window_used"] = used
    return result


def evaluation_score(rolling_metrics):
    """Lexicographic best-model score over rolling metrics.

    Prefers higher rolling success rate, then lower collision rate, then lower
    timeout rate, then higher mean reward.
    """
    return (-rolling_metrics["rolling_collision_rate"],
            rolling_metrics["rolling_safe_success_rate"],
            -rolling_metrics["rolling_risk_entry_rate"],
            rolling_metrics["rolling_success_rate"],
            -rolling_metrics["rolling_timeout_rate"],
            rolling_metrics["rolling_mean_reward"])


def update_best_score(best_score, score, valid_episodes):
    """Return the new best score, or the previous one when not beaten.

    A score is never adopted when valid_episodes <= 0 (no valid episode means
    no model may be saved as best).
    """
    if valid_episodes <= 0:
        return best_score
    if best_score is None or score > best_score:
        return score
    return best_score


def count_termination_reason(counts, done_reason):
    """Add one episode's done_reason to a per-reason counter dict.

    Args:
        counts: dict mapping reason -> count, mutated in place.
        done_reason: string reason, or None/empty for unknown.

    Returns:
        The updated counts dict.
    """
    reason = done_reason if done_reason else "unknown"
    counts[reason] = counts.get(reason, 0) + 1
    return counts
