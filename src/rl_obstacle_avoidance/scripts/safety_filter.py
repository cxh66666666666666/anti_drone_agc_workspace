#!/usr/bin/env python3
"""Pure, ROS-free safety prediction primitives for Stage 3.

The filter consumes only the current state and currently sensed obstacle
geometry.  It predicts a short constant-velocity horizon and evaluates the
signed geometric clearance function ``h = clearance - margin``.  No controller
or reward code is changed here; integration is a later task.
"""

import math

from geometry_metrics import minimum_geometry_clearance


ACTION_VECTORS = {
    0: (0.0, 0.0, 0.0),   # nominal reference velocity
    1: (0.0, 1.0, 0.0),   # left
    2: (0.0, -1.0, 0.0),  # right
    3: (0.0, 1.0, 1.0),   # left + up
    4: (0.0, -1.0, 1.0),  # right + up
    5: (0.0, 0.0, 1.0),   # up/hover
}


def sensor_is_valid(obstacles, sensor_valid=True):
    """Return whether the current observation is usable for certification."""
    return bool(sensor_valid) and obstacles is not None


def held_action(previous_action, proposed_action, remaining_steps,
                *, emergency=False):
    """Keep a previously safe action during its hold window.

    Emergency intervention may always replace the held action.  A non-positive
    remaining count means the new proposal can be applied.
    """
    if emergency or int(remaining_steps) <= 0 or previous_action is None:
        return int(proposed_action), max(0, int(remaining_steps))
    return int(previous_action), int(remaining_steps) - 1


def safety_function(clearance, margin=0.0):
    """Return signed safety value: non-negative is safe."""
    clearance = float(clearance)
    margin = float(margin)
    if margin < 0.0:
        raise ValueError("margin must be non-negative")
    return clearance - margin


def action_velocity(action, velocity, lateral_speed=0.6, vertical_speed=0.4):
    """Map an action to a predicted world velocity increment.

    ``velocity`` is ``(vx, vy, vz)``.  The lateral direction is defined in the
    world frame by ``heading`` so this function remains deterministic and
    independent of ROS messages.
    """
    action = int(action)
    if action not in ACTION_VECTORS:
        raise ValueError("action must be in range 6")
    vx, vy, vz = (float(value) for value in velocity)
    heading = math.atan2(vy, vx) if math.hypot(vx, vy) > 1e-9 else 0.0
    forward = (math.cos(heading), math.sin(heading))
    left = (-forward[1], forward[0])
    _, lateral, vertical = ACTION_VECTORS[action]
    return (vx + lateral * lateral_speed * left[0],
            vy + lateral * lateral_speed * left[1],
            vz + vertical * vertical_speed)


def predict_trajectory(position, velocity, action, horizon_s=1.0,
                       dt=0.1, lateral_speed=0.6, vertical_speed=0.4):
    """Predict sampled positions under a constant action velocity.

    The first sample is the current position and the final sample is exactly
    ``horizon_s`` (within floating-point tolerance).
    """
    if horizon_s < 0.0 or dt <= 0.0:
        raise ValueError("horizon_s must be non-negative and dt positive")
    steps = max(1, int(math.ceil(horizon_s / dt))) if horizon_s else 0
    actual_dt = horizon_s / steps if steps else 0.0
    vx, vy, vz = action_velocity(action, velocity, lateral_speed, vertical_speed)
    x, y, z = (float(value) for value in position)
    trajectory = [(x, y, z)]
    for _ in range(steps):
        x += vx * actual_dt
        y += vy * actual_dt
        z += vz * actual_dt
        trajectory.append((x, y, z))
    return trajectory


def predicted_clearance(position, velocity, action, obstacles, *,
                        horizon_s=1.0, dt=0.1, margin=0.0,
                        drone_half_extents=None, lateral_speed=0.6,
                        vertical_speed=0.4):
    """Return minimum ``h`` over the predicted trajectory samples."""
    kwargs = {} if drone_half_extents is None else {
        "drone_half_extents": drone_half_extents}
    clearances = [minimum_geometry_clearance(point, obstacles, **kwargs)
                  for point in predict_trajectory(
                      position, velocity, action, horizon_s, dt,
                      lateral_speed, vertical_speed)]
    return min(safety_function(clearance, margin) for clearance in clearances)


def safe_action_set(position, velocity, obstacles, actions=range(6), *,
                    horizon_s=1.0, dt=0.1, margin=0.0,
                    drone_half_extents=None, lateral_speed=0.6,
                    vertical_speed=0.4, alpha=1.0, epsilon_pred=0.0):
    """Return actions satisfying the discrete-time prediction condition.

    Membership follows §4.5 exactly: ``Δh + alpha*h >= epsilon_pred`` on every
    sampled transition.  ``h_k >= 0`` is the *precondition of the conditional
    lemma*, not a membership test -- requiring it here would make the set empty
    for every action whenever the drone is already inside the margin (``h_k <
    0``), i.e. exactly when the filter is most needed.  The first sample is the
    current position and is therefore action-independent, so it must not enter
    the test.
    """
    alpha = float(alpha)
    epsilon_pred = float(epsilon_pred)
    if not 0.0 < alpha <= 1.0 or epsilon_pred < 0.0:
        raise ValueError("alpha must be in (0, 1] and epsilon_pred non-negative")
    safe = []
    for action in actions:
        trajectory = predict_trajectory(position, velocity, int(action),
                                         horizon_s, dt, lateral_speed,
                                         vertical_speed)
        hs = [safety_function(minimum_geometry_clearance(
            point, obstacles, **({} if drone_half_extents is None else {
                "drone_half_extents": drone_half_extents})), margin)
              for point in trajectory]
        transitions_ok = all(
            (not math.isfinite(hs[index - 1]) or
             hs[index] - hs[index - 1] + alpha * hs[index - 1] >= epsilon_pred)
            for index in range(1, len(hs)))
        if hs and transitions_ok:
            safe.append(int(action))
    return tuple(sorted(set(safe)))


def empty_safe_set_termination(safe_actions, *, reason="shield_infeasible"):
    """Return the canonical terminal event for an empty ``A_safe``."""
    if safe_actions:
        raise ValueError("termination is only valid for an empty safe set")
    return {"done": True, "done_reason": reason,
            "safe_actions": (), "safety_filter_active": True}


def predicted_horizon_gain(position, velocity, action, obstacles, *,
                           horizon_s=1.0, dt=0.1, margin=0.0,
                           drone_half_extents=None, lateral_speed=0.6,
                           vertical_speed=0.4):
    """返回该动作在预测时域末端的净空变化 Δĥ = h_end - h_now。

    h_now 由当前位置与同一门控障碍集给出，与动作无关；h_end 取时域末端采样点。
    两者扣同一 margin，故 Δĥ 只反映几何变化。末端的定义见 §4.5 的一阶短时预测。
    """
    kwargs = {} if drone_half_extents is None else {
        "drone_half_extents": drone_half_extents}
    h_now = safety_function(
        minimum_geometry_clearance(position, obstacles, **kwargs), margin)
    end = predict_trajectory(position, velocity, int(action), horizon_s, dt,
                             lateral_speed, vertical_speed)[-1]
    h_end = safety_function(
        minimum_geometry_clearance(end, obstacles, **kwargs), margin)
    return h_end - h_now


def clearance_improving_actions(candidates, position, velocity, obstacles, *,
                                horizon_s=1.0, dt=0.1, margin=0.0,
                                drone_half_extents=None, lateral_speed=0.6,
                                vertical_speed=0.4):
    """返回候选集内使预测净空提升（Δĥ(a) > 0）的动作元组，按动作号升序。"""
    actions = tuple(sorted({int(action) for action in candidates}))
    return tuple(
        action for action in actions
        if predicted_horizon_gain(
            position, velocity, action, obstacles, horizon_s=horizon_s, dt=dt,
            margin=margin, drone_half_extents=drone_half_extents,
            lateral_speed=lateral_speed, vertical_speed=vertical_speed) > 0.0)


def escape_action(candidates, position, velocity, obstacles, *,
                  horizon_s=1.0, dt=0.1, margin=0.0,
                  drone_half_extents=None, lateral_speed=0.6,
                  vertical_speed=0.4, alpha=1.0):
    """在可行动作集内选备份逃逸动作，返回 (action, escape_margin)。

    判据为 Δĥ(a) + α h（h 为当前净空）。仅当候选集内不存在 Δĥ(a) > 0 的动作时
    返回 (None, None)，由调用方转入 shield_infeasible 终止。平手时按动作号升序
    取最小，保证确定性。候选集为空时同样返回 (None, None)。
    """
    alpha = float(alpha)
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha must be in (0, 1]")
    actions = tuple(sorted({int(action) for action in candidates}))
    if not actions:
        return None, None
    kwargs = {} if drone_half_extents is None else {
        "drone_half_extents": drone_half_extents}
    # h_now 与 predicted_horizon_gain 内部同源，只算一次避免口径不一致。
    h_now = safety_function(
        minimum_geometry_clearance(position, obstacles, **kwargs), margin)
    best_action, best_margin = None, None
    for action in actions:
        end = predict_trajectory(position, velocity, action, horizon_s, dt,
                                 lateral_speed, vertical_speed)[-1]
        gain = (safety_function(
            minimum_geometry_clearance(end, obstacles, **kwargs), margin)
            - h_now)
        if gain <= 0.0:
            continue
        escape_margin = gain + alpha * h_now
        # 严格大于：平手时保留动作号更小者（遍历已升序）。
        if best_margin is None or escape_margin > best_margin:
            best_action, best_margin = action, escape_margin
    if best_action is None:
        return None, None
    return best_action, best_margin


def best_effort_action(candidates, position, velocity, obstacles, *,
                       horizon_s=1.0, dt=0.1, drone_half_extents=None,
                       lateral_speed=0.6, vertical_speed=0.4,
                       clearance_floor=0.0):
    """A_safe 为空且无正增益逃逸动作时的兜底，返回 (action, 预测最小净空)。

    在可行动作集内取「预测时域内最小净空最大」者（最差情形最优）。该最大值仍低于
    ``clearance_floor`` 时返回 (None, 最大值)，由调用方按 §4.5 规则 3 终止——此时连
    最有利的动作都会在时域内进入风险带，继续执行不再有意义。判据只用几何净空，
    不与 safety margin 相减：兜底要回答的是「会不会撞上」，不是「证书能否维持」。
    平手时按动作号升序取最小以保证确定性；候选集为空时返回 (None, None)。
    """
    actions = tuple(sorted({int(action) for action in candidates}))
    if not actions:
        return None, None
    kwargs = {} if drone_half_extents is None else {
        "drone_half_extents": drone_half_extents}
    best_action, best_clearance = None, None
    for action in actions:
        clearance = min(
            minimum_geometry_clearance(point, obstacles, **kwargs)
            for point in predict_trajectory(position, velocity, action,
                                            horizon_s, dt, lateral_speed,
                                            vertical_speed))
        # 严格大于：平手时保留动作号更小者（遍历已升序）。
        if best_clearance is None or clearance > best_clearance:
            best_action, best_clearance = action, clearance
    if best_clearance < float(clearance_floor):
        return None, best_clearance
    return best_action, best_clearance
