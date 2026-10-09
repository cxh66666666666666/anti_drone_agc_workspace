#!/usr/bin/env python3
"""A1/P action-selection policies built on the Stage-3 safe action set.

These wrappers deliberately leave the policy network and reward untouched.
They only evaluate the network's candidate action against current sensed
geometry and choose a safe fallback when needed.
"""

from safety_filter import (best_effort_action, empty_safe_set_termination,
                           escape_action, held_action, predicted_clearance,
                           safe_action_set)


def action_geometry_cost(candidate_action, actual_action):
    """Return an explicit local-action deviation cost.

    The cost is based on action geometry, not on the numeric action labels:
    lateral changes cost 1, vertical changes cost 0.5, and reversing the
    lateral side costs 2.  This keeps diagnostics meaningful if action IDs are
    reordered in a future protocol revision.
    """
    candidate_action = int(candidate_action)
    actual_action = int(actual_action)
    vectors = {
        0: (0, 0), 1: (1, 0), 2: (-1, 0),
        3: (1, 1), 4: (-1, 1), 5: (0, 1),
    }
    if candidate_action not in vectors or actual_action not in vectors:
        raise ValueError("action must be in range 6")
    candidate_lateral, candidate_vertical = vectors[candidate_action]
    actual_lateral, actual_vertical = vectors[actual_action]
    cost = 0.0
    if candidate_lateral != actual_lateral:
        if candidate_lateral * actual_lateral < 0:
            cost += 2.0
        else:
            cost += 1.0
    if candidate_vertical != actual_vertical:
        cost += 0.5
    return cost


def modulated_margin(base_margin, tracking_error, kappa=0.0,
                     prediction_error=0.0):
    """Return the safety margin used by A1/P.

    A1 is represented by ``kappa=0``.  P increases the margin with measured
    tracking error and an optional non-negative prediction-error bound.
    """
    base_margin = float(base_margin)
    tracking_error = float(tracking_error)
    kappa = float(kappa)
    prediction_error = float(prediction_error)
    if base_margin < 0.0 or tracking_error < 0.0 or kappa < 0.0 or prediction_error < 0.0:
        raise ValueError("margin, tracking_error, kappa and prediction_error must be non-negative")
    return base_margin + kappa * (tracking_error + prediction_error)


def select_filtered_action(candidate_action, position, velocity, obstacles, *,
                           feasible_actions=None, rule_actions=None,
                           base_margin=0.0, tracking_error=0.0, kappa=0.0,
                           prediction_error=0.0, horizon_s=1.0, dt=0.1,
                           alpha=1.0, epsilon_pred=0.0,
                           drone_half_extents=None, lateral_speed=0.6,
                           vertical_speed=0.4, actions=range(6),
                           sensor_valid=True, previous_action=None,
                           hold_remaining=0, emergency=False,
                           empty_set_terminal=False, clearance_floor=0.0):
    """Select a candidate or least-deviation safe fallback.

    ``feasible_actions`` 给出环境可行性集合 ``A_feasible``，安全集在其中求取
    （即 ``A_safe ⊆ A_feasible``）；为 None 时退回 ``actions``。

    Returns ``(action, diagnostics)``.  ``A_feasible`` 为空时直接终止。``A_safe``
    为空且 ``empty_set_terminal`` 为 True 时按 §4.5 规则 3 严格终止；默认档
    （False）按两级兜底处理：先在可行集内选使预测净空提升的动作（``shield_escape``）；
    不存在这样的动作时退到最差情形最优动作，仅当其预测时域内最小净空仍低于
    ``clearance_floor`` 才以 ``shield_infeasible`` 终止（``shield_best_effort``）。
    第二级兜底是必要的：正对障碍物接近时没有任何动作能提升净空（动作集不含制动），
    若仍以正增益为唯一逃逸判据，屏蔽器会在尚有数米净空时直接放弃整段 episode。
    """
    candidate_action = int(candidate_action)
    # B1 规则掩码：仅作诊断回传（未提供则空 tuple），不参与安全裁决。
    rule_actions_diag = tuple(sorted(
        int(action) for action in (() if rule_actions is None else rule_actions)))
    feasible = tuple(sorted({int(action) for action in (
        actions if feasible_actions is None else feasible_actions)}))
    margin = modulated_margin(base_margin, tracking_error, kappa,
                              prediction_error)
    if not sensor_valid or obstacles is None:
        return None, {
            "done": True, "done_reason": "sensor_invalid",
            "candidate_action": candidate_action, "actual_action": None,
            "candidate_actions": (candidate_action,),
            "rule_actions": rule_actions_diag,
            "safe_actions": (), "kappa": float(kappa),
            "margin": margin, "filter_reason": "sensor_invalid",
            "prediction_error": float(prediction_error),
            "action_cost": None, "escape_margin": None,
            "safety_filter_active": True,
        }
    if not feasible:
        # 可行动作集为空：无任何可发布动作，直接终止。
        diagnostics = empty_safe_set_termination(())
        diagnostics.update({"candidate_action": candidate_action,
                            "candidate_actions": (candidate_action,),
                            "rule_actions": rule_actions_diag,
                            "actual_action": None, "safe_actions": (),
                            "kappa": kappa, "margin": margin,
                            "filter_reason": "shield_infeasible",
                            "prediction_error": float(prediction_error),
                            "action_cost": None, "escape_margin": None})
        return None, diagnostics
    safe = safe_action_set(
        position, velocity, obstacles, feasible,
        horizon_s=horizon_s, dt=dt, margin=margin,
        alpha=alpha, epsilon_pred=epsilon_pred,
        drone_half_extents=drone_half_extents,
        lateral_speed=lateral_speed, vertical_speed=vertical_speed)
    if not safe:
        if empty_set_terminal:
            # 消融档：禁用逃逸，退回 §4.5 规则 3 严格终止。
            diagnostics = empty_safe_set_termination(())
            diagnostics.update({"candidate_action": candidate_action,
                                "candidate_actions": (candidate_action,),
                                "rule_actions": rule_actions_diag,
                                "actual_action": None, "safe_actions": (),
                                "kappa": kappa, "margin": margin,
                                "filter_reason": "shield_infeasible",
                                "prediction_error": float(prediction_error),
                                "action_cost": None, "escape_margin": None})
            return None, diagnostics
        # 默认档：备份逃逸优先。在可行集内选使预测净空提升的动作。
        escape, escape_margin = escape_action(
            feasible, position, velocity, obstacles,
            horizon_s=horizon_s, dt=dt, margin=margin,
            drone_half_extents=drone_half_extents,
            lateral_speed=lateral_speed, vertical_speed=vertical_speed,
            alpha=alpha)
        if escape is None:
            # 二级兜底：无正增益动作时取最差情形最优动作。正对障碍物接近时所有动作
            # 的预测净空都在下降，一级逃逸恒不可达；此时只要最有利的动作在时域内
            # 不进入风险带（clearance_floor），就继续飞而不是放弃整段 episode。
            best, best_clearance = best_effort_action(
                feasible, position, velocity, obstacles,
                horizon_s=horizon_s, dt=dt,
                drone_half_extents=drone_half_extents,
                lateral_speed=lateral_speed, vertical_speed=vertical_speed,
                clearance_floor=clearance_floor)
            if best is None:
                # 连最有利动作的预测最小净空都低于风险带：真正不可行，终止。
                diagnostics = empty_safe_set_termination(())
                diagnostics.update({"candidate_action": candidate_action,
                                    "candidate_actions": (candidate_action,),
                                    "rule_actions": rule_actions_diag,
                                    "actual_action": None, "safe_actions": (),
                                    "kappa": kappa, "margin": margin,
                                    "filter_reason": "shield_infeasible",
                                    "prediction_error": float(prediction_error),
                                    "action_cost": None, "escape_margin": None,
                                    "best_effort_clearance": best_clearance})
                return None, diagnostics
            return best, {
                "done": False,
                "candidate_action": candidate_action,
                "candidate_actions": (candidate_action,),
                "rule_actions": rule_actions_diag,
                "actual_action": best,
                "safe_actions": (),
                "kappa": kappa,
                "margin": margin,
                "predicted_h": best_clearance - margin,
                "escape_margin": None,
                "best_effort_clearance": best_clearance,
                "prediction_error": float(prediction_error),
                "action_cost": action_geometry_cost(candidate_action, best),
                "filter_reason": "shield_best_effort",
                "shield_escape": True,
                "safety_filter_active": True,
            }
        return escape, {
            "done": False,
            "candidate_action": candidate_action,
            "candidate_actions": (candidate_action,),
            "rule_actions": rule_actions_diag,
            "actual_action": escape,
            "safe_actions": (),
            "kappa": kappa,
            "margin": margin,
            "predicted_h": predicted_clearance(
                position, velocity, escape, obstacles,
                horizon_s=horizon_s, dt=dt, margin=margin,
                drone_half_extents=drone_half_extents,
                lateral_speed=lateral_speed, vertical_speed=vertical_speed),
            "escape_margin": escape_margin,
            "prediction_error": float(prediction_error),
            "action_cost": action_geometry_cost(candidate_action, escape),
            "filter_reason": "shield_escape",
            "shield_escape": True,
            "safety_filter_active": True,
        }
    if candidate_action in safe:
        proposed = candidate_action
        reason = "candidate_safe"
    else:
        proposed = min(safe, key=lambda action: (action_geometry_cost(candidate_action, action), action))
        reason = "candidate_filtered"
    remaining = 0
    # 保持窗口只用于抑制抖动：仅在候选动作本身安全（proposed 即 candidate）时
    # 生效。候选被过滤器纠正时 proposed 是逃逸动作，若仍被保持窗口覆盖，
    # 纠正最多被推迟 action_hold_steps 步（实测 0.4 s，约 0.4 m 行程）。
    hold_active = (reason == "candidate_safe" and previous_action in safe and
                   int(hold_remaining) > 0 and not emergency)
    if hold_active:
        actual, remaining = held_action(
            previous_action, proposed, hold_remaining, emergency=emergency)
        reason = "action_hold"
    else:
        actual = proposed
    h = predicted_clearance(
        position, velocity, actual, obstacles, horizon_s=horizon_s, dt=dt,
        margin=margin, drone_half_extents=drone_half_extents,
        lateral_speed=lateral_speed, vertical_speed=vertical_speed)
    return actual, {
        "done": False,
        "candidate_action": candidate_action,
        "candidate_actions": (candidate_action,),
        "rule_actions": rule_actions_diag,
        "actual_action": actual,
        "safe_actions": safe,
        "kappa": kappa,
        "margin": margin,
        "predicted_h": h,
        "escape_margin": None,
        "prediction_error": float(prediction_error),
        "action_cost": action_geometry_cost(candidate_action, actual),
        "hold_remaining": remaining,
        "filter_reason": reason,
        "safety_filter_active": True,
    }


def select_a1_action(candidate_action, position, velocity, obstacles, **kwargs):
    """A1: static safety margin, equivalent to P with κ=0."""
    kwargs["kappa"] = 0.0
    kwargs["prediction_error"] = 0.0
    return select_filtered_action(candidate_action, position, velocity,
                                  obstacles, **kwargs)


def select_p_action(candidate_action, position, velocity, obstacles, *,
                    kappa, **kwargs):
    """P: tracking-error-modulated margin with strictly positive κ."""
    if float(kappa) <= 0.0:
        raise ValueError("P requires kappa > 0")
    return select_filtered_action(candidate_action, position, velocity,
                                  obstacles, kappa=kappa, **kwargs)
