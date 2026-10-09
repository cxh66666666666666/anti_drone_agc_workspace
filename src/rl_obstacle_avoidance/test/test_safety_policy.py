#!/usr/bin/env python3

import math
import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from safety_policy import (action_geometry_cost, modulated_margin, select_a1_action,
                           select_filtered_action, select_p_action)


class SafetyPolicyTest(unittest.TestCase):
    def setUp(self):
        self.obstacle = {"center": [2.0, 0.0, 2.0], "size": [1.0, 1.0, 3.0],
                         "type": "pillar_box", "yaw": 0.0}

    def test_a1_has_zero_kappa(self):
        action, info = select_a1_action(
            0, (0, 0, 2), (1, 0, 0), [self.obstacle], horizon_s=1.0)
        self.assertEqual(info["kappa"], 0.0)
        self.assertEqual(info["margin"], 0.0)
        self.assertEqual(action, 0)

    def test_p_requires_positive_kappa(self):
        with self.assertRaises(ValueError):
            select_p_action(0, (0, 0, 2), (1, 0, 0), [], kappa=0.0)

    def test_action_cost_uses_geometry_not_label_distance(self):
        self.assertEqual(action_geometry_cost(5, 3), 1.0)
        self.assertEqual(action_geometry_cost(1, 2), 2.0)
        self.assertEqual(action_geometry_cost(0, 0), 0.0)

    def test_p_margin_increases_with_tracking_error(self):
        self.assertEqual(modulated_margin(0.2, 0.5, 0.0), 0.2)
        self.assertEqual(modulated_margin(0.2, 0.5, 0.4, 0.1), 0.44)

    def test_p_can_filter_action_a1_accepts(self):
        a1, i1 = select_a1_action(
            0, (0, 0, 2), (1, 0, 0), [self.obstacle], horizon_s=1.0)
        p, i2 = select_p_action(
            0, (0, 0, 2), (1, 0, 0), [self.obstacle],
            kappa=0.2, tracking_error=1.0, horizon_s=1.0,
            lateral_speed=1.2)
        self.assertEqual(a1, 0)
        self.assertNotEqual(p, 0)
        self.assertEqual(i2["filter_reason"], "candidate_filtered")
        self.assertIn("action_cost", i2)
        self.assertGreater(i2["action_cost"], 0.0)
        self.assertGreater(i2["margin"], i1["margin"])

    def test_empty_safe_set_is_terminal(self):
        # 无人机在障碍包络内部且 1s 时域内所有动作都无法脱出 → A_safe 真空集。
        action, info = select_filtered_action(
            0, (0, 0, 2), (1, 0, 0),
            [{"center": [0, 0, 2], "size": [2, 2, 3], "type": "pillar_box"}],
            horizon_s=1.0, base_margin=0.1, empty_set_terminal=True)
        self.assertIsNone(action)
        self.assertTrue(info["done"])
        self.assertEqual(info["done_reason"], "shield_infeasible")

    def test_empty_safe_set_escapes_instead_of_terminating(self):
        # 空安全集但存在能提升预测净空的逃逸动作：不再终止，返回逃逸动作。
        action, info = select_filtered_action(
            0, (1.3, 0.0, 2.0), (1.0, 0.0, 0.0),
            [{"center": [0, 0, 2], "size": [2, 2, 3], "type": "pillar_box"}],
            horizon_s=1.0, base_margin=0.1, empty_set_terminal=False)
        self.assertFalse(info["done"])
        self.assertEqual(info["filter_reason"], "shield_escape")
        self.assertTrue(info["shield_escape"])
        self.assertEqual(info["safe_actions"], ())
        self.assertIsNotNone(action)
        self.assertTrue(math.isfinite(info["escape_margin"]))
        self.assertGreater(info["escape_margin"], 0.0)
        self.assertEqual(info["margin"], 0.1)

    def test_no_improving_action_is_truly_infeasible(self):
        # 可行集内没有任何动作能提升预测净空 → 真正不可行，终止。
        action, info = select_filtered_action(
            0, (0.8, 0.0, 2.0), (1.0, 0.0, 0.0),
            [{"center": [2.0, 0.0, 2.0], "size": [1.0, 1.0, 3.0],
              "type": "pillar_box", "yaw": 0.0}],
            horizon_s=1.0, base_margin=0.2, feasible_actions=(0,),
            empty_set_terminal=False)
        self.assertIsNone(action)
        self.assertEqual(info["done_reason"], "shield_infeasible")
        self.assertEqual(info["filter_reason"], "shield_infeasible")

    def _frontal_wall_case(self):
        """正对宽墙接近的状态：净空沿前进方向线性递减，横向/纵向动作都救不回来。

        用宽墙（y 半宽 4 m）保证六个动作的预测净空全部下降——动作集不含制动档，
        因此 Δĥ > 0 的一级逃逸在此状态下恒不可达，正好触发二级兜底。
        """
        wall = {"center": [3.0, 0.0, 2.0], "size": [1.0, 8.0, 3.0],
                "type": "pillar_box", "yaw": 0.0}
        # 起点净空 1.65 m，1 s 时域末端 0.65 m（速度 1.0 m/s）。
        return (0.5, 0.0, 2.0), (1.0, 0.0, 0.0), [wall]

    def test_frontal_approach_uses_best_effort_instead_of_terminating(self):
        # α=0.2 下 DCBF 判据对动作 0 判空（末端跳 h_10=0.35 < (1-α)h_9=0.36），
        # 且无正增益逃逸动作。二级兜底应取预测净空最大的动作继续飞，而不是在尚有
        # 0.65 m 净空时放弃整段 episode。
        position, velocity, obstacles = self._frontal_wall_case()
        action, info = select_filtered_action(
            0, position, velocity, obstacles, feasible_actions=(0,),
            horizon_s=1.0, base_margin=0.3, alpha=0.2,
            lateral_speed=1.4, vertical_speed=0.3, clearance_floor=0.60)
        self.assertFalse(info["done"])
        self.assertEqual(info["safe_actions"], ())
        self.assertEqual(info["filter_reason"], "shield_best_effort")
        self.assertTrue(info["shield_escape"])
        self.assertEqual(action, 0)
        self.assertAlmostEqual(info["best_effort_clearance"], 0.65, places=6)

    def test_alpha_relaxes_same_state_to_non_empty_safe_set(self):
        # 同一状态在 α=0.5（A1/P 统一后的取值）下判据不判空：末端跳
        # h_10=0.35 >= (1-α)h_9=0.225。记录 α 本身即是成功率落差的一部分成因。
        position, velocity, obstacles = self._frontal_wall_case()
        action, info = select_filtered_action(
            0, position, velocity, obstacles, feasible_actions=(0,),
            horizon_s=1.0, base_margin=0.3, alpha=0.5,
            lateral_speed=1.4, vertical_speed=0.3, clearance_floor=0.60)
        self.assertEqual(info["safe_actions"], (0,))
        self.assertEqual(info["filter_reason"], "candidate_safe")
        self.assertEqual(action, 0)
        self.assertFalse(info.get("shield_escape", False))

    def test_best_effort_terminates_below_clearance_floor(self):
        # 抬高硬地板超过最优预测净空（0.70 > 0.65）：连最有利动作在时域内都进入
        # 风险带，此时才真正终止——兜底不改变"撞上去"的语义。
        position, velocity, obstacles = self._frontal_wall_case()
        action, info = select_filtered_action(
            0, position, velocity, obstacles, feasible_actions=(0,),
            horizon_s=1.0, base_margin=0.3, alpha=0.2,
            lateral_speed=1.4, vertical_speed=0.3, clearance_floor=0.70)
        self.assertIsNone(action)
        self.assertTrue(info["done"])
        self.assertEqual(info["done_reason"], "shield_infeasible")
        self.assertEqual(info["filter_reason"], "shield_infeasible")
        self.assertLess(info["best_effort_clearance"], 0.70)

    def test_safe_actions_are_subset_of_feasible(self):
        # A_safe ⊆ A_feasible：候选集被限定在环境可行集内。
        obstacles = [{"center": [3.0, 0.0, 2.0], "size": [1.0, 1.0, 3.0],
                      "type": "pillar_box", "yaw": 0.0},
                     {"center": [0.0, 1.2, 2.0], "size": [0.5, 0.5, 3.0],
                      "type": "pillar_box", "yaw": 0.0}]
        action, info = select_filtered_action(
            0, (0.0, 0.0, 2.0), (0.0, 0.0, 0.0), obstacles,
            horizon_s=1.0, base_margin=0.1, feasible_actions=(0, 1, 3))
        self.assertTrue(set(info["safe_actions"]) <= {0, 1, 3})
        self.assertIn(action, (0, 1, 3))

    def test_rule3_ablation_still_terminates(self):
        # 消融档 empty_set_terminal=True：空集仍按 §4.5 规则 3 终止。
        action, info = select_filtered_action(
            0, (0, 0, 2), (1, 0, 0),
            [{"center": [0, 0, 2], "size": [2, 2, 3], "type": "pillar_box"}],
            horizon_s=1.0, base_margin=0.1, empty_set_terminal=True)
        self.assertIsNone(action)
        self.assertEqual(info["done_reason"], "shield_infeasible")
        self.assertEqual(info["filter_reason"], "shield_infeasible")

    def test_clear_scene_is_not_intervened(self):
        # 无障碍且候选在可行集内：直接放行，不介入。
        action, info = select_filtered_action(
            0, (0, 0, 2), (1, 0, 0), [], feasible_actions=(0,))
        self.assertEqual(info["filter_reason"], "candidate_safe")
        self.assertEqual(action, 0)
        self.assertEqual(info["safe_actions"], (0,))
        self.assertFalse(info.get("shield_escape", False))

    def test_current_violation_still_allows_escape_action(self):
        """§4.5：h_k >= 0 是引理前提而非 A_safe 的成员条件。

        当前 h_k < 0（已进入 margin）但动作能把预测轨迹拉回 h >= 0 时，安全集
        必须非空——否则过滤器恰好在最需要它的时候被禁用。旧实现因多加了
        ``min(hs) >= 0`` 在此情形下判空。
        """
        action, info = select_filtered_action(
            0, (0.3, 0.0, 2.0), (-1.0, 0.0, 0.0),
            [{"center": [2.0, 0.0, 2.0], "size": [0.3, 0.3, 3.0],
              "type": "pillar_box", "yaw": 0.0}],
            horizon_s=1.0, base_margin=1.3)
        # 当前净空 1.2，margin 1.3 → h_k = -0.1；predicted_h 是时域最小值，
        # 仍含当前越界样本，故为 -0.1（首样本与动作无关，不参与成员判定）。
        self.assertAlmostEqual(info["predicted_h"], -0.1, places=6)
        self.assertIsNotNone(action)
        self.assertFalse(info["done"])
        self.assertIn(0, info["safe_actions"])
        self.assertEqual(info["filter_reason"], "candidate_safe")

    def test_real_state_regression_no_terminal_with_new_speeds(self):
        # 训练日志空集终止状态：新默认横向速度 1.4 下不应再返回 done。
        half_extents = (0.35, 0.35, 0.06)
        cases = (
            ((4.601830, 0.043710, 2.020520), (-1.69, -0.81, 0.0),
             [((4.784, -1.233, 2.0), (0.25, 0.25, 2.8)),
              ((3.205, -0.177, 2.0), (0.4, 0.4, 2.8))]),
            ((3.249714, -1.624667, 2.348509), (-0.78, 1.55, 0.0),
             [((2.082, -2.158, 2.0), (0.25, 0.25, 2.8)),
              ((2.736, -0.375, 2.0), (0.4, 0.4, 2.8))]),
        )
        for position, velocity, pairs in cases:
            obstacles = [{"center": list(center), "size": list(size),
                          "type": "pillar_box", "yaw": 0.0}
                         for center, size in pairs]
            action, info = select_a1_action(
                0, position, velocity, obstacles, horizon_s=1.0, dt=0.1,
                alpha=1.0, drone_half_extents=half_extents,
                lateral_speed=1.4, vertical_speed=0.3)
            self.assertFalse(info["done"], msg=str(position))
            self.assertNotEqual(info["safe_actions"], (), msg=str(position))
            self.assertFalse(info.get("shield_escape", False), msg=str(position))
            self.assertIn(action, info["safe_actions"])

    def test_invalid_sensor_is_terminal_and_not_assumed_clear(self):
        action, info = select_filtered_action(
            0, (0, 0, 2), (1, 0, 0), None, sensor_valid=False)
        self.assertIsNone(action)
        self.assertEqual(info["done_reason"], "sensor_invalid")

    def test_high_tracking_error_reduces_safe_set(self):
        args = dict(position=(0, 0, 2), velocity=(1, 0, 0),
                    obstacles=[self.obstacle], horizon_s=1.0,
                    lateral_speed=1.2)
        _, low = select_p_action(0, kappa=0.2, tracking_error=0.0, **args)
        _, high = select_p_action(0, kappa=0.2, tracking_error=2.0, **args)
        self.assertGreater(high["margin"], low["margin"])
        self.assertLessEqual(len(high["safe_actions"]), len(low["safe_actions"]))

    def test_safe_previous_action_is_held(self):
        action, info = select_a1_action(
            2, (0, 0, 2), (0, 0, 0), [], previous_action=1,
            hold_remaining=2)
        self.assertEqual(action, 1)
        self.assertEqual(info["filter_reason"], "action_hold")
        self.assertEqual(info["hold_remaining"], 1)

    def test_emergency_bypasses_action_hold(self):
        action, info = select_a1_action(
            2, (0, 0, 2), (0, 0, 0), [], previous_action=1,
            hold_remaining=2, emergency=True)
        self.assertEqual(action, 2)
        self.assertEqual(info["filter_reason"], "candidate_safe")

    def test_matching_proposal_still_consumes_hold_window(self):
        action, info = select_a1_action(
            1, (0, 0, 2), (0, 0, 0), [], previous_action=1,
            hold_remaining=2)
        self.assertEqual(action, 1)
        self.assertEqual(info["filter_reason"], "action_hold")
        self.assertEqual(info["hold_remaining"], 1)

    def test_unsafe_previous_action_is_not_held(self):
        action, info = select_a1_action(
            1, (0, 0, 2), (1, 0, 0), [self.obstacle], horizon_s=2.0,
            lateral_speed=1.2, previous_action=0, hold_remaining=2)
        self.assertEqual(action, 1)
        self.assertNotEqual(info["filter_reason"], "action_hold")

    def test_filtered_candidate_is_not_deferred_by_hold_window(self):
        # 候选被纠正时 proposed 是逃逸动作：保持窗口若覆盖它，纠正会被推迟最多
        # action_hold_steps 步（实测 0.4 s）。用 geometry cost 更高的安全旧动作
        # 触发旧行为，断言现在立即应用纠正动作。
        _, base = select_p_action(
            0, (0, 0, 2), (1, 0, 0), [self.obstacle],
            kappa=0.2, tracking_error=1.0, horizon_s=1.0, lateral_speed=1.2)
        self.assertEqual(base["filter_reason"], "candidate_filtered")
        held = max(base["safe_actions"],
                   key=lambda action: action_geometry_cost(0, action))
        action, info = select_p_action(
            0, (0, 0, 2), (1, 0, 0), [self.obstacle],
            kappa=0.2, tracking_error=1.0, horizon_s=1.0, lateral_speed=1.2,
            previous_action=held, hold_remaining=3)
        self.assertEqual(info["filter_reason"], "candidate_filtered")
        self.assertEqual(action, base["actual_action"])
        self.assertEqual(info["hold_remaining"], 0)

    def test_hold_window_keeps_action_when_not_emergency(self):
        # 非紧急：候选安全且在可行集内，保持上一动作并消耗保持窗口。
        action, info = select_filtered_action(
            0, (0, 0, 2), (0, 0, 0), [], feasible_actions=(0, 1, 2),
            previous_action=1, hold_remaining=2, emergency=False)
        self.assertEqual(action, 1)
        self.assertEqual(info["filter_reason"], "action_hold")
        self.assertEqual(info["hold_remaining"], 1)

    def test_hold_window_is_overridden_by_emergency(self):
        # 紧急：必须替换为候选动作，不得保持，窗口清零。
        action, info = select_filtered_action(
            0, (0, 0, 2), (0, 0, 0), [], feasible_actions=(0, 1, 2),
            previous_action=1, hold_remaining=2, emergency=True)
        self.assertEqual(action, 0)
        self.assertNotEqual(info["filter_reason"], "action_hold")
        self.assertEqual(info["hold_remaining"], 0)

    def test_hold_requires_previous_action_in_feasible_set(self):
        # previous_action 不在 A_feasible（故不在 A_safe）内 → 保持窗口不生效。
        action, info = select_filtered_action(
            0, (0, 0, 2), (0, 0, 0), [], feasible_actions=(0, 1),
            previous_action=2, hold_remaining=2)
        self.assertNotEqual(info["filter_reason"], "action_hold")
        self.assertEqual(action, 0)

    def test_escape_is_not_deferred_by_hold_window(self):
        # 空安全集但存在提升动作：逃逸立即应用，不被保持窗口推迟。
        action, info = select_filtered_action(
            0, (1.3, 0.0, 2.0), (1.0, 0.0, 0.0),
            [{"center": [0, 0, 2], "size": [2, 2, 3], "type": "pillar_box"}],
            horizon_s=1.0, base_margin=0.1, empty_set_terminal=False,
            previous_action=1, hold_remaining=3)
        self.assertEqual(info["filter_reason"], "shield_escape")
        self.assertTrue(info["shield_escape"])
        # 逃逸分支不返回保持窗口诊断。
        self.assertNotIn("hold_remaining", info)
        self.assertIsNotNone(action)

    def test_emergency_override_preserves_feasible_subset(self):
        # 紧急覆盖后动作仍必须落在 A_feasible 内。
        action, info = select_filtered_action(
            1, (0, 0, 2), (0, 0, 0), [], feasible_actions=(0, 1, 2),
            previous_action=0, hold_remaining=2, emergency=True)
        self.assertEqual(action, 1)  # 替换为候选而非保持 previous=0
        self.assertNotEqual(info["filter_reason"], "action_hold")
        self.assertIn(action, (0, 1, 2))

    def test_left_right_symmetry_of_mirrored_obstacles(self):
        left_obstacle = {"center": [1.5, 0.8, 2.0], "size": [0.6, 0.6, 3.0],
                         "type": "pillar_box"}
        right_obstacle = dict(left_obstacle, center=[1.5, -0.8, 2.0])
        _, left_info = select_a1_action(1, (0, 0, 2), (1, 0, 0),
                                        [left_obstacle], horizon_s=0.5)
        _, right_info = select_a1_action(2, (0, 0, 2), (1, 0, 0),
                                         [right_obstacle], horizon_s=0.5)
        self.assertEqual(left_info["predicted_h"], right_info["predicted_h"])

    def test_rule_actions_diagnostics_passthrough(self):
        # rule_actions 透传并排序去重；candidate_actions 记录策略候选。
        _, info = select_filtered_action(
            3, (0, 0, 2), (1, 0, 0), [], rule_actions=[3, 1],
            feasible_actions=(0, 1, 3))
        self.assertEqual(info["rule_actions"], (1, 3))
        self.assertEqual(info["candidate_actions"], (3,))

    def test_rule_actions_defaults_to_empty(self):
        _, info = select_filtered_action(
            0, (0, 0, 2), (1, 0, 0), [], feasible_actions=(0,))
        self.assertEqual(info["rule_actions"], ())
        self.assertEqual(info["candidate_actions"], (0,))

    def test_raw_interface_candidate_can_be_filtered(self):
        # raw 档核心可观测性：完整动作集内候选不满足 DCBF，但存在安全动作。
        action, info = select_p_action(
            0, (0, 0, 2), (1, 0, 0), [self.obstacle], kappa=0.2,
            tracking_error=1.0, horizon_s=1.0, lateral_speed=1.2,
            rule_actions=(0, 1, 2))
        self.assertEqual(info["filter_reason"], "candidate_filtered")
        self.assertNotEqual(action, 0)
        self.assertFalse(info.get("shield_escape", False))
        self.assertIn(action, info["safe_actions"])
        self.assertEqual(info["candidate_actions"], (0,))
        self.assertEqual(info["rule_actions"], (0, 1, 2))

    def test_empty_safe_set_diagnostics_include_new_keys(self):
        # 二级兜底分支同样回传 candidate_actions / rule_actions 诊断。
        position, velocity, obstacles = self._frontal_wall_case()
        _, info = select_filtered_action(
            0, position, velocity, obstacles, feasible_actions=(0,),
            horizon_s=1.0, base_margin=0.3, alpha=0.2,
            lateral_speed=1.4, vertical_speed=0.3, clearance_floor=0.60,
            rule_actions=[0])
        self.assertEqual(info["filter_reason"], "shield_best_effort")
        self.assertEqual(info["candidate_actions"], (0,))
        self.assertEqual(info["rule_actions"], (0,))

    def test_empty_safe_set_terminal_diagnostics_include_new_keys(self):
        # 严格终止分支（empty_set_terminal=True）同样回传新增诊断键。
        action, info = select_filtered_action(
            0, (0, 0, 2), (1, 0, 0),
            [{"center": [0, 0, 2], "size": [2, 2, 3], "type": "pillar_box"}],
            horizon_s=1.0, base_margin=0.1, empty_set_terminal=True,
            rule_actions=[])
        self.assertIsNone(action)
        self.assertEqual(info["candidate_actions"], (0,))
        self.assertEqual(info["rule_actions"], ())


if __name__ == "__main__":
    unittest.main()
