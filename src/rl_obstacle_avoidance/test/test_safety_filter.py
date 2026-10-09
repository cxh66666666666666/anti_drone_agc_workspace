#!/usr/bin/env python3

import math
import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from geometry_metrics import minimum_geometry_clearance
from safety_filter import (best_effort_action, clearance_improving_actions,
                           empty_safe_set_termination, escape_action,
                           held_action, predicted_clearance,
                           predicted_horizon_gain, predict_trajectory,
                           safety_function, safe_action_set)


class SafetyFilterTest(unittest.TestCase):
    def setUp(self):
        self.obstacle = {"center": [2.0, 0.0, 2.0], "size": [1.0, 1.0, 3.0],
                         "type": "pillar_box", "yaw": 0.0}

    def test_safety_function_signed_margin(self):
        self.assertEqual(safety_function(1.5, 1.0), 0.5)
        self.assertEqual(safety_function(0.5, 1.0), -0.5)
        with self.assertRaises(ValueError):
            safety_function(1.0, -0.1)

    def test_prediction_includes_current_and_horizon(self):
        points = predict_trajectory((0, 0, 2), (1, 0, 0), 0,
                                    horizon_s=1.0, dt=0.3)
        self.assertEqual(points[0], (0.0, 0.0, 2.0))
        self.assertAlmostEqual(points[-1][0], 1.0)
        self.assertEqual(len(points), 5)

    def test_forward_action_becomes_unsafe(self):
        h = predicted_clearance((0, 0, 2), (1, 0, 0), 0,
                                [self.obstacle], horizon_s=2.0,
                                dt=0.1, margin=0.2)
        self.assertLess(h, 0.0)

    def test_empty_obstacles_all_actions_safe(self):
        actions = safe_action_set((0, 0, 2), (1, 0, 0), [], horizon_s=1.0)
        self.assertEqual(actions, (0, 1, 2, 3, 4, 5))

    def test_lateral_action_can_remain_safe(self):
        actions = safe_action_set((0, 0, 2), (1, 0, 0), [self.obstacle],
                                  horizon_s=2.0, margin=0.1,
                                  lateral_speed=1.2)
        self.assertIn(1, actions)
        self.assertNotIn(0, actions)

    def test_empty_safe_set_terminal_event(self):
        event = empty_safe_set_termination(())
        self.assertTrue(event["done"])
        self.assertEqual(event["done_reason"], "shield_infeasible")
        with self.assertRaises(ValueError):
            empty_safe_set_termination((1,))

    def test_boundary_clearance_is_safe_at_zero(self):
        self.assertEqual(safety_function(0.0), 0.0)
        self.assertEqual(safety_function(0.0, 0.1), -0.1)

    def test_action_hold_and_emergency_override(self):
        self.assertEqual(held_action(1, 2, 2), (1, 1))
        self.assertEqual(held_action(1, 2, 2, emergency=True), (2, 2))

    def test_held_action_emergency_override_matrix(self):
        # 矩阵断言：紧急或窗口耗尽/无上一动作时返回 proposed；否则保持 previous 并递减。
        cases = (
            # (previous, proposed, remaining, emergency, expected)
            (1, 2, 3, False, (1, 2)),
            (1, 2, 1, False, (1, 0)),
            (1, 2, 0, False, (2, 0)),
            (1, 2, -1, False, (2, 0)),
            (1, 2, 3, True, (2, 3)),
            (1, 2, 0, True, (2, 0)),
            (None, 2, 3, False, (2, 3)),
        )
        for previous, proposed, remaining, emergency, expected in cases:
            with self.subTest(previous=previous, remaining=remaining,
                              emergency=emergency):
                self.assertEqual(
                    held_action(previous, proposed, remaining,
                               emergency=emergency),
                    expected)


class RealStateRegressionTest(unittest.TestCase):
    """训练日志中 27/204 回合空集终止的真实状态回归。

    旧实现把偏移量 min_offset_xy(0.3) 当作横向速度传入，导致安全集为空；
    真实可达横向速度上限为 max_offset_xy(1.4)。
    """

    HALF_EXTENTS = (0.35, 0.35, 0.06)

    def _obstacles(self, pairs):
        return [{"center": list(center), "size": list(size),
                 "type": "pillar_box", "yaw": 0.0}
                for center, size in pairs]

    def _assert_recovered(self, position, velocity, pairs):
        common = dict(horizon_s=1.0, dt=0.1, margin=0.0, alpha=1.0,
                      drone_half_extents=self.HALF_EXTENTS,
                      vertical_speed=0.3)
        obstacles = self._obstacles(pairs)
        old = safe_action_set(position, velocity, obstacles,
                              lateral_speed=0.3, **common)
        new = safe_action_set(position, velocity, obstacles,
                              lateral_speed=1.4, **common)
        self.assertEqual(old, ())   # 旧偏移量口径下确实为空集
        self.assertNotEqual(new, ())  # 新横向速度口径下非空
        return new

    def test_s2v2_fe6677b45f_is_no_longer_empty(self):
        actions = self._assert_recovered(
            (4.601830, 0.043710, 2.020520), (-1.69, -0.81, 0.0),
            [((4.784, -1.233, 2.0), (0.25, 0.25, 2.8)),
             ((3.205, -0.177, 2.0), (0.4, 0.4, 2.8))])
        self.assertIn(1, actions)
        self.assertIn(3, actions)

    def test_s2v2_761be834a0_is_no_longer_empty(self):
        actions = self._assert_recovered(
            (3.249714, -1.624667, 2.348509), (-0.78, 1.55, 0.0),
            [((2.082, -2.158, 2.0), (0.25, 0.25, 2.8)),
             ((2.736, -0.375, 2.0), (0.4, 0.4, 2.8))])
        self.assertNotEqual(actions, ())


class EscapeActionTest(unittest.TestCase):
    """备份逃逸：在可行集内按预测净空提升 Δĥ 选动作。"""

    def _obstacle(self, center, size):
        return {"center": list(center), "size": list(size),
                "type": "pillar_box", "yaw": 0.0}

    def test_escape_action_prefers_largest_clearance_gain(self):
        position, velocity = (0.0, 0.3, 2.0), (0.0, 0.0, 0.0)
        obstacles = [self._obstacle((1.0, 0.0, 2.0), (1.0, 1.0, 3.0))]
        candidates = (0, 1, 2, 3, 4, 5)
        gains = {action: predicted_horizon_gain(position, velocity, action,
                                                obstacles)
                 for action in candidates}
        improving = {action: gain for action, gain in gains.items()
                     if gain > 0.0}
        self.assertTrue(improving)  # 至少存在能提升净空的动作
        best_gain = max(improving.values())
        expected = min(action for action, gain in improving.items()
                       if gain == best_gain)  # 平手取动作号最小
        h_now = safety_function(minimum_geometry_clearance(position, obstacles))
        action, escape_margin = escape_action(candidates, position, velocity,
                                              obstacles, alpha=1.0)
        self.assertEqual(action, expected)
        self.assertAlmostEqual(escape_margin, best_gain + h_now, places=6)

    def test_escape_action_returns_none_without_improvement(self):
        position, velocity = (0.0, 0.0, 2.0), (1.0, 0.0, 0.0)
        obstacles = [self._obstacle((1.0, 0.0, 2.0), (1.0, 1.0, 3.0))]
        self.assertLess(
            predicted_horizon_gain(position, velocity, 0, obstacles), 0.0)
        self.assertEqual(escape_action((0,), position, velocity, obstacles),
                         (None, None))
        self.assertEqual(
            clearance_improving_actions((0,), position, velocity, obstacles), ())

    def test_clearance_improving_actions_excludes_non_improving(self):
        position, velocity = (0.0, 0.3, 2.0), (0.0, 0.0, 0.0)
        obstacles = [self._obstacle((1.0, 0.0, 2.0), (1.0, 1.0, 3.0))]
        result = clearance_improving_actions((0, 1, 2, 3, 4, 5), position,
                                             velocity, obstacles)
        self.assertEqual(result, (1, 3))  # 仅左移/左移上升提升净空，升序
        for action in result:
            self.assertGreater(
                predicted_horizon_gain(position, velocity, action, obstacles), 0.0)
        for action in set(range(6)) - set(result):
            self.assertLessEqual(
                predicted_horizon_gain(position, velocity, action, obstacles), 0.0)

    def test_escape_action_rejects_bad_alpha(self):
        obstacles = [self._obstacle((1.0, 0.0, 2.0), (1.0, 1.0, 3.0))]
        for alpha in (0.0, 1.5):
            with self.assertRaises(ValueError):
                escape_action((0,), (0, 0, 2), (0, 0, 0), obstacles, alpha=alpha)

    def test_escape_action_empty_candidates(self):
        self.assertEqual(escape_action((), (0, 0, 2), (0, 0, 0), []),
                         (None, None))


class BestEffortActionTest(unittest.TestCase):
    """二级兜底：无正增益动作时取预测时域内最小净空最大的动作。"""

    def _obstacle(self, center, size):
        return {"center": list(center), "size": list(size),
                "type": "pillar_box", "yaw": 0.0}

    def setUp(self):
        # 正对障碍物接近：所有动作的预测净空都在下降，正增益逃逸不可达。
        self.position = (0.0, 0.0, 2.0)
        self.velocity = (1.0, 0.0, 0.0)
        self.obstacles = [self._obstacle((2.0, 0.0, 2.0), (1.0, 1.0, 3.0))]

    def _min_clearance(self, action):
        return min(minimum_geometry_clearance(point, self.obstacles)
                   for point in predict_trajectory(self.position, self.velocity,
                                                   action, 1.0, 0.1, 1.4, 0.3))

    def test_picks_largest_predicted_minimum_clearance(self):
        # 横向动作比直飞更能保住净空，兜底应选它而非动作 0。
        self.assertGreater(self._min_clearance(1), self._min_clearance(0))
        action, clearance = best_effort_action(
            (0, 1), self.position, self.velocity, self.obstacles,
            lateral_speed=1.4, vertical_speed=0.3, clearance_floor=0.30)
        self.assertEqual(action, 1)
        self.assertAlmostEqual(clearance, self._min_clearance(1), places=6)

    def test_returns_none_below_clearance_floor(self):
        # 最优动作的预测最小净空仍低于风险带 → 交由调用方终止。
        action, clearance = best_effort_action(
            (0, 1), self.position, self.velocity, self.obstacles,
            lateral_speed=1.4, vertical_speed=0.3, clearance_floor=0.50)
        self.assertIsNone(action)
        self.assertAlmostEqual(clearance, self._min_clearance(1), places=6)

    def test_keeps_lowest_action_on_tie(self):
        # 平手按动作号升序取最小，保证确定性。
        action, _ = best_effort_action(
            (0, 1, 2), (0.0, 0.0, 2.0), (0.0, 0.0, 0.0), [])
        self.assertEqual(action, 0)

    def test_empty_candidates(self):
        self.assertEqual(best_effort_action((), self.position, self.velocity,
                                            self.obstacles), (None, None))


if __name__ == "__main__":
    unittest.main()
