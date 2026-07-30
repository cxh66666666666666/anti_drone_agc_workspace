#!/usr/bin/env python3

import os
import sys
import math
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from training_utils import (  # noqa: E402
    action_change_cost,
    avoidance_magnitude,
    constrain_avoidance_action,
    directional_clearance,
    episode_succeeded,
    epsilon_at_step,
    limit_change,
    motion_frame_offset,
    motion_heading,
    proximity_penalty,
    rotate_sectors_to_heading,
    smooth_value,
    update_avoidance_progress,
    validate_distance_thresholds,
)


class TrainingUtilsTest(unittest.TestCase):
    def test_epsilon_decay_reaches_end_by_environment_step(self):
        self.assertEqual(epsilon_at_step(1.0, 0.05, 1000, 0), 1.0)
        self.assertAlmostEqual(epsilon_at_step(1.0, 0.05, 1000, 500),
                               0.05 ** 0.5)
        self.assertEqual(epsilon_at_step(1.0, 0.05, 1000, 1000), 0.05)
        self.assertEqual(epsilon_at_step(1.0, 0.05, 1000, 5000), 0.05)

    def test_action_change_is_penalized_once(self):
        self.assertEqual(action_change_cost(1, 2, 0.05), 0.05)
        self.assertEqual(action_change_cost(2, 2, 0.05), 0.0)

    def test_rule_baseline_offset_scales_with_obstacle_distance(self):
        self.assertAlmostEqual(avoidance_magnitude(3.5, 3.5, 0.3, 1.2), 0.3)
        self.assertAlmostEqual(avoidance_magnitude(0.0, 3.5, 0.3, 1.2), 1.2)
        self.assertAlmostEqual(avoidance_magnitude(1.75, 3.5, 0.3, 1.2), 0.75)

    def test_rule_baseline_smoothing(self):
        self.assertAlmostEqual(smooth_value(0.0, 1.0, 0.8), 0.8)
        self.assertAlmostEqual(smooth_value(0.8, 0.0, 0.8), 0.16)

    def test_offset_change_is_rate_limited(self):
        self.assertAlmostEqual(limit_change(1.2, -1.2, 0.3), 0.9)
        self.assertAlmostEqual(limit_change(0.0, 0.2, 0.3), 0.2)

    def test_sectors_rotate_from_body_to_motion_heading(self):
        sectors = list(range(12))
        rotated = rotate_sectors_to_heading(sectors, 0.0, math.pi)
        self.assertEqual(rotated[6], sectors[0])
        self.assertEqual(rotated[0], sectors[6])

    def test_motion_frame_left_is_perpendicular_to_forward(self):
        dx, dy = motion_frame_offset(0.0, 1.0, math.pi / 2.0)
        self.assertAlmostEqual(dx, -1.0)
        self.assertAlmostEqual(dy, 0.0, places=7)

    def test_motion_heading_retains_fallback_at_low_speed(self):
        self.assertEqual(motion_heading(0.01, 0.01, 1.2), 1.2)
        self.assertAlmostEqual(motion_heading(0.0, 1.0, 0.0), math.pi / 2.0)

    def test_directional_clearance_ignores_rear_obstacle(self):
        sectors = [1.0] * 12
        sectors[0] = 0.1
        self.assertEqual(directional_clearance(sectors), 1.0)
        sectors[6] = 0.2
        self.assertEqual(directional_clearance(sectors), 0.2)

    def test_proximity_penalty_is_bounded(self):
        self.assertEqual(proximity_penalty(0.4, 0.06, 0.15, 0.35), 0.0)
        self.assertLessEqual(
            proximity_penalty(0.06, 0.06, 0.15, 0.35), 0.5)

    def test_avoidance_side_stays_locked_until_forward_clear(self):
        action, side = constrain_avoidance_action(1, 0, 0.2, 0.35)
        self.assertEqual((action, side), (1, 1))
        action, side = constrain_avoidance_action(4, side, 0.2, 0.35)
        self.assertEqual((action, side), (3, 1))
        action, side = constrain_avoidance_action(2, side, 0.5, 0.35)
        self.assertEqual((action, side), (2, -1))

    def test_worst_case_dense_success_still_beats_collision(self):
        worst_dense_step = 0.05 - 0.02 - 0.05 - 0.02 - 0.01
        successful_return = 500 * worst_dense_step + 30.0
        self.assertGreater(successful_return, -30.0)

    def test_only_explicit_avoidance_completion_is_success(self):
        self.assertTrue(episode_succeeded("avoidance_complete"))
        self.assertFalse(episode_succeeded("timeout"))
        self.assertFalse(episode_succeeded("collision"))

    def test_avoidance_completion_requires_encounter_then_stable_clearance(self):
        encountered, clear_steps, done = update_avoidance_progress(
            False, 0, 1.0, 0.1, 0.35, 1.0, 2)
        self.assertEqual((encountered, clear_steps, done), (False, 0, False))

        encountered, clear_steps, done = update_avoidance_progress(
            encountered, clear_steps, 0.2, 0.5, 0.35, 1.0, 2)
        self.assertEqual((encountered, clear_steps, done), (True, 0, False))

        encountered, clear_steps, done = update_avoidance_progress(
            encountered, clear_steps, 0.5, 0.8, 0.35, 1.0, 2)
        self.assertEqual((encountered, clear_steps, done), (True, 1, False))
        encountered, clear_steps, done = update_avoidance_progress(
            encountered, clear_steps, 0.5, 0.8, 0.35, 1.0, 2)
        self.assertEqual((encountered, clear_steps, done), (True, 2, True))

    def test_avoidance_clearance_must_be_consecutive(self):
        encountered, clear_steps, _ = update_avoidance_progress(
            True, 1, 0.3, 0.8, 0.35, 1.0, 2)
        self.assertEqual((encountered, clear_steps), (True, 0))

    def test_collision_threshold_must_be_observable(self):
        validate_distance_thresholds(0.5, 0.6, 1.5, 3.5)
        with self.assertRaises(ValueError):
            validate_distance_thresholds(0.5, 0.3, 1.5, 3.5)


if __name__ == "__main__":
    unittest.main()
