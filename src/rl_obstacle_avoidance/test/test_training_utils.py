#!/usr/bin/env python3

import os
import sys
import math
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from training_utils import (  # noqa: E402
    action_change_cost,
    NStepAccumulator,
    action_reversal_cost,
    action_swept_clearance,
    allowed_avoidance_actions,
    avoidance_completion_allowed,
    avoidance_magnitude,
    avoidance_quality_reward,
    constrain_avoidance_action,
    clearance_change_reward,
    cone_min_clearance,
    corridor_aware_clearance,
    directional_clearance,
    episode_is_success,
    episode_succeeded,
    epsilon_at_step,
    limit_change,
    lateral_clearances,
    local_control_profile,
    motion_frame_offset,
    motion_heading,
    path_lookahead_features,
    path_swept_sectors,
    swept_forward_clearance,
    proximity_penalty,
    project_path_progress,
    avoidance_tail_gate,
    retain_obstacle_sectors,
    reset_recovery_timeout,
    rotate_sectors_to_heading,
    swept_heading_sectors,
    smooth_value,
    time_to_collision,
    update_avoidance_event,
    update_avoidance_progress,
    update_settle_progress,
    validate_distance_thresholds,
)


class TrainingUtilsTest(unittest.TestCase):
    def test_parallel_wall_cannot_finish_before_tail_is_seen_and_clear(self):
        self.assertFalse(avoidance_completion_allowed(1.0, False, True))
        self.assertFalse(avoidance_completion_allowed(1.0, True, True))
        self.assertTrue(avoidance_completion_allowed(1.0, True, False))

    def test_safe_center_gap_is_not_treated_as_a_frontal_block(self):
        self.assertEqual(corridor_aware_clearance(
            0.10, 0.10, 0.50, 0.12, 0.13,
            0.06, 0.24, 0.30), 0.50)

    def test_narrow_gap_and_one_sided_wall_still_trigger_avoidance(self):
        self.assertEqual(corridor_aware_clearance(
            0.10, 0.05, 0.50, 0.05, 0.13,
            0.06, 0.24, 0.30), 0.05)
        self.assertEqual(corridor_aware_clearance(
            0.10, 0.10, 0.50, 0.12, 0.50,
            0.06, 0.24, 0.30), 0.10)

    def test_long_reset_return_gets_distance_aware_timeout(self):
        self.assertEqual(reset_recovery_timeout(20.0, 2.0, 1.0), 20.0)
        self.assertEqual(reset_recovery_timeout(20.0, 12.5, 1.0), 30.0)

    def test_obstacle_frame_survives_brief_empty_frame(self):
        remembered = [1.0, 0.08, 1.0]
        self.assertEqual(
            retain_obstacle_sectors([], remembered, 0.2, 0.4),
            remembered)

    def test_obstacle_frame_expires_so_avoidance_can_end(self):
        self.assertEqual(
            retain_obstacle_sectors([1.0] * 3, [1.0, 0.08, 1.0], 0.5, 0.4),
            [1.0] * 3)

    def test_n_step_accumulator_discounts_and_flushes_terminal(self):
        acc = NStepAccumulator(3, 0.9)
        self.assertEqual(acc.append(([0], 0, 1.0, [1], False, [0], 1.0)), [])
        self.assertEqual(acc.append(([1], 0, 2.0, [2], False, [0], 2.0)), [])
        ready = acc.append(([2], 0, 3.0, [3], True, [], 3.0))
        self.assertEqual(len(ready), 3)
        self.assertAlmostEqual(ready[0][2], 1.0 + 0.9 * 2.0 + 0.81 * 3.0)
        self.assertTrue(ready[0][4])
        self.assertAlmostEqual(ready[0][6], 0.9 ** 3)
        self.assertEqual(ready[0][7], 3.0)

    def test_swept_clearance_follows_action_direction(self):
        self.assertEqual(action_swept_clearance(0, 0.4, 0.1, 0.3), 0.4)
        self.assertEqual(action_swept_clearance(3, 0.4, 0.1, 0.3), 0.1)
        self.assertEqual(action_swept_clearance(4, 0.4, 0.1, 0.3), 0.3)
        self.assertEqual(action_swept_clearance(5, 0.4, 0.1, 0.3), 0.1)

    def test_clearance_improvement_has_immediate_feedback(self):
        self.assertGreater(clearance_change_reward(0.08, 0.10, 0.18, 0.2), 0.0)
        self.assertLess(clearance_change_reward(0.10, 0.08, 0.18, 0.2), 0.0)
        self.assertEqual(clearance_change_reward(0.20, 0.21, 0.18, 0.2), 0.0)

    def test_ttc_uses_closing_rate(self):
        self.assertAlmostEqual(time_to_collision(0.16, 0.05, 0.06), 2.0)
        self.assertTrue(math.isinf(time_to_collision(0.16, -0.01, 0.06)))

    def test_direction_reversal_cost_is_stronger(self):
        self.assertEqual(action_reversal_cost(1, 2, 0.01, 0.05), 0.05)
        self.assertEqual(action_reversal_cost(1, 3, 0.01, 0.05), 0.01)

    def test_path_projection_uses_arc_progress(self):
        progress, error = project_path_progress(
            (2.0, 1.0), [(0, 0), (2, 0), (2, 2)])
        self.assertAlmostEqual(progress, 3.0)
        self.assertAlmostEqual(error, 0.0)

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

    def test_sector_rotation_keeps_boundary_obstacle_visible(self):
        sectors = [1.0] * 12
        sectors[4] = 0.1
        sector_size = 2.0 * math.pi / len(sectors)

        just_before = rotate_sectors_to_heading(
            sectors, 0.0, 0.49 * sector_size)
        just_after = rotate_sectors_to_heading(
            sectors, 0.0, 0.51 * sector_size)

        self.assertEqual(directional_clearance(just_before, math.pi / 3.0),
                         0.1)
        self.assertEqual(directional_clearance(just_after, math.pi / 3.0),
                         0.1)

    def test_reverse_motion_detects_body_rear_obstacle_at_arbitrary_yaw(self):
        sectors = [1.0] * 12
        sectors[0] = 0.08
        body_yaw = math.radians(30.0)
        heading_world = body_yaw + math.pi

        rotated = rotate_sectors_to_heading(
            sectors, body_yaw, heading_world)

        self.assertEqual(directional_clearance(rotated, math.pi / 3.0),
                         0.08)

    def test_heading_wrap_does_not_hide_forward_obstacle(self):
        sectors = [1.0] * 12
        sectors[6] = 0.12

        below_pi = rotate_sectors_to_heading(
            sectors, math.pi - 0.01, -math.pi + 0.01)
        above_minus_pi = rotate_sectors_to_heading(
            sectors, -math.pi + 0.01, math.pi - 0.01)

        self.assertEqual(directional_clearance(below_pi, math.pi / 3.0),
                         0.12)
        self.assertEqual(directional_clearance(above_minus_pi,
                                               math.pi / 3.0), 0.12)

    def test_curve_sweep_detects_obstacle_outside_current_forward_cone(self):
        sectors = [1.0] * 12
        sectors[9] = 0.1  # body-left now, forward after a left turn
        current = rotate_sectors_to_heading(sectors, 0.0, 0.0)
        swept = swept_heading_sectors(
            sectors, 0.0, 0.0, math.pi / 2.0)
        self.assertEqual(directional_clearance(current, math.pi / 3.0), 1.0)
        self.assertEqual(directional_clearance(swept, math.pi / 3.0), 0.1)

    def test_curve_sweep_uses_shortest_heading_path_across_pi(self):
        sectors = [1.0] * 12
        sectors[6] = 0.2
        swept = swept_heading_sectors(
            sectors, math.pi, math.pi - 0.2, -math.pi + 0.2)
        self.assertEqual(directional_clearance(swept, math.pi / 3.0), 0.2)

    def test_straight_sweep_matches_single_heading_rotation(self):
        sectors = [float(index) for index in range(12)]
        expected = rotate_sectors_to_heading(sectors, 0.3, 1.1)
        self.assertEqual(
            swept_heading_sectors(sectors, 0.3, 1.1, 1.1), expected)

    def test_s_curve_sweep_keeps_intermediate_turn(self):
        sectors = [1.0] * 12
        sectors[9] = 0.1
        direct = path_swept_sectors(sectors, 0.0, [0.0, 0.0])
        curved = path_swept_sectors(
            sectors, 0.0, [0.0, math.pi / 2.0, 0.0])
        self.assertEqual(directional_clearance(direct, math.pi / 3.0), 1.0)
        self.assertEqual(directional_clearance(curved, math.pi / 3.0), 0.1)

    def test_path_sweep_detects_obstacle_between_actual_and_motion_heading(self):
        sectors = [1.0] * 12
        sectors[9] = 0.1  # 位于 actual(+90°) 与 motion(0°) 中间靠转弯内侧
        single = path_swept_sectors(sectors, 0.0, [0.0])
        swept = path_swept_sectors(
            sectors, 0.0, [math.pi / 2.0, 0.0])
        self.assertEqual(directional_clearance(single, math.pi / 3.0), 1.0)
        self.assertEqual(directional_clearance(swept, math.pi / 3.0), 0.1)

    def test_swept_forward_clearance_checks_each_heading_cone(self):
        sectors = [1.0] * 12
        sectors[9] = 0.1
        self.assertEqual(
            swept_forward_clearance(sectors, 0.0, [0.0, math.pi / 2.0]),
            0.1)

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

    def test_lateral_clearances_keep_left_and_right_separate(self):
        sectors = [1.0] * 12
        sectors[8] = 0.1
        sectors[3] = 0.2
        self.assertEqual(lateral_clearances(sectors), (0.1, 0.2))

    def test_cone_min_clearance_matches_sector_angles(self):
        # 12 扇区时索引 5/6 落在正前 ±30° 锥内，索引 7 (45°) 在锥外
        self.assertEqual(
            cone_min_clearance([1.0] * 12, 0.0, math.pi / 6.0), 1.0)
        sectors = [1.0] * 12
        sectors[6] = 0.5
        self.assertEqual(cone_min_clearance(sectors, 0.0, math.pi / 6.0), 0.5)
        self.assertEqual(cone_min_clearance([], 0.0), 1.0)
        sectors = [1.0] * 12
        sectors[7] = 0.3
        self.assertEqual(cone_min_clearance(sectors, 0.0, math.pi / 6.0), 1.0)

    def test_figure_eight_far_lobe_obstacle_does_not_trigger(self):
        # 未来叶障碍 3.0 m；d_0/d_1 = 2.4-0.65 = 1.75 m 下限、d_2 = 2.0 m，
        # 沿原航迹最多走 2.0+0.65 = 2.65 m < 3.0 m，够不到 → 不触发。
        sectors = [1.0] * 12
        sectors[6] = 0.3  # 方位 15°，正前锥内，3.0 m
        features, avoid_needed = path_lookahead_features(
            sectors, 0.0, [0.0, 0.0, 0.0], [1.75, 1.75, 2.0], 10.0, 0.65)
        self.assertEqual(features[0], 0.0)
        self.assertEqual(features[4], 0.0)
        self.assertFalse(avoid_needed)

    def test_side_corridor_keeps_straight_flight(self):
        # mirror_pair：障碍方位约 41°（用 45° 扇区近似），2.66 m
        sectors = [1.0] * 12
        sectors[7] = 0.266
        features, avoid_needed = path_lookahead_features(
            sectors, 0.0, [0.0, 0.0, 0.0], [2.4, 2.3, 2.0], 10.0, 0.65)
        self.assertEqual(features[0], 0.0)  # 41° 在 ±30° 正前锥外
        self.assertEqual(features[1], 0.0)  # 侧方够不到，记满量程
        self.assertEqual(features[4], 0.0)
        self.assertFalse(avoid_needed)

    def test_dense_corridor_obstacles_stay_outside_reach(self):
        sectors = [1.0] * 12
        sectors[7] = 0.19  # 左前 45°，1.9 m
        sectors[4] = 0.19  # 右前 -45°，1.9 m
        features, avoid_needed = path_lookahead_features(
            sectors, 0.0, [0.0, 0.0, 0.0], [2.4, 2.3, 2.0], 10.0, 0.65)
        self.assertEqual(features[4], 0.0)
        self.assertFalse(avoid_needed)

    def test_frontal_block_triggers_avoidance(self):
        sectors = [1.0] * 12
        sectors[6] = 0.15  # 正前 1.5 m
        features, avoid_needed = path_lookahead_features(
            sectors, 0.0, [0.0, 0.0, 0.0], [1.75, 1.75, 1.75], 10.0, 0.65)
        self.assertGreater(features[0], 0.0)
        self.assertEqual(features[4], 1.0)
        self.assertTrue(avoid_needed)

    def test_true_corridor_triggers_avoidance(self):
        sectors = [1.0] * 12
        sectors[7] = 0.04  # 左前 0.4 m
        sectors[4] = 0.04  # 右前 0.4 m
        features, avoid_needed = path_lookahead_features(
            sectors, 0.0, [0.0, 0.0, 0.0], [1.75, 1.75, 1.75], 10.0, 0.65)
        self.assertGreater(features[1], 0.0)  # 带宽 0.4 m < body_margin
        self.assertEqual(features[4], 1.0)
        self.assertTrue(avoid_needed)

    def test_all_zero_sectors_are_conservative(self):
        features, avoid_needed = path_lookahead_features(
            [0.0] * 12, 0.0, [0.0, 0.0, 0.0], [1.75, 1.75, 1.75], 10.0, 0.65)
        self.assertEqual(features[0], 1.0)
        self.assertEqual(features[1], 1.0)
        self.assertEqual(features[4], 1.0)
        self.assertTrue(avoid_needed)

    def test_fully_clear_sectors_need_no_avoidance(self):
        features, avoid_needed = path_lookahead_features(
            [1.0] * 12, 0.0, [0.0, 0.0, 0.0], [1.75, 1.75, 1.75], 10.0, 0.65)
        self.assertEqual(features, [0.0, 0.0, 1.0, 1.0, 0.0])
        self.assertFalse(avoid_needed)

    def test_empty_sectors_are_treated_as_clear(self):
        # 空扇区不报错：三个锥都返回 1.0，等价"全清"
        features, avoid_needed = path_lookahead_features(
            [], 0.0, [0.0], [1.0], 10.0, 0.65)
        self.assertEqual(features, [0.0, 0.0, 1.0, 1.0, 0.0])
        self.assertFalse(avoid_needed)

    def test_lookahead_distance_is_paired_with_its_own_frame(self):
        # 同一障碍只在第 3 帧落到正前方，只有该帧的距离决定是否可达
        sectors = [1.0] * 12
        sectors[6] = 0.3  # 3.0 m
        short, _ = path_lookahead_features(
            sectors, 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 2.0], 10.0, 0.65)
        self.assertEqual(short[0], 0.0)
        long, _ = path_lookahead_features(
            sectors, 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 3.0], 10.0, 0.65)
        self.assertGreater(long[0], 0.0)

    def test_lookahead_rejects_invalid_inputs(self):
        sectors = [1.0] * 12
        with self.assertRaises(ValueError):
            path_lookahead_features(
                sectors, 0.0, [0.0], [1.0, 2.0], 10.0, 0.65)
        with self.assertRaises(ValueError):
            path_lookahead_features(sectors, 0.0, [], [], 10.0, 0.65)
        with self.assertRaises(ValueError):
            path_lookahead_features(sectors, 0.0, [0.0], [1.0], 0.0, 0.65)
        with self.assertRaises(ValueError):
            path_lookahead_features(sectors, 0.0, [0.0], [1.0], 10.0, -1.0)
        with self.assertRaises(ValueError):
            path_lookahead_features(
                sectors, 0.0, [0.0], [1.0], 10.0, 0.65, 0.0)

    def test_lookahead_features_are_bounded_and_finite(self):
        sectors = [1.0, 0.05, 0.5, 0.9, 0.2, 1.0,
                   0.3, 0.04, 1.0, 0.8, 0.6, 1.0]
        features, avoid_needed = path_lookahead_features(
            sectors, 0.3, [0.0, 0.4, -0.2], [1.75, 2.3, 2.0], 10.0, 0.65)
        self.assertEqual(len(features), 5)
        for value in features:
            self.assertTrue(math.isfinite(value))
            self.assertGreaterEqual(value, 0.0)
            self.assertLessEqual(value, 1.0)
        self.assertIsInstance(avoid_needed, bool)

    def test_proximity_penalty_is_bounded(self):
        self.assertEqual(proximity_penalty(0.4, 0.06, 0.15, 0.35), 0.0)
        self.assertLessEqual(
            proximity_penalty(0.06, 0.06, 0.15, 0.35), 0.5)

    def test_danger_step_has_negative_reward(self):
        base_reward = 0.02 - 0.01
        penalty = proximity_penalty(
            0.05, 0.06, 0.10, 0.18, 0.10, 0.50)
        self.assertLess(base_reward - penalty, 0.0)

    def test_emergency_zone_disallows_forward_motion(self):
        self.assertEqual(
            allowed_avoidance_actions(0.09, 0.14, 0.24, 0.30, 0), [3, 4])
        self.assertEqual(
            allowed_avoidance_actions(0.09, 0.14, 0.24, 0.30, 1), [3])
        self.assertEqual(
            allowed_avoidance_actions(0.20, 0.14, 0.24, 0.30, -1), [2, 4])
        self.assertEqual(
            allowed_avoidance_actions(0.20, 0.14, 0.24, 0.30, 0),
            [1, 3, 2, 4])
        self.assertEqual(
            allowed_avoidance_actions(0.35, 0.14, 0.24, 0.30, 0), [0])

    def test_warning_zone_starts_early_without_vertical_or_pause_action(self):
        self.assertEqual(
            allowed_avoidance_actions(0.28, 0.14, 0.24, 0.30, 0),
            [0, 1, 2])

    def test_multi_obstacle_action_mask_uses_side_clearance(self):
        self.assertEqual(
            allowed_avoidance_actions(
                0.10, 0.14, 0.24, 0.30, 0, 0.08, 0.20), [4])
        self.assertEqual(
            allowed_avoidance_actions(
                0.10, 0.14, 0.24, 0.30, 0, 0.08, 0.09), [5])
        self.assertEqual(
            allowed_avoidance_actions(
                0.20, 0.14, 0.24, 0.30, 0, 0.08, 0.09), [0])
        self.assertIn(
            0, allowed_avoidance_actions(
                0.35, 0.14, 0.24, 0.30, 0, 0.08, 0.09))

    def test_omni_face_emergency_unlocks_avoidance_when_cone_clear(self):
        # 前方锥开阔但全向已贴脸：不再锁死 [0]，解锁两侧绕行动作。
        self.assertEqual(
            allowed_avoidance_actions(
                1.0, 0.14, 0.24, 0.30, 0, 0.30, 0.30, omni_min=0.10),
            [1, 3, 2, 4])

    def test_omni_min_large_keeps_forward_only_in_clear_zone(self):
        # 全向距离足够时，开阔锥仍只允许直行 [0]。
        self.assertEqual(
            allowed_avoidance_actions(
                1.0, 0.14, 0.24, 0.30, 0, 0.30, 0.30, omni_min=0.50),
            [0])

    def test_control_profile_never_pauses_outside_emergency(self):
        self.assertEqual(
            local_control_profile(0.35, 0.14, 0.24, 0.30),
            (1.0, False, 0))
        self.assertEqual(
            local_control_profile(0.27, 0.14, 0.24, 0.30),
            (0.8, False, 0))
        self.assertEqual(
            local_control_profile(0.20, 0.14, 0.24, 0.30),
            (0.65, False, 0))

    def test_emergency_zone_slows_without_pausing_or_reversing(self):
        self.assertEqual(
            local_control_profile(0.10, 0.14, 0.24, 0.30),
            (0.3, False, 0))

    def test_tail_completion_precedes_return_to_path_release(self):
        self.assertEqual(
            avoidance_tail_gate(True, True, 0.13, 0.12, 0.16),
            (True, True))
        self.assertEqual(
            avoidance_tail_gate(True, True, 0.16, 0.12, 0.16),
            (True, False))
        self.assertEqual(
            avoidance_tail_gate(True, False, 0.20, 0.12, 0.16),
            (False, True))
        self.assertEqual(
            avoidance_tail_gate(False, False, 1.0, 0.12, 0.16),
            (False, False))

    def test_avoidance_side_stays_locked_until_forward_clear(self):
        action, side = constrain_avoidance_action(1, 0, 0.2, 0.35)
        self.assertEqual((action, side), (1, 1))
        action, side = constrain_avoidance_action(4, side, 0.2, 0.35)
        self.assertEqual((action, side), (3, 1))
        action, side = constrain_avoidance_action(2, side, 0.5, 0.35)
        self.assertEqual((action, side), (2, -1))

    def test_invalid_action_is_rejected_instead_of_falling_back(self):
        with self.assertRaises(ValueError):
            constrain_avoidance_action(6, 0, 0.2, 0.35)

    def test_only_explicit_completion_is_success(self):
        self.assertTrue(episode_succeeded("avoidance_complete"))
        self.assertTrue(episode_succeeded("goal_reached"))
        self.assertTrue(episode_succeeded("trajectory_complete"))
        self.assertFalse(episode_succeeded("timeout"))
        self.assertFalse(episode_succeeded("collision"))

    def test_timeout_is_always_a_failure(self):
        # 新口径：超时一律失败，不再按「是否发生碰撞」区分
        self.assertFalse(episode_is_success("timeout", 0))
        self.assertFalse(episode_is_success("timeout", collision_event_count=0))

    def test_timeout_with_collision_event_is_a_failure(self):
        self.assertFalse(episode_is_success("timeout", 1))

    def test_explicit_completion_is_success_regardless_of_events(self):
        self.assertTrue(episode_is_success("trajectory_complete", 0))
        self.assertTrue(episode_is_success("avoidance_complete", 0))
        self.assertTrue(episode_is_success("goal_reached", 0))

    def test_other_terminations_are_failures(self):
        for reason in ("collision", "shield_infeasible", "diverged",
                       "low_altitude", "perception_timeout", "timeout"):
            self.assertFalse(episode_is_success(reason, 0))

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

    def test_event_clearance_does_not_wait_for_tracking_recovery(self):
        state = update_avoidance_event(
            True, 0, 0, 0.10, None, 0.20, 5.0, 9.0,
            0.18, 1.0, 2, 5, 2.0)
        state = update_avoidance_event(
            *state[:5], 0.20, 5.0, 9.0, 0.18, 1.0, 2, 5, 2.0)
        self.assertTrue(state[5])

    def test_event_cannot_complete_while_tail_or_forward_progress_is_unsafe(self):
        state = update_avoidance_event(
            True, 1, 0, 0.10, None, 0.20, 0.2, 9.0,
            0.18, 1.0, 2, 5, 2.0, False)
        self.assertEqual((state[0], state[1], state[2], state[5]),
                         (True, 0, 0, False))

    def test_event_completion_uses_strict_forward_clearance(self):
        # The wide trigger cone may still see the obstacle at the side/rear;
        # once the strict forward cone is clear, the event can complete.
        state = update_avoidance_event(
            True, 0, 0, 0.10, None, 0.15, 0.2, 9.0,
            0.18, 1.0, 2, 5, 2.0, True, 0.25)
        state = update_avoidance_event(
            *state[:5], 0.15, 0.2, 9.0, 0.18, 1.0, 2, 5, 2.0,
            True, 0.25)
        self.assertTrue(state[5])

    def test_avoidance_clearance_must_be_consecutive(self):
        encountered, clear_steps, _ = update_avoidance_progress(
            True, 1, 0.3, 0.8, 0.35, 1.0, 2)
        self.assertEqual((encountered, clear_steps), (True, 0))

    def test_avoidance_quality_rewards_clear_and_precise_recovery(self):
        excellent, excellent_quality = avoidance_quality_reward(
            0.14, 0.0, 0.12, 0.06, 1.0, 8.0, 10.0)
        risky, risky_quality = avoidance_quality_reward(
            0.08, 0.8, 0.12, 0.06, 1.0, 8.0, 10.0)
        self.assertAlmostEqual(excellent, 18.0)
        self.assertAlmostEqual(excellent_quality, 1.0)
        self.assertGreater(risky, 8.0)
        self.assertLess(risky_quality, excellent_quality)

    def test_avoidance_quality_never_rewards_collision_clearance(self):
        reward, quality = avoidance_quality_reward(
            0.05, 1.0, 0.12, 0.06, 1.0, 8.0, 10.0)
        self.assertEqual((reward, quality), (8.0, 0.0))

    def test_avoidance_events_rearm_on_new_danger_cluster(self):
        state = update_avoidance_event(
            False, 0, 0, 1.0, None, 0.10, 0.2, 10.0,
            0.18, 1.0, 2, 5, 2.0)
        self.assertTrue(state[0])
        state = update_avoidance_event(
            *state[:5], 0.20, 0.2, 9.0, 0.18, 1.0, 2, 5, 2.0)
        state = update_avoidance_event(
            *state[:5], 0.20, 0.2, 9.0, 0.18, 1.0, 2, 5, 2.0)
        self.assertEqual((state[0], state[2], state[5]), (False, 1, True))

        rearmed = update_avoidance_event(
            *state[:5], 0.10, 0.2, 8.0, 0.18, 1.0, 2, 5, 2.0)
        self.assertTrue(rearmed[0])

    def test_vertical_hold_action_does_not_change_side_lock(self):
        self.assertEqual(
            constrain_avoidance_action(5, 1, 0.10, 0.18), (5, 1))

    def test_collision_threshold_must_be_observable(self):
        # 合法: 包络接触(0.0) <= 紧急(0.6) < 风险带(1.5) < 安全(3.5)
        validate_distance_thresholds(0.5, 1.5, 0.6, 3.5, contact_dist=0.0)
        with self.assertRaises(ValueError):
            validate_distance_thresholds(0.5, 0.6, 1.5, 3.5)

    def test_contact_dist_must_not_exceed_emergency_distance(self):
        validate_distance_thresholds(0.5, 1.5, 0.6, 3.5, contact_dist=0.6)
        with self.assertRaises(ValueError):
            validate_distance_thresholds(0.5, 1.5, 0.6, 3.5, contact_dist=0.61)

    def test_settle_progress_requires_low_speed_and_stable_position(self):
        steps = update_settle_progress(
            (1.0, 2.0), 0, (1.0, 2.0), 0.1, 0.15, 0.2)
        self.assertEqual(steps, 1)
        steps = update_settle_progress(
            (1.0, 2.0), steps, (1.1, 2.0), 0.1, 0.15, 0.2)
        self.assertEqual(steps, 2)
        steps = update_settle_progress(
            (1.0, 2.0), steps, (1.1, 2.0), 0.3, 0.15, 0.2)
        self.assertEqual(steps, 0)


if __name__ == "__main__":
    unittest.main()
