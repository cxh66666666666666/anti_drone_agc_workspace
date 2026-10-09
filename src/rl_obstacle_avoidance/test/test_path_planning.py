#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from path_planning import (  # noqa: E402
    anchor_path_start,
    astar_path,
    build_grid,
    build_typed_grid,
    is_goal_valid,
    nearest_free_point,
    segment_blocked,
    smooth_path,
    world_to_grid,
)


BOUNDS = (-10.0, 10.0, -10.0, 10.0)
RES = 0.5


class PathPlanningTest(unittest.TestCase):
    def test_empty_grid_straight_path(self):
        blocked = build_grid([], BOUNDS, RES, 1.0)
        path = astar_path(blocked, (0.0, 0.0), (4.0, 0.0), BOUNDS, RES)
        self.assertGreaterEqual(len(path), 2)
        self.assertAlmostEqual(path[0][0], 0.0, places=1)
        self.assertAlmostEqual(path[-1][0], 4.0, places=1)

    def test_inflation_blocks_cells_around_pole(self):
        blocked = build_grid([(2.0, 0.0)], BOUNDS, RES, 1.0)
        # 柱心与 1.0 膨胀：路径不应穿过 (2,0) 附近
        path = astar_path(blocked, (0.0, 0.0), (4.0, 0.0), BOUNDS, RES)
        self.assertTrue(path)
        for x, y in path:
            self.assertGreater(math.hypot(x - 2.0, y), 0.9)
        # 绕行路径比直线长
        self.assertGreater(len(path), int(4.0 / RES))

    def test_goal_inside_inflation_is_unreachable(self):
        blocked = build_grid([(2.0, 0.0)], BOUNDS, RES, 1.5)
        path = astar_path(blocked, (0.0, 0.0), (2.0, 0.0), BOUNDS, RES)
        self.assertEqual(path, [])

    def test_nearest_free_point_recovers_blocked_start(self):
        blocked = build_grid([(0.0, 0.0)], BOUNDS, RES, 1.0)
        recovered = nearest_free_point(blocked, (0.0, 0.0), BOUNDS, RES)
        self.assertIsNotNone(recovered)
        self.assertNotIn(world_to_grid(*recovered, BOUNDS, RES), blocked)
        self.assertLessEqual(math.hypot(*recovered), 1.5)

    def test_nearest_free_point_preserves_free_start(self):
        point = (2.2, -1.3)
        self.assertEqual(
            nearest_free_point(set(), point, BOUNDS, RES), point)

    def test_nearest_free_point_returns_none_without_free_cell(self):
        small_bounds = (0.0, 0.5, 0.0, 0.5)
        blocked = {(0, 0), (0, 1), (1, 0), (1, 1)}
        self.assertIsNone(nearest_free_point(
            blocked, (0.0, 0.0), small_bounds, RES))

    def test_smooth_path_shortcuts_collinear_points(self):
        blocked = set()
        raw = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]
        smoothed = smooth_path(raw, blocked, BOUNDS, RES)
        self.assertEqual(len(smoothed), 2)
        self.assertAlmostEqual(smoothed[-1][0], 4.0, places=1)

    def test_smooth_path_does_not_cut_corners(self):
        # 柱在 (2,0)，直连会穿过膨胀区 → 平滑后应保留绕行点
        blocked = build_grid([(2.0, 0.0)], BOUNDS, RES, 1.0)
        raw = [(0.0, -2.0), (1.0, -1.0), (2.0, -1.5), (3.0, -1.0),
               (4.0, -2.0)]
        smoothed = smooth_path(raw, blocked, BOUNDS, RES)
        for i in range(len(smoothed) - 1):
            self.assertFalse(segment_blocked(
                smoothed[i], smoothed[i + 1], blocked, BOUNDS, RES))

    def test_segment_blocked_detects_crossing(self):
        blocked = {world_to_grid(2.0, 0.0, BOUNDS, RES)}
        self.assertTrue(segment_blocked(
            (0.0, 0.0), (4.0, 0.0), blocked, BOUNDS, RES))
        self.assertFalse(segment_blocked(
            (0.0, 3.0), (4.0, 3.0), blocked, BOUNDS, RES))

    def test_goal_validity_checks_clearance(self):
        obstacles = [(3.0, 0.0)]
        self.assertTrue(is_goal_valid((0.0, 0.0), obstacles, 1.5))
        self.assertFalse(is_goal_valid((4.2, 0.0), obstacles, 1.5))

    def test_path_uses_exact_actual_start_before_grid_waypoints(self):
        start = (1.23, -4.56)
        path = anchor_path_start([(1.0, -4.5), (3.0, -4.5)], start)
        self.assertEqual(path[0], start)
        self.assertEqual(path[1], (1.0, -4.5))

    def test_typed_wall_blocks_its_rotated_footprint(self):
        wall = {"type": "wall", "center": [0.0, 0.0, 2.0],
                "size": [4.0, 0.5, 2.5], "yaw": 0.0}
        blocked = build_typed_grid([wall], BOUNDS, RES, 0.5)
        self.assertIn(world_to_grid(0.0, 0.0, BOUNDS, RES), blocked)
        self.assertNotIn(world_to_grid(0.0, 3.0, BOUNDS, RES), blocked)

    def test_typed_cylinder_uses_radius(self):
        cylinder = {"type": "cylinder", "center": [0.0, 0.0, 2.0],
                    "size": [2.0, 2.0, 2.5], "yaw": 0.0}
        blocked = build_typed_grid([cylinder], BOUNDS, RES, 0.2)
        self.assertIn(world_to_grid(0.0, 0.0, BOUNDS, RES), blocked)
        self.assertNotIn(world_to_grid(2.0, 0.0, BOUNDS, RES), blocked)


if __name__ == "__main__":
    unittest.main()
