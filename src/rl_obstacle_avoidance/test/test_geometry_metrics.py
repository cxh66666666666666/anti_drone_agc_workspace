#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from geometry_metrics import (box_surface_clearance,
                              obstacle_surface_clearance,
                              minimum_geometry_clearance)


class GeometryMetricsTest(unittest.TestCase):
    def test_clearance_to_box_surface(self):
        self.assertAlmostEqual(
            box_surface_clearance((2.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  (1.0, 1.0, 1.0), (0.0, 0.0, 0.0)),
            1.5)

    def test_overlap_is_signed(self):
        self.assertLess(
            box_surface_clearance((0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  (1.0, 1.0, 1.0), (0.6, 0.0, 0.0)),
            0.0)

    def test_empty_scene(self):
        self.assertTrue(math.isinf(minimum_geometry_clearance(
            (0.0, 0.0, 0.0), [])))

    def test_typed_cylinder_clearance(self):
        obstacle = {"type": "cylinder", "center": [0.0, 0.0, 0.0],
                    "size": [2.0, 2.0, 2.0], "yaw": 0.0}
        self.assertAlmostEqual(
            obstacle_surface_clearance((2.0, 0.0, 0.0), obstacle,
                                        (0.0, 0.0, 0.0)), 1.0)

    def test_typed_wall_respects_yaw(self):
        obstacle = {"type": "wall", "center": [0.0, 0.0, 0.0],
                    "size": [4.0, 0.5, 1.0], "yaw": math.pi / 2.0}
        self.assertLess(
            obstacle_surface_clearance((0.0, 1.0, 0.0), obstacle,
                                        (0.0, 0.0, 0.0)), 0.0)


if __name__ == "__main__":
    unittest.main()
