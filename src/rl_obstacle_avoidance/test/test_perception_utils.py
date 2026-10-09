#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from perception_utils import points_to_sectors  # noqa: E402


class PerceptionUtilsTest(unittest.TestCase):
    def test_front_point_maps_to_front_sector(self):
        sectors = points_to_sectors(
            np.array([[2.0, 0.0, 0.0]]), 12, 0.3, 10.0, 1.5)
        self.assertAlmostEqual(sectors[6], 0.2)
        self.assertEqual(sum(value < 1.0 for value in sectors), 1)

    def test_filters_height_and_range_but_keeps_collision_band(self):
        points = np.array([
            [0.2, 0.0, 0.0],
            [0.35, 0.0, 0.0],
            [0.55, 0.0, 0.0],
            [1.0, 0.0, 2.0],
        ])
        sectors = points_to_sectors(points, 12, 0.4, 10.0, 1.5)
        self.assertAlmostEqual(sectors[6], 0.055)


if __name__ == "__main__":
    unittest.main()
