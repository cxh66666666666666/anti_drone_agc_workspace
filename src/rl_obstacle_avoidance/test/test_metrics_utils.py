#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from metrics_utils import path_length, recovery_time, summarize_episode


class MetricsUtilsTest(unittest.TestCase):
    def test_path_length_is_planar(self):
        self.assertAlmostEqual(path_length([(0, 0), (3, 4), (3, 8)]), 9.0)

    def test_summary_metrics(self):
        summary = summarize_episode(
            [1.0, 0.5, 0.0], [0.0, 0.5, 1.0],
            [(0, 0), (1, 0), (1, 1)], steady_window_s=0.6)
        self.assertAlmostEqual(summary["rmse"], math.sqrt(1.25 / 3.0))
        self.assertAlmostEqual(summary["max_error"], 1.0)
        self.assertAlmostEqual(summary["steady_error"], 0.25)
        self.assertAlmostEqual(summary["path_length"], 2.0)

    def test_recovery_requires_hold_window(self):
        self.assertIsNone(recovery_time(
            [0.0, 0.2, 0.4], [2.0, 0.5, 1.5], 1.0, 0.5, 0.0))
        self.assertAlmostEqual(recovery_time(
            [0.0, 0.2, 0.6, 0.8], [2.0, 0.5, 0.4, 0.3],
            1.0, 0.5, 0.0), 0.2)


if __name__ == "__main__":
    unittest.main()
