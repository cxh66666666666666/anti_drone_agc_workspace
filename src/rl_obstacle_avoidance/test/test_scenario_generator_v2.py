#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from scenario_generator_v2 import (DEFAULT_CONFIG, LAYOUT_TYPES, TRAJECTORY_TYPES,
                                   generate_layout, generate_scenarios,
                                   trajectory_frame)


class ScenarioGeneratorV2Test(unittest.TestCase):
    def test_seed_reproduces_trajectory_aware_layout(self):
        first = generate_layout(17, "validation", 8)
        second = generate_layout(17, "validation", 8)
        self.assertEqual(first, second)
        self.assertTrue(first["feasibility"]["feasible"])

    def test_main_splits_cover_supported_trajectories_and_layouts(self):
        scenarios = generate_scenarios("test", 30, seed=19)
        self.assertEqual(
            {item["trajectory"]["type"] for item in scenarios},
            set(TRAJECTORY_TYPES))
        self.assertEqual(
            {item["layout"]["type"] for item in scenarios},
            set(LAYOUT_TYPES))
        self.assertNotIn("clear", {item["layout"]["type"]
                                    for item in scenarios})
        self.assertNotIn("step", {item["trajectory"]["type"]
                                  for item in scenarios})
        self.assertNotIn("helical", {item["trajectory"]["type"]
                                     for item in scenarios})

    def test_obstacles_are_anchored_to_their_own_trajectory(self):
        for index in range(30):
            scenario = generate_layout(23, "test", index)
            trajectory = scenario["trajectory"]
            for obstacle in scenario["obstacles"]:
                point, _, normal = trajectory_frame(
                    trajectory, obstacle["trajectory_progress"])
                dx = obstacle["center"][0] - point[0]
                dy = obstacle["center"][1] - point[1]
                lateral = dx * normal[0] + dy * normal[1]
                self.assertAlmostEqual(
                    lateral, obstacle["lateral_offset_m"], places=2)

    def test_test_split_contains_familiar_and_unseen_sizes(self):
        scenarios = generate_scenarios("test", 30, seed=29)
        familiar = [item for item in scenarios
                    if not item["layout"]["uses_unseen_size"]]
        unseen = [item for item in scenarios
                  if item["layout"]["uses_unseen_size"]]
        self.assertTrue(familiar)
        self.assertTrue(unseen)
        for scenario in unseen:
            for obstacle in scenario["obstacles"]:
                self.assertEqual(
                    obstacle["size"],
                    DEFAULT_CONFIG["test_unseen_sizes"][obstacle["type"]])

    def test_figure_eight_frame_is_continuous_at_cycle_end(self):
        trajectory = generate_layout(
            31, "train", 2)["trajectory"]
        start, _, _ = trajectory_frame(trajectory, 0.0)
        end, _, _ = trajectory_frame(trajectory, 1.0)
        self.assertLess(math.hypot(start[0] - end[0], start[1] - end[1]),
                        1e-9)


if __name__ == "__main__":
    unittest.main()
