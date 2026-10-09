#!/usr/bin/env python3

import json
import os
import shutil
import sys
import tempfile
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from scenario_generator_v3 import (DEFAULT_CONFIG, LAYOUT_TYPES, SPLITS,
                                   TRAJECTORY_TYPES, analyze_scenario,
                                   generate_layout, generate_scenarios,
                                   validate_manifests, validate_scenario,
                                   verify_manifest, write_manifests)


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
V2_DIR = os.path.join(REPO_ROOT, "experiments", "stage2_v2")
V3_DIR = os.path.join(REPO_ROOT, "experiments", "stage2_v3")
KNOWN_V2_FAILURES = ("s2v2-d3fa2a37b4", "s2v2-2cfe4ed311",
                     "s2v2-bcf69e8267", "s2v2-849c1729e4")


class ScenarioGeneratorV3Test(unittest.TestCase):
    def test_seed_reproduces_clearance_constrained_layout(self):
        first = generate_layout(17, "validation", 8)
        second = generate_layout(17, "validation", 8)
        self.assertEqual(first, second)
        self.assertTrue(first["feasibility"]["feasible"])
        self.assertTrue(validate_scenario(first)["ok"])

    def test_generated_scenarios_satisfy_clearance_rules(self):
        scenarios = (generate_scenarios("validation", 5, seed=19) +
                     generate_scenarios("test", 10, seed=19))
        for scenario in scenarios:
            result = validate_scenario(scenario)
            self.assertEqual(result["failures"], [], scenario["scenario_id"])
            # 净空必须落在 [下限, 触发避障门限) 之间，否则任务退化为无障碍
            self.assertGreaterEqual(result["obstacle_clearance_min_m"],
                                    DEFAULT_CONFIG["min_obstacle_clearance_m"])
            self.assertLess(result["obstacle_clearance_min_m"],
                            DEFAULT_CONFIG["max_encounter_clearance_m"])
            self.assertGreaterEqual(
                result["start_clearance_min_norm"],
                DEFAULT_CONFIG["reset_min_clearance_norm"])

    def test_main_splits_cover_supported_trajectories_and_layouts(self):
        scenarios = generate_scenarios("test", 30, seed=19)
        self.assertEqual(
            {item["trajectory"]["type"] for item in scenarios},
            set(TRAJECTORY_TYPES))
        self.assertEqual(
            {item["layout"]["type"] for item in scenarios},
            set(LAYOUT_TYPES))

    def test_low_profile_variant_exposes_vertical_escape(self):
        seen_low, seen_tall = False, False
        for index in range(30):
            scenario = generate_layout(23, "train", index)
            result = analyze_scenario(scenario)
            limit = (scenario["altitude"] -
                     DEFAULT_CONFIG["low_profile"]["vertical_margin_m"] - 0.06)
            if scenario["layout"]["low_profile"]:
                seen_low = True
                self.assertTrue(result["vertical_escape"])
                self.assertLessEqual(result["obstacle_top_max_m"], limit)
            else:
                seen_tall = True
                self.assertFalse(result["vertical_escape"])
                self.assertGreater(result["obstacle_top_max_m"],
                                   scenario["altitude"])
        self.assertTrue(seen_low and seen_tall)

    def test_validator_flags_v2_known_start_clearance_failures(self):
        if not os.path.exists(os.path.join(V2_DIR, "manifests")):
            self.skipTest("stage2_v2 scenarios not available")
        report = validate_manifests(V2_DIR)
        self.assertFalse(report["ok"])
        for scenario_id in KNOWN_V2_FAILURES:
            self.assertIn(scenario_id, report["failures"])
            self.assertIn("start_clearance_below_reset_min",
                          report["failures"][scenario_id])
        # 同一批 split/index 在 v3 规则下不再产生出生净空不达标场景
        for split in SPLITS:
            with open(os.path.join(V2_DIR, "manifests", split + ".json"),
                      encoding="utf-8") as stream:
                count = len(json.load(stream)["scenarios"])
            v3 = generate_scenarios(split, min(count, 5), seed=20270116)
            for scenario in v3:
                self.assertTrue(validate_scenario(scenario)["ok"])

    def test_manifest_and_index_hashes_are_consistent(self):
        output_dir = tempfile.mkdtemp(prefix="stage2-v3-test-")
        try:
            manifests = write_manifests(
                output_dir, counts={"train": 3, "validation": 2, "test": 2},
                force=True)
            for split, manifest in manifests.items():
                self.assertTrue(verify_manifest(manifest))
                with open(os.path.join(output_dir, "manifests",
                                       split + ".json"),
                          encoding="utf-8") as stream:
                    stored = json.load(stream)
                self.assertTrue(verify_manifest(stored))
                self.assertEqual(stored["manifest_sha256"],
                                 manifest["manifest_sha256"])
            with open(os.path.join(output_dir, "manifests", "index.json"),
                      encoding="utf-8") as stream:
                index = json.load(stream)
            for split in SPLITS:
                self.assertEqual(index["manifests"][split],
                                 manifests[split]["manifest_sha256"])
            report = validate_manifests(output_dir)
            self.assertTrue(report["ok"])
            self.assertTrue(all(item["index_hash_valid"]
                                for item in report["splits"].values()))
        finally:
            shutil.rmtree(output_dir, ignore_errors=True)

    def test_generated_v3_directory_passes_validation(self):
        if not os.path.exists(os.path.join(V3_DIR, "manifests")):
            self.skipTest("stage2_v3 scenarios not generated yet")
        report = validate_manifests(V3_DIR)
        self.assertEqual(report["failures"], {})
        self.assertEqual(report["splits"]["train"]["count"], 50)
        self.assertEqual(report["splits"]["validation"]["count"], 12)
        self.assertEqual(report["splits"]["test"]["count"], 30)
        self.assertGreater(report["splits"]["train"]["vertical_escape"], 0)


if __name__ == "__main__":
    unittest.main()