#!/usr/bin/env python3

import json
import os
import shutil
import sys
import tempfile
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from scenario_generator import (build_manifest, check_feasibility,
                                generate_layout, generate_scenarios,
                                sha256_json, verify_manifest, write_manifests)


class ScenarioGeneratorTest(unittest.TestCase):
    def test_seed_reproduces_exact_layout(self):
        first = generate_layout(17, "validation", 4)
        second = generate_layout(17, "validation", 4)
        self.assertEqual(first, second)
        self.assertEqual(first["scenario_id"], second["scenario_id"])
        self.assertTrue(first["feasibility"]["feasible"])

    def test_split_streams_are_distinct(self):
        train = generate_layout(17, "train", 1)
        test = generate_layout(17, "test", 1)
        self.assertNotEqual(train["scenario_id"], test["scenario_id"])
        self.assertNotEqual(train["obstacles"], test["obstacles"])

    def test_infeasible_layout_is_reported(self):
        scenario = {
            "bounds": [-2, 2, -2, 2],
            "planner": {"resolution": 0.5, "safety_margin": 1.0},
            "start": [0, 0, 2], "goal": [1, 0, 2],
            "obstacles": [{"center": [0, 0, 2], "size": [1, 1, 1]}],
        }
        result = check_feasibility(scenario)
        self.assertFalse(result["feasible"])
        self.assertIn("start_too_close", result["reasons"])

    def test_manifest_hash_detects_mutation(self):
        manifest = build_manifest("train", generate_scenarios("train", 2, 9), 9)
        self.assertEqual(manifest["manifest_sha256"], sha256_json(
            {key: value for key, value in manifest.items()
             if key != "manifest_sha256"}))
        self.assertTrue(verify_manifest(manifest))
        manifest["scenarios"][0]["index"] = 99
        self.assertFalse(verify_manifest(manifest))

    def test_write_and_freeze(self):
        output = tempfile.mkdtemp(prefix="scenario-generator-")
        try:
            manifests = write_manifests(
                output, counts={"train": 1, "validation": 1, "test": 1})
            self.assertTrue(verify_manifest(manifests["test"]))
            marker = os.path.join(output, "manifests", ".frozen")
            with open(marker, "w", encoding="utf-8") as stream:
                stream.write("frozen\n")
            with self.assertRaises(PermissionError):
                write_manifests(output, counts={"train": 1, "validation": 1,
                                                 "test": 1}, force=True)
            with open(os.path.join(output, "manifests", "test.json"),
                      encoding="utf-8") as stream:
                self.assertEqual(json.load(stream)["split"], "test")
        finally:
            shutil.rmtree(output)


if __name__ == "__main__":
    unittest.main()
