#!/usr/bin/env python3

import csv
import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path


STAGE2 = Path(__file__).resolve().parents[3] / "experiments" / "stage2"
sys.path.insert(0, str(STAGE2))

from baseline_protocol import (METHODS, PROTOCOL_VERSION, STANDARD_FIELDS,
                               aggregate_rows, load_manifest, normalize_record,
                               validate_run_metadata)


class BaselineProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest_path = STAGE2 / "manifests" / "test.json"
        cls.manifest = load_manifest(cls.manifest_path)
        cls.scenario = cls.manifest["scenarios"][0]

    def metadata(self, **updates):
        value = {
            "protocol_version": PROTOCOL_VERSION,
            "method": "B0",
            "scenario_id": self.scenario["scenario_id"],
            "split": "test",
            "manifest_sha256": self.manifest["manifest_sha256"],
            "seeds": {"scenario_seed": self.scenario["seed"],
                      "policy_seed": 11, "disturbance_seed": 12},
        }
        value.update(updates)
        return value

    def test_metadata_requires_manifest_hash(self):
        bad = self.metadata(manifest_sha256="bad")
        with self.assertRaises(ValueError):
            validate_run_metadata(bad, self.manifest)

    def test_metadata_requires_manifest_membership(self):
        bad = self.metadata(scenario_id="s-does-not-exist")
        with self.assertRaises(ValueError):
            validate_run_metadata(bad, self.manifest)

    def test_methods_and_standard_fields_are_frozen(self):
        self.assertEqual(set(METHODS), {"B0", "B1", "B2"})
        self.assertEqual(len(STANDARD_FIELDS), len(set(STANDARD_FIELDS)))
        self.assertIn("reset_failure_reason", STANDARD_FIELDS)
        self.assertIn("geometry_collision_events", STANDARD_FIELDS)

    def test_reset_failures_remain_in_aggregation_denominator(self):
        metadata = self.metadata()
        rows = [
            normalize_record({"episode": 0, "result": "success",
                              "done_reason": "success", "reset_ok": 1,
                              "total_reward": 2}, metadata),
            normalize_record({"episode": 1, "result": "failed",
                              "done_reason": "reset_failed", "reset_ok": 0,
                              "reset_failure_reason": "service_timeout"}, metadata),
        ]
        summary = aggregate_rows(rows)
        self.assertEqual(summary["episodes"], 2)
        self.assertEqual(summary["valid_episodes"], 1)
        self.assertEqual(summary["reset_failures"], 1)
        self.assertEqual(summary["success_rate"], 0.5)


if __name__ == "__main__":
    unittest.main()
