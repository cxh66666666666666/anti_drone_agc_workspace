#!/usr/bin/env python3

import shutil
import sys
import tempfile
import unittest
from pathlib import Path

STAGE2 = Path(__file__).resolve().parents[3] / "experiments" / "stage2"
sys.path.insert(0, str(STAGE2))

from baseline_protocol import load_manifest
from run_smoke import METHODS_ORDER, SEEDS, fixture_rows, verify, write_metadata


class Stage2SmokeTest(unittest.TestCase):
    def test_fixture_verifier_covers_two_seeds_and_reset_failure(self):
        root = Path(tempfile.mkdtemp(prefix="stage2-smoke-test-"))
        try:
            manifest = load_manifest(STAGE2 / "manifests" / "test.json")
            scenarios = manifest["scenarios"][:5]
            for method in METHODS_ORDER:
                for seed in SEEDS:
                    for scenario in scenarios:
                        run = root / (method + "_" + scenario["scenario_id"] +
                                      "_" + str(seed))
                        run.mkdir()
                        metadata = write_metadata(run, method, scenario, manifest, seed)
                        from baseline_protocol import write_csv, STANDARD_FIELDS
                        write_csv(run / "episodes.csv", fixture_rows(
                            metadata, scenario,
                            method == "B2" and seed == 202 and scenario is scenarios[-1]),
                            STANDARD_FIELDS)
            self.assertEqual(verify(root, len(METHODS_ORDER) * len(SEEDS) * 5), [])
        finally:
            shutil.rmtree(root)


if __name__ == "__main__":
    unittest.main()
