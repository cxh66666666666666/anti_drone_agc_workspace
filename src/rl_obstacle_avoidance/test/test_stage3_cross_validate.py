#!/usr/bin/env python3

import csv
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "experiments"))

from stage3_task5_cross_validate import cross_validate


class Stage3CrossValidationTest(unittest.TestCase):
    def _write(self, root, rows):
        path = root / "step_log.csv"
        fields = ["predicted_h", "geometry_clearance", "filter_margin",
                  "filter_clearance_now"]
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_prediction_matches_observed_geometry(self):
        """同量判据：残差落在时域推导的双侧界内（保守但不过度）。"""
        root = Path(tempfile.mkdtemp(prefix="cross-validate-"))
        try:
            path = self._write(root, [
                # 残差 = 0.2+0.3-0.55 = -0.05（略保守，界内）
                {"predicted_h": "0.2", "geometry_clearance": "0.1",
                 "filter_margin": "0.3", "filter_clearance_now": "0.55"},
                # 残差 = -0.1+0.3-0.15 = +0.05（等于乐观上界，仍算通过）
                {"predicted_h": "-0.1", "geometry_clearance": "-0.05",
                 "filter_margin": "0.3", "filter_clearance_now": "0.15"},
                {"predicted_h": "nan", "geometry_clearance": "0.3",
                 "filter_margin": "0.3", "filter_clearance_now": "0.3"}])
            result = cross_validate(path, tolerance=0.25, optimism_tolerance=0.05,
                                    speed_limit=1.02, lateral_speed=1.4,
                                    vertical_speed=0.3, horizon=1.0)
            self.assertEqual(result["rows"], 3)
            self.assertEqual(result["compared"], 2)
            self.assertEqual(result["skipped"], 1)
            self.assertEqual(result["same_quantity_compared"], 2)
            self.assertEqual(result["same_quantity_skipped"], 1)
            self.assertTrue(result["within_tolerance"])
            self.assertEqual(result["false_safe"], 0)
            self.assertTrue(result["sound"])
            self.assertEqual(result["optimistic"], 0)
            self.assertEqual(result["conservative_exceeded"], 0)
            self.assertAlmostEqual(result["max_speed"], 2.42, places=6)
            self.assertAlmostEqual(result["conservative_bound"], -2.47, places=6)
        finally:
            shutil.rmtree(root)

    def test_false_safe_and_inconsistent_prediction_are_flagged(self):
        """声学性、乐观预测与超保守界必须各自暴露。"""
        root = Path(tempfile.mkdtemp(prefix="cross-validate-"))
        try:
            path = self._write(root, [
                # predicted_h>=0 但真实净空为负 → false_safe（残差 -0.05 界内）
                {"predicted_h": "0.2", "geometry_clearance": "-0.1",
                 "filter_margin": "0.3", "filter_clearance_now": "0.55"},
                # 残差 = 0.0+0.3-0.0 = +0.3 超过乐观上界 → 预测偏乐观
                {"predicted_h": "0.0", "geometry_clearance": "0.5",
                 "filter_margin": "0.3", "filter_clearance_now": "0.0"},
                # 残差 = 0.0+0.3-3.0 = -2.7 低于保守界 -2.47 → 与运动学不自洽
                {"predicted_h": "0.0", "geometry_clearance": "0.5",
                 "filter_margin": "0.3", "filter_clearance_now": "3.0"}])
            result = cross_validate(path, tolerance=0.25, optimism_tolerance=0.05,
                                    speed_limit=1.02, lateral_speed=1.4,
                                    vertical_speed=0.3, horizon=1.0)
            self.assertEqual(result["false_safe"], 1)
            self.assertFalse(result["sound"])
            self.assertFalse(result["within_tolerance"])
            self.assertEqual(result["optimistic"], 1)
            self.assertEqual(result["conservative_exceeded"], 1)
        finally:
            shutil.rmtree(root)

    def test_missing_same_quantity_reference_is_not_faked(self):
        """缺少 filter_clearance_now 时同量判据记为不可比较，不替换为零。"""
        root = Path(tempfile.mkdtemp(prefix="cross-validate-"))
        try:
            path = root / "step_log.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=["predicted_h", "geometry_clearance"])
                writer.writeheader()
                writer.writerow({"predicted_h": "0.2",
                                 "geometry_clearance": "0.1"})
            result = cross_validate(path, tolerance=0.25)
            self.assertEqual(result["compared"], 1)
            self.assertEqual(result["same_quantity_compared"], 0)
            self.assertIsNone(result["max_abs_prediction_error"])
            self.assertIsNone(result["median_prediction_error"])
            self.assertFalse(result["within_tolerance"])
        finally:
            shutil.rmtree(root)


if __name__ == "__main__":
    unittest.main()
