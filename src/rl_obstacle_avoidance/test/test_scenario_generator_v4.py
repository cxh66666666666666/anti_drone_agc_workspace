#!/usr/bin/env python3
"""Stage-2 v4 场景生成器：组合覆盖、产物一致性与两阶段出生净空求解。

不依赖 ros master / Gazebo；读盘测试在产物缺失时跳过。
"""

import json
import os
import random
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from scenario_generator_v2 import (LAYOUT_TYPES, TRAJECTORY_TYPES,
                                   _obstacles_for_layout, trajectory_frame,
                                   verify_manifest)
from scenario_generator_v4 import (COLLISION_DIST_M, COMBO_COUNT, DEFAULT_CONFIG,
                                   EXCLUDED_COMBOS, SPAWN_CLEARANCE_TARGET_M,
                                   TRAIN_MIN_BAND_M, TRAIN_MIN_CLEARANCE_M,
                                   combo_parts, generate_layout,
                                   passability_metrics, validate_scenario,
                                   _obstacle_specs, _solve_spawn_clearance)


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
MANIFEST_DIR = os.path.join(REPO_ROOT, "experiments", "stage2_v4", "manifests")
SPLITS = ("train", "validation", "test")
LATERAL_SEED = 20270116
# 手算用例用：x 从 -1 到 1 的直线航迹（法向恒为 +y）。
LINEAR_TRAJECTORY = {
    "type": "linear",
    "start": [-1.0, 0.0, 2.0],
    "end": [1.0, 0.0, 2.0],
    "duration_s": 40.0,
    "loop": False,
}
VALIDATION_KEYS = {
    "spawn_clearance_m", "spawn_clearance_target_m", "min_path_clearance_m",
    "band_width_m", "band_center_m", "best_min_clearance_m", "difficulty",
    "lateral_offsets", "combo", "obstacle_count", "spawn_shift",
    "spawn_progress_adjustments",
}


def combo_key(scenario):
    return (scenario["layout"]["type"], scenario["trajectory"]["type"])


def pillar(lateral, size=0.40):
    """手算用例用的立柱（轴对齐 box 近似，与 _build_obstacles 一致）。"""
    return {"id": "p", "group": "a", "type": "pillar_box",
            "center": [0.0, lateral, 2.0], "size": [size, size, 2.8],
            "yaw": 0.0, "trajectory_progress": 0.5,
            "lateral_offset_m": lateral}


class ScenarioGeneratorV4Test(unittest.TestCase):
    def test_combo_parts_covers_all_15_combinations(self):
        # 显式组合枚举：0..14 一一对应，且 (layout, trajectory) 组合两两不同
        combos = [combo_parts(index)[0] for index in range(COMBO_COUNT)]
        self.assertEqual(combos, list(range(COMBO_COUNT)))
        pairs = {(combo_parts(index)[1], combo_parts(index)[2])
                 for index in range(COMBO_COUNT)}
        self.assertEqual(len(pairs), COMBO_COUNT)
        self.assertEqual({layout for layout, _ in pairs}, set(LAYOUT_TYPES))
        self.assertEqual({traj for _, traj in pairs}, set(TRAJECTORY_TYPES))

    def test_two_phase_solver_fixes_dense_corridor_figure_eight(self):
        # test split index 13 = dense_corridor + figure_eight 的 extreme 版：
        # 阶段 1 无解，需阶段 2 微调（train split 改用 train 版参数，另测）。
        scenario = generate_layout(20270116, "test", 13)
        self.assertEqual(scenario["layout"]["type"], "dense_corridor")
        self.assertEqual(scenario["trajectory"]["type"], "figure_eight")
        self.assertEqual(scenario["layout"]["difficulty"], "extreme")
        validation = scenario["layout_validation"]
        self.assertGreaterEqual(validation["spawn_clearance_m"],
                                SPAWN_CLEARANCE_TARGET_M)
        self.assertEqual(validation["spawn_shift"], 0.0)
        adjustments = validation["spawn_progress_adjustments"]
        self.assertTrue(any(abs(value) > 1e-9 for value in adjustments))
        for value in adjustments:
            self.assertLessEqual(abs(value), 0.05)

    def test_solver_fails_fast_when_target_unreachable(self):
        # 目标不可达时必须抛错，绝不返回降级布局
        scenario = generate_layout(20270116, "test", 13)
        specs = _obstacle_specs(scenario["trajectory"], scenario["layout"]["type"],
                                scenario["layout"]["variant"],
                                scenario["layout"]["difficulty"])
        sizes = [obstacle["size"] for obstacle in scenario["obstacles"]]
        with self.assertRaises(RuntimeError):
            _solve_spawn_clearance(
                scenario["trajectory"], specs, sizes, 2.0, target_m=99.0,
                layout=scenario["layout"]["type"],
                variant=scenario["layout"]["variant"])


class PassabilityMetricsTest(unittest.TestCase):
    def test_matches_hand_calculation_and_is_deterministic(self):
        # 对称双柱 ±1.40（柱半宽 0.20 + 机体半宽 0.35）：偏移 o 处全程净空
        # = 1.40 − 0.55 − |o| = 0.85 − |o|；门槛 0.65 ⇒ |o| ≤ 0.20。
        obstacles = [pillar(1.40), pillar(-1.40)]
        metrics = dict(threshold_m=0.65, offset_range_m=2.5,
                       offset_step_m=0.05, samples=120)
        first = passability_metrics(LINEAR_TRAJECTORY, obstacles, **metrics)
        self.assertEqual(first, passability_metrics(LINEAR_TRAJECTORY, obstacles,
                                                    **metrics))
        self.assertEqual(first["band_width_m"], 0.40)
        self.assertEqual(first["band_center_m"], 0.0)
        self.assertEqual(first["best_min_clearance_m"], 0.85)

    def test_reports_no_band_when_obstacle_pinches_trajectory(self):
        # 对称双柱 ±0.95（extreme 版）在 ±0.5 搜索范围内无可行带，
        # 最优偏移净空 = 0.95 − 0.55 = 0.40 < 门槛 0.65。
        obstacles = [pillar(0.95), pillar(-0.95)]
        metrics = passability_metrics(LINEAR_TRAJECTORY, obstacles,
                                      threshold_m=0.65, offset_range_m=0.5,
                                      offset_step_m=0.05, samples=120)
        self.assertEqual(metrics["band_width_m"], 0.0)
        self.assertIsNone(metrics["band_center_m"])
        self.assertEqual(metrics["best_min_clearance_m"], 0.40)


class ScenarioDifficultyTest(unittest.TestCase):
    def test_layout_validation_records_passability_and_difficulty(self):
        scenario = generate_layout(20270116, "train", 2)
        validation = scenario["layout_validation"]
        self.assertEqual(set(validation), VALIDATION_KEYS)
        self.assertEqual(scenario["layout"]["difficulty"], "train")
        self.assertEqual(validation["difficulty"], "train")

    def test_train_split_marks_every_scenario_train(self):
        for index in (0, 2, 3, 5, 7, 8, 10):
            scenario = generate_layout(20270116, "train", index)
            self.assertEqual(scenario["layout"]["difficulty"], "train", index)
            self.assertEqual(scenario["layout_validation"]["difficulty"],
                             "train", index)

    def test_train_corridor_layouts_meet_passability_gate(self):
        # train 版夹缝布局必须满足门槛。figure_eight（自交轨迹）无论多宽的
        # 夹缝都无法满足常值横向偏移门槛，属已知遗留问题，不在本用例内。
        for index in (2, 7, 3, 8):
            scenario = generate_layout(20270116, "train", index)
            validation = scenario["layout_validation"]
            self.assertIn(scenario["layout"]["type"],
                          ("mirror_pair", "dense_corridor"))
            self.assertGreaterEqual(validation["band_width_m"],
                                    TRAIN_MIN_BAND_M, index)
            self.assertGreaterEqual(validation["best_min_clearance_m"],
                                    TRAIN_MIN_CLEARANCE_M, index)

    def test_train_split_rejects_unpassable_layout(self):
        # extreme 版（test split 偶 variant）换成 train split 后必须被拒绝，
        # 不允许静默降级。
        scenario = generate_layout(20270116, "test", 2)
        self.assertEqual(scenario["layout"]["difficulty"], "extreme")
        self.assertLess(scenario["layout_validation"]["band_width_m"],
                        TRAIN_MIN_BAND_M)
        scenario["split"] = "train"
        with self.assertRaises(RuntimeError) as context:
            validate_scenario(scenario)
        self.assertIn("航迹可通行性未达标", str(context.exception))

    def test_extreme_version_only_in_test_split(self):
        # test split 偶 variant 用 extreme 版（不过 train 门槛校验）
        extreme = generate_layout(20270116, "test", 2)
        self.assertEqual(extreme["layout"]["type"], "mirror_pair")
        self.assertEqual(extreme["layout"]["difficulty"], "extreme")
        self.assertEqual(extreme["layout_validation"]["difficulty"], "extreme")
        self.assertEqual([obstacle["lateral_offset_m"]
                          for obstacle in extreme["obstacles"]],
                         [0.95, -0.95])
        # test split 奇 variant 用 train 版
        self.assertEqual(generate_layout(20270116, "test", 17)["layout"]["difficulty"],
                         "train")
        # train split 的 mirror_pair 用 train 版 ±1.75
        train = generate_layout(20270116, "train", 2)
        self.assertEqual([obstacle["lateral_offset_m"]
                          for obstacle in train["obstacles"]],
                         [1.75, -1.75])

    def test_train_dense_corridor_drops_center_pillar(self):
        # train 版 dense_corridor = 6 面墙（±1.90）且无中心柱；
        # extreme 版保留 7 个障碍（含 0.35 中心柱）。
        train = generate_layout(20270116, "train", 3)
        extreme = generate_layout(20270116, "test", 3)
        self.assertEqual(len(train["obstacles"]), 6)
        self.assertTrue(all(obstacle["type"] == "wall"
                            for obstacle in train["obstacles"]))
        self.assertEqual(sorted({abs(obstacle["lateral_offset_m"])
                                 for obstacle in train["obstacles"]}), [1.9])
        self.assertEqual(len(extreme["obstacles"]), 7)

    def test_train_clearance_threshold_stays_above_risk_band(self):
        # 门槛放宽到 0.75 m，但仍必须高于风险带 0.65 m。
        self.assertEqual(TRAIN_MIN_CLEARANCE_M, 0.75)
        self.assertGreater(TRAIN_MIN_CLEARANCE_M, COLLISION_DIST_M)

    def test_figure_eight_keeps_v2_tight_layout(self):
        # 自交轨迹沿用 v2 原参数：加宽会把障碍推到另一叶航迹上。
        mirror = generate_layout(20270116, "train", 12)   # mirror_pair + figure_eight
        self.assertEqual(mirror["trajectory"]["type"], "figure_eight")
        self.assertEqual(mirror["layout"]["type"], "mirror_pair")
        self.assertEqual([obstacle["lateral_offset_m"]
                          for obstacle in mirror["obstacles"]], [0.95, -0.95])
        corridor = generate_layout(20270116, "train", 13)  # dense_corridor + figure_eight
        self.assertEqual(corridor["trajectory"]["type"], "figure_eight")
        self.assertEqual(len(corridor["obstacles"]), 7)    # 6 面 ±1.10 墙 + 中心柱
        self.assertEqual(sorted({abs(obstacle["lateral_offset_m"])
                                 for obstacle in corridor["obstacles"]}),
                         [0.35, 1.1])

    def test_figure_eight_skips_passability_gate(self):
        # figure_eight 的可行带结构性趋近 0（障碍必然贴近另一叶航迹），
        # train 门槛不适用；仍受出生净空这一硬门限约束。
        scenario = generate_layout(20270116, "train", 12)
        validation = scenario["layout_validation"]
        self.assertEqual(scenario["trajectory"]["type"], "figure_eight")
        # 净空可能仍高于放宽后的 0.75 m 门槛，但可行带结构性不足（残差必被
        # 另一叶航迹吃掉），故该门槛对自交轨迹整体不适用。
        self.assertLess(validation["band_width_m"], TRAIN_MIN_BAND_M)
        validate_scenario(scenario)  # 不抛错 = 门槛被豁免

    def test_figure_eight_spawn_clearance_still_enforced(self):
        # 豁免的是可通行性门槛，出生净空仍是硬门限。
        scenario = generate_layout(20270116, "train", 13)
        self.assertGreaterEqual(scenario["layout_validation"]["spawn_clearance_m"],
                                SPAWN_CLEARANCE_TARGET_M)
        scenario["obstacles"][0]["center"] = list(
            trajectory_frame(scenario["trajectory"], 0.0)[0])
        with self.assertRaises(RuntimeError) as context:
            validate_scenario(scenario)
        self.assertIn("出生净空未达标", str(context.exception))


@unittest.skipUnless(os.path.isdir(MANIFEST_DIR),
                     "stage2_v4 manifests 不存在，跳过产物一致性检查")
class ScenarioGeneratorV4ManifestTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifests = {}
        for split in SPLITS:
            path = os.path.join(MANIFEST_DIR, split + ".json")
            if not os.path.exists(path):
                raise unittest.SkipTest("%s 缺失" % path)
            with open(path, encoding="utf-8") as stream:
                cls.manifests[split] = json.load(stream)

    def _all_scenarios(self):
        return [scenario for split in SPLITS
                for scenario in self.manifests[split]["scenarios"]]

    def test_train_mirror_and_unseen_distribution(self):
        train = self.manifests["train"]["scenarios"]
        combination_signs = {}
        for scenario in train:
            combination_signs.setdefault(combo_key(scenario), []).append(
                scenario["layout"]["mirror_sign"])
        # train 覆盖除 EXCLUDED_COMBOS 外的全部组合（13/15），每个组合 ±1 各至少 2 条
        self.assertEqual(len(combination_signs), COMBO_COUNT - len(EXCLUDED_COMBOS))
        for key in combination_signs:
            self.assertNotIn(key, EXCLUDED_COMBOS)
        for signs in combination_signs.values():
            self.assertGreaterEqual(signs.count(1), 2)
            self.assertGreaterEqual(signs.count(-1), 2)
        # test 覆盖全部组合，且同时含 unseen 与 familiar 尺寸
        test = self.manifests["test"]["scenarios"]
        self.assertEqual({combo_key(item) for item in test},
                         set(combination_signs))
        self.assertTrue(any(item["layout"]["uses_unseen_size"]
                            for item in test))
        self.assertTrue(any(not item["layout"]["uses_unseen_size"]
                            for item in test))

    def test_excluded_combos_absent_from_every_split(self):
        # 自交轨迹 × 夹缝布局一律不生成（不入任何 split，也不留 test）。
        for scenario in self._all_scenarios():
            self.assertNotIn(
                (scenario["trajectory"]["type"], scenario["layout"]["type"]),
                EXCLUDED_COMBOS, scenario["scenario_id"])

    def test_manifests_verify(self):
        for split in SPLITS:
            self.assertTrue(verify_manifest(self.manifests[split]), split)

    def test_spawn_clearance_meets_target(self):
        # 索引空间 60/15/30 各去掉 EXCLUDED_COMBOS 的索引（8/2/4 个）后的条数
        expected = {"train": 52, "validation": 13, "test": 26}
        for split in SPLITS:
            self.assertEqual(len(self.manifests[split]["scenarios"]),
                             expected[split], split)
        scenarios = self._all_scenarios()
        self.assertEqual(len(scenarios), sum(expected.values()))
        for scenario in scenarios:
            validation = scenario["layout_validation"]
            self.assertGreaterEqual(
                validation["spawn_clearance_m"], SPAWN_CLEARANCE_TARGET_M,
                scenario["scenario_id"])
            self.assertEqual(validation["spawn_clearance_target_m"],
                             SPAWN_CLEARANCE_TARGET_M)

    def test_lateral_offsets_match_v2(self):
        # 未改动的布局侧向偏移仍与 v2 逐值一致；mirror_pair / dense_corridor
        # 按 difficulty 取 train 版或 extreme 版参数（extreme 版保留 v2 值），
        # figure_eight 例外：自交轨迹沿用 v2 参数。
        if "difficulty" not in self._all_scenarios()[0]["layout_validation"]:
            self.skipTest("stage2_v4 产物为修复前 schema，待重新生成后复核")
        rng = random.Random(LATERAL_SEED)
        for scenario in self._all_scenarios():
            validation = scenario["layout_validation"]
            layout = scenario["layout"]
            tight = (layout["difficulty"] == "extreme" or
                     scenario["trajectory"]["type"] == "figure_eight")
            self.assertEqual(set(validation), VALIDATION_KEYS,
                             scenario["scenario_id"])
            if layout["type"] == "mirror_pair":
                lateral = 0.95 if tight else 1.75
                self.assertEqual(validation["lateral_offsets"],
                                 [lateral, -lateral], scenario["scenario_id"])
                continue
            if layout["type"] == "dense_corridor":
                lateral = 1.10 if tight else 1.90
                expected = [lateral, -lateral] * 3
                if tight:
                    expected.append(round(0.35 * layout["mirror_sign"], 3))
                self.assertEqual(validation["lateral_offsets"], expected,
                                 scenario["scenario_id"])
                continue
            reference = _obstacles_for_layout(
                scenario["trajectory"], layout["type"], layout["variant"],
                scenario["split"], rng, DEFAULT_CONFIG)
            self.assertEqual(
                validation["lateral_offsets"],
                [item["lateral_offset_m"] for item in reference],
                scenario["scenario_id"])


if __name__ == "__main__":
    unittest.main()