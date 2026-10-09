#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5-A：低/高屏障「越顶 vs 水平绕行」对比场景组（纯 Python，可哈希、固定）。

两个场景组逐字段相同，唯一差异是屏障的竖直范围：
  - low_barrier : 顶 1.8 m < 飞行高度 2.0 m → 可越顶
  - tall_barrier: 顶 3.4 m ≥ 飞行高度      → 只能水平绕行
屏障为薄墙，横向跨度 1.8 m：不避障直飞会撞墙，横向绕行可行。

用法:
  python3 src/rl_obstacle_avoidance/scripts/n5_barrier_scenarios.py \
      --write --out-dir experiments/n5/drone/manifests
  python3 src/rl_obstacle_avoidance/scripts/n5_barrier_scenarios.py \
      --print-sha --out-dir /tmp/n5check
"""

import argparse
import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scenario_generator import check_feasibility
from scenario_generator_v2 import canonical_json


SCHEMA_VERSION = "n5-barriers-v1"
SPLIT = "test"
ALTITUDE = 2.0
BOUNDS = [-12.0, 12.0, -10.0, 10.0]
RESOLUTION = 0.25
SAFETY_MARGIN = 0.65
SPEED_LIMIT = 1.02
SENSOR_NOISE = 0.0
DECISION_DELAY_S = 0.0
SCENARIO_SEED = 20270116

# 屏障横向几何：薄墙（沿飞行方向 0.30 m）跨越 1.8 m 横向，可绕行但直撞碰撞。
BARRIER_SIZE_X = 0.30
BARRIER_SIZE_Y = 0.8
# 屏障竖直范围：唯一分组差异（low 顶 1.8 < 2.0；tall 顶 3.4 ≥ 2.0）。
LOW_BARRIER = {"size_z": 1.6, "center_z": 1.0}
TALL_BARRIER = {"size_z": 2.8, "center_z": 2.0}

# 5 组基场景（>=5），仅轨迹横向中心与屏障纵向位置不同；两组共用同一基场景。
VARIANTS = (
    {"track_y": -0.8, "barrier_x": -1.0},
    {"track_y": -0.4, "barrier_x": -0.5},
    {"track_y": 0.0, "barrier_x": 0.0},
    {"track_y": 0.4, "barrier_x": 0.5},
    {"track_y": 0.8, "barrier_x": 1.0},
)

GROUPS = ("low_barrier", "tall_barrier")
_BARRIER_SPECS = {"low_barrier": LOW_BARRIER, "tall_barrier": TALL_BARRIER}


def barrier_top_m(group):
    """屏障顶面高度（center_z + size_z/2）。"""
    spec = _BARRIER_SPECS[group]
    return spec["center_z"] + 0.5 * spec["size_z"]


def _scenario(index, variant, group):
    """按基场景 + 分组屏障高度构造一个完整 v4 风格场景。"""
    spec = _BARRIER_SPECS[group]
    track_y = float(variant["track_y"])
    barrier_x = float(variant["barrier_x"])
    start = [-8.0, track_y, ALTITUDE]
    goal = [8.0, track_y, ALTITUDE]
    trajectory = {"type": "linear", "start": list(start), "end": list(goal),
                  "duration_s": 40.0, "loop": False}
    progress = (barrier_x - start[0]) / (goal[0] - start[0])
    obstacle = {
        "id": "barrier0",
        "group": "a",
        "type": "wall",
        "center": [barrier_x, track_y, float(spec["center_z"])],
        "size": [BARRIER_SIZE_X, BARRIER_SIZE_Y, float(spec["size_z"])],
        "yaw": 0.0,
        "trajectory_progress": round(progress, 4),
        "lateral_offset_m": 0.0,
    }
    scenario = {
        "schema_version": SCHEMA_VERSION,
        "split": SPLIT,
        "index": index,
        "seed": SCENARIO_SEED,
        "seeds": {
            "scenario_seed": SCENARIO_SEED,
            "generator_seed": SCENARIO_SEED + index,
            "disturbance_seed": SCENARIO_SEED + index,
            "policy_seed": None,
        },
        "layout": {"type": "frontal_barrier", "variant": index, "combo": index,
                   "mirror_sign": 1, "uses_unseen_size": False,
                   "difficulty": "n5"},
        "bounds": list(BOUNDS),
        "start": start,
        "goal": goal,
        "altitude": ALTITUDE,
        "speed_limit": SPEED_LIMIT,
        "sensor_noise": SENSOR_NOISE,
        "decision_delay_s": DECISION_DELAY_S,
        "trajectory": trajectory,
        "obstacles": [obstacle],
        "obstacles_a": [list(obstacle["center"])],
        "obstacles_b": [],
        "planner": {"resolution": RESOLUTION, "safety_margin": SAFETY_MARGIN},
    }
    scenario["feasibility"] = check_feasibility(scenario)
    # scenario_id 由布局身份计算（含障碍尺寸，故 low/tall 各自唯一）。
    identity = {key: scenario[key] for key in (
        "schema_version", "split", "index", "layout", "bounds", "start",
        "goal", "trajectory", "obstacles", "planner")}
    scenario["scenario_id"] = "n5-" + hashlib.sha1(
        canonical_json(identity).encode("utf-8")).hexdigest()[:10]
    return scenario


def build_groups():
    """返回 {"low_barrier": [...], "tall_barrier": [...]}，每组 5 个场景。"""
    groups = {name: [] for name in GROUPS}
    for index, variant in enumerate(VARIANTS):
        for group in GROUPS:
            groups[group].append(_scenario(index, variant, group))
    return groups


def manifest_sha256(group):
    """场景列表的规范 JSON sha256（与 manifest 内记录一致）。"""
    return hashlib.sha256(
        canonical_json(build_groups()[group]).encode("utf-8")).hexdigest()


def write_manifests(out_dir):
    """写出 low_barrier.json / tall_barrier.json，返回 {group: 路径}。"""
    groups = build_groups()
    os.makedirs(out_dir, exist_ok=True)
    paths = {}
    for group, scenarios in groups.items():
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "split": SPLIT,
            "scenarios": scenarios,
            "manifest_sha256": hashlib.sha256(
                canonical_json(scenarios).encode("utf-8")).hexdigest(),
        }
        path = os.path.join(out_dir, "%s.json" % group)
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        paths[group] = path
    return paths


def main():
    parser = argparse.ArgumentParser(description="N5-A 屏障对比场景组")
    parser.add_argument("--write", action="store_true", help="写出两组 manifest")
    parser.add_argument("--print-sha", action="store_true", help="打印组 sha256")
    parser.add_argument("--out-dir", default="experiments/n5/drone/manifests")
    args = parser.parse_args()

    groups = build_groups()
    for group in GROUPS:
        print("[N5-A] %s: %d 场景 barrier_top=%.2f m"
              % (group, len(groups[group]), barrier_top_m(group)))
    if args.write:
        paths = write_manifests(args.out_dir)
        for group, path in paths.items():
            print("[N5-A] 已写出 %s -> %s" % (group, path))
    if args.print_sha or not args.write:
        for group in GROUPS:
            print("[N5-A] %s manifest_sha256=%s" % (group,
                                                    manifest_sha256(group)))


if __name__ == "__main__":
    main()