#!/usr/bin/env python3
"""Generate trajectory-aware Stage-2 scenarios without modifying frozen v1 data."""

import argparse
import copy
import hashlib
import json
import math
import os
import random
import tempfile

from scenario_generator import check_feasibility


SCHEMA_VERSION = "stage2-scenarios-v2"
SPLITS = ("train", "validation", "test")
TRAJECTORY_TYPES = ("linear", "circular", "figure_eight")
LAYOUT_TYPES = (
    "frontal_block",
    "single_side",
    "mirror_pair",
    "dense_corridor",
    "curve_challenge",
)

DEFAULT_CONFIG = {
    "bounds": [-12.0, 12.0, -10.0, 10.0],
    "resolution": 0.25,
    "safety_margin": 0.65,
    "altitude": 2.0,
    "speed_limit": 1.02,
    "sensor_noise": 0.0,
    "decision_delay_s": 0.0,
    "trajectory_duration": 40.0,
    "angular_velocity": 0.2,
    "profiles": {
        "train": {"count": 50},
        "validation": {"count": 12},
        "test": {"count": 30},
    },
    "sizes": {
        "pillar_box": [[0.25, 0.25, 2.8], [0.40, 0.40, 2.8]],
        "cylinder": [[0.65, 0.65, 2.8], [0.85, 0.85, 2.8]],
        "wall": [[1.40, 0.25, 2.8], [1.80, 0.30, 2.8]],
    },
    "test_unseen_sizes": {
        "pillar_box": [0.55, 0.35, 2.8],
        "cylinder": [1.05, 1.05, 2.8],
        "wall": [2.05, 0.28, 2.8],
    },
}


def canonical_json(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False)


def sha256_json(value):
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def _merge_config(config=None):
    merged = copy.deepcopy(DEFAULT_CONFIG)
    if not config:
        return merged
    for key, value in config.items():
        if key in ("profiles", "sizes", "test_unseen_sizes"):
            merged[key].update(copy.deepcopy(value))
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _trajectory(index, config):
    block = index // len(LAYOUT_TYPES)
    kind = TRAJECTORY_TYPES[(index + block) % len(TRAJECTORY_TYPES)]
    altitude = float(config["altitude"])
    duration = float(config["trajectory_duration"])
    omega = float(config["angular_velocity"])
    variation = block % 3
    if kind == "linear":
        y = (-0.5, 0.0, 0.5)[variation]
        return {
            "type": kind,
            "start": [-8.0, y, altitude],
            "end": [8.0, y, altitude],
            "duration_s": duration,
            "loop": False,
        }
    if kind == "circular":
        radius = (4.2, 4.6, 5.0)[variation]
        return {
            "type": kind,
            "center": [0.0, 0.0, altitude],
            "radius_m": radius,
            "height_m": altitude,
            "angular_velocity_rad_s": omega,
            "duration_s": 2.0 * math.pi / omega,
            "loop": True,
        }
    radius_x = (4.2, 4.6, 5.0)[variation]
    radius_y = (2.4, 2.7, 3.0)[variation]
    return {
        "type": kind,
        "center": [0.0, 0.0, altitude],
        "radius_x_m": radius_x,
        "radius_y_m": radius_y,
        "height_m": altitude,
        "angular_velocity_rad_s": omega,
        "duration_s": 2.0 * math.pi / omega,
        "loop": True,
    }


def trajectory_frame(trajectory, progress):
    """Return position, unit tangent and left normal at normalized progress."""
    u = min(max(float(progress), 0.0), 1.0)
    kind = trajectory["type"]
    if kind == "linear":
        start, end = trajectory["start"], trajectory["end"]
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        tangent = (dx / length, dy / length)
        point = (start[0] + u * dx, start[1] + u * dy)
    elif kind == "circular":
        theta = 2.0 * math.pi * u
        center, radius = trajectory["center"], trajectory["radius_m"]
        point = (center[0] + radius * math.cos(theta),
                 center[1] + radius * math.sin(theta))
        tangent = (-math.sin(theta), math.cos(theta))
    else:
        theta = 2.0 * math.pi * u
        center = trajectory["center"]
        radius_x = trajectory["radius_x_m"]
        radius_y = trajectory["radius_y_m"]
        point = (center[0] + radius_x * math.sin(theta),
                 center[1] + radius_y * math.sin(theta) * math.cos(theta))
        derivative = (radius_x * math.cos(theta),
                      radius_y * math.cos(2.0 * theta))
        length = math.hypot(derivative[0], derivative[1])
        tangent = (derivative[0] / length, derivative[1] / length)
    normal = (-tangent[1], tangent[0])
    return point, tangent, normal


def _size(shape, rng, config, unseen):
    if unseen:
        return [float(value) for value in config["test_unseen_sizes"][shape]]
    choices = config["sizes"][shape]
    return [float(value) for value in choices[rng.randrange(len(choices))]]


def _obstacles_for_layout(trajectory, layout, variant, split, rng, config):
    obstacles = []
    unseen = split == "test" and variant % 2 == 1
    mirror = -1.0 if variant % 2 else 1.0
    altitude = float(config["altitude"])

    def add(shape, progress, lateral=0.0, longitudinal=0.0,
            wall_alignment="tangent"):
        point, tangent, normal = trajectory_frame(trajectory, progress)
        x = point[0] + longitudinal * tangent[0] + lateral * normal[0]
        y = point[1] + longitudinal * tangent[1] + lateral * normal[1]
        yaw = math.atan2(tangent[1], tangent[0])
        if wall_alignment == "normal":
            yaw += math.pi / 2.0
        item = {
            "id": "o%02d" % len(obstacles),
            "group": "a" if len(obstacles) % 2 == 0 else "b",
            "type": shape,
            "center": [round(x, 3), round(y, 3), altitude],
            "size": _size(shape, rng, config, unseen),
            "yaw": round(yaw if shape == "wall" else 0.0, 6),
            "trajectory_progress": round(progress, 4),
            "lateral_offset_m": round(lateral, 3),
        }
        obstacles.append(item)

    anchor = (0.30, 0.42, 0.56)[variant % 3]
    if layout == "frontal_block":
        add("cylinder", anchor)
    elif layout == "single_side":
        add("pillar_box", anchor, 0.15 * mirror)
        add("wall", anchor + 0.035, 1.05 * mirror, wall_alignment="tangent")
    elif layout == "mirror_pair":
        add("pillar_box", anchor, 0.95)
        add("pillar_box", anchor, -0.95)
    elif layout == "dense_corridor":
        dense_progress = ((0.18, 0.32, 0.68)
                          if trajectory["type"] == "figure_eight"
                          else (0.27, 0.43, 0.59))
        for progress in dense_progress:
            add("wall", progress, 1.10, wall_alignment="tangent")
            add("wall", progress, -1.10, wall_alignment="tangent")
        center_progress = (0.72 if trajectory["type"] == "figure_eight"
                           else 0.50)
        add("pillar_box", center_progress, 0.35 * mirror)
    elif layout == "curve_challenge":
        # Curved trajectories use their turning sections; linear provides the
        # matched low-curvature control for the same multi-encounter template.
        curve_points = (((0.12, 0.0), (0.30, 0.25 * mirror),
                         (0.70, -0.20 * mirror))
                        if trajectory["type"] == "figure_eight" else
                        ((0.18, 0.0), (0.52, 0.25 * mirror),
                         (0.78, -0.20 * mirror)))
        for progress, lateral in curve_points:
            add("cylinder", progress, lateral)
    else:
        raise ValueError("unknown layout: %s" % layout)
    return obstacles


def _start_goal(trajectory):
    start, _, _ = trajectory_frame(trajectory, 0.0)
    goal_progress = 1.0 if trajectory["type"] == "linear" else 0.92
    goal, _, _ = trajectory_frame(trajectory, goal_progress)
    z = float(trajectory.get("height_m", trajectory.get("start", [0, 0, 2])[2]))
    return [round(start[0], 6), round(start[1], 6), z], \
        [round(goal[0], 6), round(goal[1], 6), z]


def generate_layout(seed, split="train", index=0, config=None,
                    disturbance_seed=20270117, policy_seed=None):
    if split not in SPLITS:
        raise ValueError("split must be one of %s" % ", ".join(SPLITS))
    config = _merge_config(config)
    index = int(index)
    effective_seed = int(seed) + {"train": 0, "validation": 1000003,
                                  "test": 2000003}[split] + index * 7919
    rng = random.Random(effective_seed)
    trajectory = _trajectory(index, config)
    layout = LAYOUT_TYPES[index % len(LAYOUT_TYPES)]
    variant = index // (len(TRAJECTORY_TYPES) * len(LAYOUT_TYPES))
    obstacles = _obstacles_for_layout(
        trajectory, layout, variant, split, rng, config)
    start, goal = _start_goal(trajectory)
    scenario = {
        "schema_version": SCHEMA_VERSION,
        "split": split,
        "index": index,
        "seed": int(seed),
        "seeds": {
            "scenario_seed": int(seed),
            "generator_seed": effective_seed,
            "disturbance_seed": int(disturbance_seed) + index,
            "policy_seed": policy_seed,
        },
        "layout": {
            "type": layout,
            "variant": variant,
            "mirror_sign": -1 if variant % 2 else 1,
            "uses_unseen_size": bool(
                obstacles and split == "test" and variant % 2 == 1),
        },
        "bounds": [float(value) for value in config["bounds"]],
        "start": start,
        "goal": goal,
        "altitude": float(config["altitude"]),
        "speed_limit": float(config["speed_limit"]),
        "sensor_noise": float(config["sensor_noise"]),
        "decision_delay_s": float(config["decision_delay_s"]),
        "trajectory": trajectory,
        "obstacles": obstacles,
        "obstacles_a": [item["center"] for item in obstacles
                        if item["group"] == "a"],
        "obstacles_b": [item["center"] for item in obstacles
                        if item["group"] == "b"],
        "planner": {
            "resolution": float(config["resolution"]),
            "safety_margin": float(config["safety_margin"]),
        },
    }
    scenario["feasibility"] = check_feasibility(scenario)
    identity = {key: scenario[key] for key in (
        "schema_version", "split", "index", "layout", "bounds", "start",
        "goal", "trajectory", "obstacles", "planner")}
    scenario["scenario_id"] = "s2v2-" + hashlib.sha1(
        canonical_json(identity).encode("utf-8")).hexdigest()[:10]
    return scenario


def generate_scenarios(split, count=None, seed=20270116, config=None,
                       disturbance_seed=20270117, policy_seed=None):
    config = _merge_config(config)
    if count is None:
        count = int(config["profiles"][split]["count"])
    scenarios = [generate_layout(seed, split, index, config,
                                 disturbance_seed, policy_seed)
                 for index in range(int(count))]
    invalid = [item["scenario_id"] for item in scenarios
               if not item["feasibility"]["feasible"]]
    if invalid:
        raise RuntimeError("generated infeasible scenarios: %s" % invalid)
    return scenarios


def build_manifest(split, scenarios, seed=20270116, config=None,
                   disturbance_seed=20270117, policy_seed=None):
    records = copy.deepcopy(list(scenarios))
    manifest_config = _merge_config(config)
    manifest_config["profiles"][split]["count"] = len(records)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "split": split,
        "scenario_seed": int(seed),
        "disturbance_seed": int(disturbance_seed),
        "policy_seed": policy_seed,
        "config": manifest_config,
        "scenarios": records,
    }
    manifest["manifest_sha256"] = sha256_json(manifest)
    return manifest


def verify_manifest(manifest):
    recorded = manifest.get("manifest_sha256")
    payload = {key: value for key, value in manifest.items()
               if key != "manifest_sha256"}
    return bool(recorded) and recorded == sha256_json(payload)


def _atomic_json_write(path, value):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".scenario-v2-",
                                     dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2,
                      sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    except Exception:
        if os.path.exists(temporary):
            os.unlink(temporary)
        raise


def write_manifests(output_dir, *, seed=20270116,
                    disturbance_seed=20270117, policy_seed=None,
                    config=None, counts=None, force=False):
    output_dir = os.path.abspath(output_dir)
    marker = os.path.join(output_dir, "manifests", ".frozen")
    if os.path.exists(marker):
        raise PermissionError("manifest directory is frozen and read-only")
    counts = counts or {}
    manifests = {}
    for split in SPLITS:
        target = os.path.join(output_dir, "manifests", split + ".json")
        if os.path.exists(target) and not force:
            raise FileExistsError("%s exists; use --force to replace it" % target)
        scenarios = generate_scenarios(
            split, counts.get(split), seed, config,
            disturbance_seed, policy_seed)
        manifest = build_manifest(split, scenarios, seed, config,
                                  disturbance_seed, policy_seed)
        manifests[split] = manifest
        _atomic_json_write(target, manifest)
        split_dir = os.path.join(output_dir, "scenarios", split)
        os.makedirs(split_dir, exist_ok=True)
        if force:
            keep = {item["scenario_id"] + ".json" for item in scenarios}
            for name in os.listdir(split_dir):
                if (name.startswith("s2v2-") and name.endswith(".json") and
                        name not in keep):
                    os.unlink(os.path.join(split_dir, name))
        for scenario in scenarios:
            _atomic_json_write(os.path.join(
                split_dir, scenario["scenario_id"] + ".json"), scenario)
    _atomic_json_write(os.path.join(output_dir, "manifests", "index.json"), {
        "schema_version": SCHEMA_VERSION,
        "manifests": {split: manifests[split]["manifest_sha256"]
                      for split in SPLITS},
    })
    return manifests


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="生成轨迹匹配的 Stage-2 v2 场景清单")
    parser.add_argument("--output-dir", default="experiments/stage2_v2")
    parser.add_argument("--scenario-seed", type=int, default=20270116)
    parser.add_argument("--disturbance-seed", type=int, default=20270117)
    parser.add_argument("--policy-seed", type=int)
    parser.add_argument("--train-count", type=int)
    parser.add_argument("--validation-count", type=int)
    parser.add_argument("--test-count", type=int)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--freeze", action="store_true")
    parser.add_argument("--verify")
    parser.add_argument("--check")
    args = parser.parse_args(argv)
    if args.verify:
        with open(args.verify, encoding="utf-8") as stream:
            valid = verify_manifest(json.load(stream))
        print("valid" if valid else "invalid")
        return 0 if valid else 1
    if args.check:
        with open(args.check, encoding="utf-8") as stream:
            result = check_feasibility(json.load(stream))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["feasible"] else 1
    counts = {"train": args.train_count,
              "validation": args.validation_count,
              "test": args.test_count}
    counts = {key: value for key, value in counts.items()
              if value is not None}
    manifests = write_manifests(
        args.output_dir, seed=args.scenario_seed,
        disturbance_seed=args.disturbance_seed,
        policy_seed=args.policy_seed, counts=counts, force=args.force)
    if args.freeze:
        marker = os.path.join(os.path.abspath(args.output_dir),
                              "manifests", ".frozen")
        with open(marker, "w", encoding="utf-8") as stream:
            stream.write("Frozen by scenario_generator_v2.py\n")
    for split in SPLITS:
        print("%s: %d scenarios sha256=%s" % (
            split, len(manifests[split]["scenarios"]),
            manifests[split]["manifest_sha256"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
