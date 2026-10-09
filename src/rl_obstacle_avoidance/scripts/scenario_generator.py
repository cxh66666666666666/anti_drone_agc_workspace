#!/usr/bin/env python3
"""Deterministic Stage-2 scenario generation and manifest utilities.

The module deliberately has no ROS dependency.  A scenario is a complete,
serialisable description of one layout (including the three random-seed
roles), while the feasibility check uses the same grid A* implementation as
the runtime planner.  This keeps the train/validation/test split useful both
from CI and from a ROS/Gazebo launch script.
"""

import argparse
import copy
import hashlib
import json
import math
import os
import random
import sys
import tempfile


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from path_planning import (astar_path, build_typed_grid, typed_goal_valid)  # noqa: E402


SCHEMA_VERSION = "stage2-scenarios-v1"
SPLITS = ("train", "validation", "test")
DEFAULT_CONFIG = {
    "bounds": [-20.0, 20.0, -15.0, 15.0],
    "resolution": 0.5,
    "safety_margin": 1.8,
    "start": [-15.0, 0.0, 2.0],
    "goal": [15.0, 0.0, 2.0],
    "altitude": 2.0,
    "speed_limit": 1.02,
    "sensor_noise": 0.0,
    "decision_delay_s": 0.0,
    "trajectory_types": ["linear", "circular", "helical",
                          "fixed_horizontal", "step"],
    "obstacle_sizes": [[0.3, 0.3, 3.0], [0.5, 0.5, 3.0],
                       [0.8, 0.4, 3.0]],
    "obstacle_types": ["pillar_box", "wall", "cylinder"],
    "shape_sizes": {
        "wall": [[2.0, 0.5, 2.5], [3.0, 0.5, 3.0]],
        "cylinder": [[1.0, 1.0, 3.0], [1.5, 1.5, 2.5]],
    },
    "profiles": {
        # Keep the largest configured size out of training so test exercises a
        # genuinely unseen geometry combination by default.
        "train": {"count": 50, "max_obstacles": 10,
                   "size_indices": [0, 1]},
        "validation": {"count": 12, "max_obstacles": 8,
                        "size_indices": [0, 1, 2]},
        "test": {"count": 30, "max_obstacles": 14,
                 "size_indices": [1, 2]},
    },
}


def canonical_json(value):
    """Return the byte-stable JSON representation used for all hashes."""
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False)


def sha256_json(value):
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def _scenario_id(scenario):
    # Do not let the generated annotation or retry bookkeeping affect the
    # identity of the physical layout itself.  Policy/disturbance seeds and
    # split bookkeeping are protocol metadata, not layout identity.
    payload = {key: scenario[key] for key in (
        "bounds", "start", "goal", "altitude", "speed_limit",
        "sensor_noise", "decision_delay_s", "trajectory", "obstacles",
        "planner", "split", "index") if key in scenario}
    return "s-" + hashlib.sha1(canonical_json(payload).encode("utf-8")).hexdigest()[:10]


def _merge_config(config=None):
    merged = copy.deepcopy(DEFAULT_CONFIG)
    if config:
        for key, value in config.items():
            if key == "profiles" and isinstance(value, dict):
                for split, profile in value.items():
                    if split in merged["profiles"] and isinstance(profile, dict):
                        merged["profiles"][split].update(copy.deepcopy(profile))
                    else:
                        merged["profiles"][split] = copy.deepcopy(profile)
            else:
                merged[key] = copy.deepcopy(value)
    return merged


def _validate_config(config):
    bounds = config["bounds"]
    if len(bounds) != 4 or not (bounds[0] < bounds[1] and bounds[2] < bounds[3]):
        raise ValueError("bounds must be [xmin, xmax, ymin, ymax]")
    if float(config["resolution"]) <= 0 or float(config["safety_margin"]) < 0:
        raise ValueError("resolution must be positive and safety_margin non-negative")
    if len(config["start"]) < 2 or len(config["goal"]) < 2:
        raise ValueError("start and goal must contain x and y")
    if not config["trajectory_types"]:
        raise ValueError("trajectory_types cannot be empty")
    if not config.get("obstacle_types"):
        raise ValueError("obstacle_types cannot be empty")
    for size in config["obstacle_sizes"]:
        if len(size) != 3 or any(float(v) <= 0 for v in size):
            raise ValueError("obstacle_sizes must contain positive [x, y, z] sizes")


def _split_seed(seed, split):
    # Large, fixed offsets make accidental cross-split collisions unlikely and
    # keep each split reproducible when counts are changed independently.
    return int(seed) + {"train": 0, "validation": 1000003, "test": 2000003}[split]


def _profile_count(config, split, count):
    if count is not None:
        if count < 0:
            raise ValueError("count must be non-negative")
        return int(count)
    return int(config["profiles"][split]["count"])


def _obstacle_count(rng, index, maximum):
    if index % 10 == 0:
        return 0  # required no-obstacle coverage in every split
    if index % 10 in (1, 2, 3):
        return min(maximum, 1 + rng.randrange(2))
    if index % 10 in (4, 5):
        return min(maximum, max(3, 2 + rng.randrange(3)))
    return rng.randint(0, maximum)


def _candidate_points(rng, count, bounds, symmetric=False):
    xmin, xmax, ymin, ymax = map(float, bounds)
    # Leave a generous clearance around start/goal and the world boundary.
    xlo, xhi = xmin + 4.0, xmax - 4.0
    ylo, yhi = ymin + 2.5, ymax - 2.5
    points = []
    attempts = 0
    target = (count + 1) // 2 if symmetric else count
    while len(points) < target and attempts < 2000:
        attempts += 1
        point = (round(rng.uniform(xlo, xhi), 3),
                 round(rng.uniform(ylo, yhi), 3))
        if any(math.hypot(point[0] - old[0], point[1] - old[1]) < 2.2
               for old in points):
            continue
        points.append(point)
    if len(points) < target:
        # This deterministic fallback is only reachable for unusually small
        # bounds; it is preferable to silently returning fewer obstacles.
        for i in range(target - len(points)):
            x = xlo + (xhi - xlo) * (i + 1) / float(target + 1)
            y = yhi if i % 2 else ylo
            points.append((round(x, 3), round(y, 3)))
    if symmetric:
        mirrored = [(x, -y) for x, y in points]
        points = points + mirrored
    return points[:count]


def _make_obstacles(rng, count, config, symmetric=False, split="train"):
    points = _candidate_points(rng, count, config["bounds"], symmetric)
    sizes = config["obstacle_sizes"]
    size_indices = config["profiles"].get(split, {}).get(
        "size_indices", list(range(len(sizes))))
    size_indices = [int(index) for index in size_indices
                    if 0 <= int(index) < len(sizes)]
    if not size_indices:
        raise ValueError("profile size_indices must reference obstacle_sizes")
    # Test layouts intentionally use the largest/novel size combinations;
    # train/validation still sample all configured sizes reproducibly.
    obstacles = []
    for index, (x, y) in enumerate(points):
        shape_index = index % len(config["obstacle_types"])
        obstacle_type = config["obstacle_types"][shape_index]
        if obstacle_type not in ("pillar_box", "wall", "cylinder"):
            raise ValueError("unsupported obstacle type: %s" % obstacle_type)
        if obstacle_type == "pillar_box":
            if split == "test" and len(size_indices) > 1 and index % 3 == 0:
                size = sizes[size_indices[-1]]
            else:
                size = sizes[size_indices[rng.randrange(len(size_indices))]]
        else:
            shape_sizes = config.get("shape_sizes", {}).get(obstacle_type)
            if not shape_sizes:
                raise ValueError("missing shape_sizes for %s" % obstacle_type)
            size = shape_sizes[rng.randrange(len(shape_sizes))]
        center = [x, y, float(config["altitude"])]
        obstacles.append({
            "id": "o%02d" % index,
            "group": "a" if index % 2 == 0 else "b",
            "type": obstacle_type,
            "center": center,
            "size": [float(v) for v in size],
            "yaw": (round(rng.uniform(-math.pi, math.pi), 6)
                    if obstacle_type == "wall" else 0.0),
        })
    return obstacles


def check_feasibility(scenario, *, bounds=None, resolution=None,
                      safety_margin=None):
    """Check start/goal clearance and A* reachability for one scenario.

    The return value is JSON-safe and intentionally records failures rather
    than raising for an invalid layout.  Malformed input still raises
    ``ValueError`` so callers cannot mistake it for a valid negative result.
    """
    bounds = scenario.get("bounds", bounds)
    planner = scenario.get("planner") or {}
    resolution = planner.get("resolution", resolution)
    safety_margin = planner.get("safety_margin", safety_margin)
    if bounds is None or resolution is None or safety_margin is None:
        raise ValueError("bounds, resolution and safety_margin are required")
    start = scenario.get("start")
    goal = scenario.get("goal")
    if start is None or goal is None or len(start) < 2 or len(goal) < 2:
        raise ValueError("scenario requires start and goal [x,y,z] points")
    obstacles = scenario.get("obstacles", [])
    if len(bounds) != 4 or not (float(bounds[0]) < float(bounds[1]) and
                                float(bounds[2]) < float(bounds[3])):
        raise ValueError("bounds must be [xmin, xmax, ymin, ymax]")
    if float(resolution) <= 0 or float(safety_margin) < 0:
        raise ValueError("resolution must be positive and safety_margin non-negative")
    xmin, xmax, ymin, ymax = map(float, bounds)
    centers = []
    for item in obstacles:
        if (not isinstance(item, dict) or
                len(item.get("center", [])) != 3 or
                len(item.get("size", [])) != 3):
            raise ValueError("each obstacle requires center [x,y,z] and size [x,y,z]")
        if any(float(value) <= 0 for value in item["size"]):
            raise ValueError("obstacle sizes must be positive")
        centers.append((float(item["center"][0]), float(item["center"][1])))
    inflation = float(safety_margin)
    reasons = []
    if not (xmin <= float(start[0]) <= xmax and ymin <= float(start[1]) <= ymax):
        reasons.append("start_out_of_bounds")
    if not (xmin <= float(goal[0]) <= xmax and ymin <= float(goal[1]) <= ymax):
        reasons.append("goal_out_of_bounds")
    for center in centers:
        if not (xmin <= center[0] <= xmax and ymin <= center[1] <= ymax):
            reasons.append("obstacle_out_of_bounds")
            break
    if not typed_goal_valid((float(start[0]), float(start[1])), obstacles, inflation):
        reasons.append("start_too_close")
    if not typed_goal_valid((float(goal[0]), float(goal[1])), obstacles, inflation):
        reasons.append("goal_too_close")
    blocked = build_typed_grid(obstacles, bounds, float(resolution), inflation)
    path = [] if reasons else astar_path(
        blocked, (float(start[0]), float(start[1])),
        (float(goal[0]), float(goal[1])), bounds, float(resolution))
    if not path and not reasons:
        reasons.append("no_path")
    path_length = sum(math.hypot(path[i][0] - path[i - 1][0],
                                 path[i][1] - path[i - 1][1])
                      for i in range(1, len(path)))
    def clearance(point):
        if not obstacles:
            # JSON has no portable infinity literal; ``None`` denotes an
            # unbounded clearance in the manifest.
            return None
        from path_planning import obstacle_clearance_2d
        return min(obstacle_clearance_2d(point, obstacle)
                   for obstacle in obstacles)
    return {
        "feasible": not reasons,
        "path_found": bool(path),
        "path_length_m": round(path_length, 6),
        "start_clearance_m": (None if clearance(start) is None
                              else round(clearance(start), 6)),
        "goal_clearance_m": (None if clearance(goal) is None
                             else round(clearance(goal), 6)),
        "reasons": reasons,
        "planner": {
            "bounds": [float(v) for v in bounds],
            "resolution": float(resolution),
            "inflation_radius": float(inflation),
        },
    }


def generate_layout(seed, split="train", index=0, config=None,
                    disturbance_seed=None, policy_seed=None):
    """Generate one deterministic, feasible scenario layout.

    ``seed`` is the scenario-generation seed.  The other two seeds are
    recorded for protocol completeness and never influence obstacle placement.
    """
    if split not in SPLITS:
        raise ValueError("split must be one of %s" % (", ".join(SPLITS)))
    config = _merge_config(config)
    _validate_config(config)
    index = int(index)
    if index < 0:
        raise ValueError("index must be non-negative")
    effective_seed = _split_seed(seed, split) + index * 7919
    rng = random.Random(effective_seed)
    maximum = int(config["profiles"][split].get("max_obstacles", 0))
    symmetric = index % 10 == 3
    count = _obstacle_count(rng, index, maximum)
    if symmetric:
        count = max(2, count + count % 2)
    obstacles = _make_obstacles(rng, count, config, symmetric, split)
    trajectory = config["trajectory_types"][index % len(config["trajectory_types"])]
    curve_radius = (round(rng.uniform(6.0, 12.0), 3)
                    if trajectory == "circular" else None)
    scenario = {
        "schema_version": SCHEMA_VERSION,
        "split": split,
        "index": index,
        "generation_attempt": 0,
        "seed": int(seed),
        "seeds": {
            "scenario_seed": int(seed),
            "generator_seed": int(effective_seed),
            "disturbance_seed": (None if disturbance_seed is None
                                  else int(disturbance_seed) + index),
            "policy_seed": (None if policy_seed is None else int(policy_seed)),
        },
        "bounds": [float(v) for v in config["bounds"]],
        "start": [float(v) for v in config["start"]],
        "goal": [float(v) for v in config["goal"]],
        "altitude": float(config["altitude"]),
        "speed_limit": float(config["speed_limit"]),
        "sensor_noise": float(config["sensor_noise"]),
        "decision_delay_s": float(config["decision_delay_s"]),
        "trajectory": {
            "type": trajectory,
            "curvature": (0.0 if curve_radius is None else round(1.0 / curve_radius, 8)),
            "radius_m": curve_radius,
        },
        "obstacles": obstacles,
    }
    scenario["obstacles_a"] = [item["center"] for item in obstacles
                                 if item["group"] == "a"]
    scenario["obstacles_b"] = [item["center"] for item in obstacles
                                 if item["group"] == "b"]
    scenario["planner"] = {
        "resolution": float(config["resolution"]),
        "safety_margin": float(config["safety_margin"]),
    }
    scenario["feasibility"] = check_feasibility(scenario)
    # Rejection sampling is deterministic: an infeasible candidate is replaced
    # with a new index-derived stream.  The record always explains the outcome.
    if not scenario["feasibility"]["feasible"]:
        for retry in range(1, 101):
            retry_rng = random.Random(effective_seed + retry * 104729)
            scenario["obstacles"] = _make_obstacles(
                retry_rng, count, config, symmetric, split)
            scenario["obstacles_a"] = [item["center"] for item in scenario["obstacles"]
                                        if item["group"] == "a"]
            scenario["obstacles_b"] = [item["center"] for item in scenario["obstacles"]
                                        if item["group"] == "b"]
            scenario["feasibility"] = check_feasibility(scenario)
            scenario["generation_attempt"] = retry
            if scenario["feasibility"]["feasible"]:
                break
    if not scenario["feasibility"]["feasible"]:
        # Keep the requested density but produce an actionable diagnostic;
        # callers can lower max_obstacles or increase the world bounds.
        scenario["generation_error"] = ";".join(
            scenario["feasibility"]["reasons"])
    scenario["scenario_id"] = _scenario_id(scenario)
    return scenario


def generate_scenarios(split, count=None, seed=20270116, config=None,
                       disturbance_seed=20270117, policy_seed=None):
    """Return a deterministic list of feasible scenarios for ``split``."""
    if split not in SPLITS:
        raise ValueError("split must be one of %s" % (", ".join(SPLITS)))
    config = _merge_config(config)
    _validate_config(config)
    count = _profile_count(config, split, count)
    scenarios = [generate_layout(seed, split, index, config,
                                 disturbance_seed, policy_seed)
                 for index in range(count)]
    infeasible = [item["scenario_id"] for item in scenarios
                  if not item["feasibility"]["feasible"]]
    if infeasible:
        raise RuntimeError("generated infeasible scenarios: %s" % infeasible)
    return scenarios


def build_manifest(split, scenarios, seed=20270116, config=None,
                   disturbance_seed=20270117, policy_seed=None):
    """Build a hash-addressed manifest object from scenario records."""
    if split not in SPLITS:
        raise ValueError("split must be one of %s" % (", ".join(SPLITS)))
    records = copy.deepcopy(list(scenarios))
    ids = [record.get("scenario_id") for record in records]
    if any(not value for value in ids) or len(set(ids)) != len(ids):
        raise ValueError("scenario_id values must be present and unique")
    manifest_config = _merge_config(config)
    # Record the effective count when the CLI requests a smoke-sized split.
    manifest_config["profiles"][split]["count"] = len(records)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "split": split,
        "scenario_seed": int(seed),
        "disturbance_seed": (None if disturbance_seed is None
                              else int(disturbance_seed)),
        "policy_seed": (None if policy_seed is None else int(policy_seed)),
        "config": manifest_config,
        "scenarios": records,
    }
    manifest["manifest_sha256"] = sha256_json(manifest)
    return manifest


def verify_manifest(manifest):
    """Return whether a manifest's content still matches its recorded hash."""
    recorded = manifest.get("manifest_sha256")
    if not recorded:
        return False
    payload = {key: value for key, value in manifest.items()
               if key != "manifest_sha256"}
    return recorded == sha256_json(payload)


def write_manifests(output_dir, *, seed=20270116, disturbance_seed=20270117,
                    policy_seed=None, config=None, counts=None, force=False):
    """Generate and atomically write split manifests and individual scenarios."""
    output_dir = os.path.abspath(output_dir)
    config = _merge_config(config)
    counts = counts or {}
    os.makedirs(output_dir, exist_ok=True)
    manifest_dir = os.path.join(output_dir, "manifests")
    scenario_dir = os.path.join(output_dir, "scenarios")
    os.makedirs(manifest_dir, exist_ok=True)
    os.makedirs(scenario_dir, exist_ok=True)
    frozen = os.path.join(manifest_dir, ".frozen")
    if os.path.exists(frozen):
        raise PermissionError("manifest directory is frozen and read-only")
    manifests = {}
    for split in SPLITS:
        target = os.path.join(manifest_dir, split + ".json")
        if os.path.exists(target) and not force:
            raise FileExistsError("%s exists; use --force to replace it" % target)
        scenarios = generate_scenarios(
            split, counts.get(split), seed, config, disturbance_seed, policy_seed)
        manifest = build_manifest(split, scenarios, seed, config,
                                  disturbance_seed, policy_seed)
        manifests[split] = manifest
        split_dir = os.path.join(scenario_dir, split)
        os.makedirs(split_dir, exist_ok=True)
        if force:
            keep = {scenario["scenario_id"] + ".json" for scenario in scenarios}
            for name in os.listdir(split_dir):
                if name.startswith("s-") and name.endswith(".json") and name not in keep:
                    os.unlink(os.path.join(split_dir, name))
        for scenario in scenarios:
            path = os.path.join(split_dir, scenario["scenario_id"] + ".json")
            if os.path.exists(path) and not force:
                raise FileExistsError("%s exists; use --force to replace it" % path)
            _atomic_json_write(path, scenario)
        _atomic_json_write(target, manifest)
    index = {"schema_version": SCHEMA_VERSION,
             "manifests": {split: manifests[split]["manifest_sha256"]
                           for split in SPLITS}}
    _atomic_json_write(os.path.join(manifest_dir, "index.json"), index)
    return manifests


def _atomic_json_write(path, value):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".scenario-", dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                    indent=2) + "\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _load_config(path):
    if not path:
        return None
    with open(path, encoding="utf-8") as stream:
        if path.endswith((".yaml", ".yml")):
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError("YAML config requires PyYAML; use JSON instead") from exc
            return yaml.safe_load(stream)
        return json.load(stream)


def _load_json(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def main(argv=None):
    parser = argparse.ArgumentParser(description="生成可复现的 Stage-2 场景清单")
    parser.add_argument("--output-dir", default="experiments/stage2",
                        help="输出目录（默认: experiments/stage2）")
    parser.add_argument("--config", help="JSON/YAML 配置覆盖文件")
    parser.add_argument("--scenario-seed", type=int, default=20270116)
    parser.add_argument("--disturbance-seed", type=int, default=20270117)
    parser.add_argument("--policy-seed", type=int, default=None)
    parser.add_argument("--train-count", type=int)
    parser.add_argument("--validation-count", type=int)
    parser.add_argument("--test-count", type=int)
    parser.add_argument("--force", action="store_true",
                        help="允许替换未冻结的已有清单")
    parser.add_argument("--freeze", action="store_true",
                        help="生成后写入 manifests/.frozen，后续默认拒绝覆盖")
    parser.add_argument("--check", metavar="SCENARIO",
                        help="只检查一个场景 JSON 的可行性，不生成清单")
    parser.add_argument("--verify", metavar="MANIFEST",
                        help="只验证 manifest_sha256，不生成清单")
    args = parser.parse_args(argv)
    if args.check and args.verify:
        parser.error("--check 和 --verify 不能同时使用")
    if args.check:
        result = check_feasibility(_load_json(args.check))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["feasible"] else 1
    if args.verify:
        result = verify_manifest(_load_json(args.verify))
        print("valid" if result else "invalid")
        return 0 if result else 1
    config = _load_config(args.config)
    counts = {"train": args.train_count, "validation": args.validation_count,
              "test": args.test_count}
    counts = {key: value for key, value in counts.items() if value is not None}
    manifests = write_manifests(
        args.output_dir, seed=args.scenario_seed,
        disturbance_seed=args.disturbance_seed, policy_seed=args.policy_seed,
        config=config, counts=counts, force=args.force)
    if args.freeze:
        marker = os.path.join(os.path.abspath(args.output_dir), "manifests", ".frozen")
        with open(marker, "w", encoding="utf-8") as stream:
            stream.write("Frozen by scenario_generator.py\n")
    for split in SPLITS:
        print("%s: %d scenarios sha256=%s" %
              (split, len(manifests[split]["scenarios"]),
               manifests[split]["manifest_sha256"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
