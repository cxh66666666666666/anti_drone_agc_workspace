#!/usr/bin/env python3
"""生成 Stage-2 v3 场景集：修正 v2 的航迹净空与出生净空缺陷，并加入垂直逃逸变体。

与 v2（scenario_generator_v2.py）的差异只在「障碍放置约束」：
1. 障碍到参考航迹的净空（机体包络→障碍表面）下限默认 1.0 m（覆盖实测
   0.7~1.0 m 跟踪误差），同时必须 < 2.4 m（safe_distance_norm ×
   lidar_max_range），否则无人机永远触发不到避障；
2. 出生点到障碍轮廓的水平距离 ≥ reset_min_clearance_norm × lidar_max_range
   （与 gazebo_env reset 的 start_obstacle_min 同度量）；
3. 每 N 个场景使用「低矮障碍」变体：障碍顶面低于飞行高度，留出垂直逃逸通道。

轨迹定义、场景 schema 与 manifest 组织方式全部沿用 v2，保证与 v2 可比。
"""

import argparse
import copy
import hashlib
import json
import math
import os
import random
import tempfile

from geometry_metrics import (DEFAULT_DRONE_HALF_EXTENTS,
                              obstacle_surface_clearance)
from path_planning import obstacle_clearance_2d
from scenario_generator import check_feasibility
# 轨迹/起终点几何直接复用 v2 实现，避免两处定义漂移导致不可比。
from scenario_generator_v2 import (LAYOUT_TYPES, SPLITS, TRAJECTORY_TYPES,
                                   _start_goal, _trajectory, canonical_json,
                                   sha256_json, trajectory_frame,
                                   verify_manifest)


SCHEMA_VERSION = "stage2-scenarios-v3"
SCENARIO_PREFIX = "s2v3-"
PATH_SAMPLES = 1025         # 航迹离散采样数（生成与校验共用，保证判定自洽）
ROUNDING_EPS = 0.01         # 坐标四舍五入到 1 mm 后的裕量
LATERAL_STEP = 0.05         # 横向偏移搜索步长
ANCHOR_STEP = 0.03          # 锚点沿航迹平移步长（也用于成对布局的错位）
ANCHOR_TRIES = 30           # 锚点平移重试上限

DEFAULT_CONFIG = {
    # ---- 与 v2 完全一致的物理/轨迹参数 ----
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
    # ---- v3 新增约束 ----
    # 障碍离航迹净空下限：机体包络（drone_half_extents）到障碍表面的最小距离。
    "min_obstacle_clearance_m": 1.0,
    # 净空下限之上的确定性抖动，使「贴近」与「从容」两类遭遇都有覆盖。
    "obstacle_clearance_jitter_m": 0.4,
    # 超过该距离不会触发避障事件（safe_distance_norm 0.24 × lidar_max_range 10），
    # 因此净空必须严格小于它，否则任务退化为无障碍。
    "max_encounter_clearance_m": 2.4,
    "lidar_max_range": 10.0,
    "reset_min_clearance_norm": 0.24,
    # 出生净空额外余量：吸收复位后位置偏差与 LiDAR 量化误差。
    "start_clearance_margin_m": 0.2,
    "max_lateral_offset_m": 3.0,
    "low_profile": {
        "enabled": True,
        "every": 3,                 # 每 3 个 index 生成一个垂直逃逸变体
        "height_m": 1.6,            # 障碍高度（顶面 = altitude - top_below_altitude_m）
        "top_below_altitude_m": 0.2,
        "vertical_margin_m": 0.1,   # 垂直逃逸判定余量
    },
}


def _merge_config(config=None):
    merged = copy.deepcopy(DEFAULT_CONFIG)
    if not config:
        return merged
    for key, value in config.items():
        if key in ("profiles", "sizes", "test_unseen_sizes", "low_profile"):
            merged[key].update(copy.deepcopy(value))
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _size(shape, rng, config, unseen):
    if unseen:
        return [float(value) for value in config["test_unseen_sizes"][shape]]
    choices = config["sizes"][shape]
    return [float(value) for value in choices[rng.randrange(len(choices))]]


def path_points(trajectory, samples=PATH_SAMPLES):
    """把参考航迹离散成 3D 采样点（z 取飞行高度）。"""
    height = float(trajectory.get(
        "height_m", trajectory.get("start", [0.0, 0.0, 0.0])[2]))
    return [trajectory_frame(trajectory, index / float(samples - 1))[0] +
            (height,) for index in range(samples)]


def obstacle_path_clearance(points, obstacle):
    """机体包络到障碍表面的最小净空（负值 = 包络穿入障碍）。"""
    return min(obstacle_surface_clearance(point, obstacle)
               for point in points)


def _obstacle_top(obstacle):
    return (float(obstacle["center"][2]) + 0.5 * float(obstacle["size"][2]))


def has_vertical_escape(obstacles, altitude, config):
    """所有障碍顶面低于飞行高度（含机体半高与余量）时存在垂直逃逸通道。"""
    if not obstacles:
        return False
    limit = (float(altitude) - DEFAULT_DRONE_HALF_EXTENTS[2] -
             float(config["low_profile"]["vertical_margin_m"]))
    return all(_obstacle_top(item) <= limit for item in obstacles)


def analyze_scenario(scenario, config=None):
    """计算场景净空指标；生成器与校验器共用同一实现。"""
    config = _merge_config(config)
    obstacles = scenario.get("obstacles", [])
    points = path_points(scenario["trajectory"])
    start = scenario["start"]
    altitude = float(scenario.get("altitude", config["altitude"]))
    lids = float(config["lidar_max_range"])
    clearances = [obstacle_path_clearance(points, item) for item in obstacles]
    # 出生净空与 reset 的 start_obstacle_min 同度量：无人机中心到障碍水平
    # 轮廓的距离 / lidar_max_range。
    starts = [obstacle_clearance_2d(start, item) for item in obstacles]
    start_min = min(starts) if starts else None
    return {
        "obstacle_clearance_min_m": (None if not clearances
                                     else round(min(clearances), 4)),
        "obstacle_clearance_max_m": (None if not clearances
                                     else round(max(clearances), 4)),
        "start_clearance_min_m": (None if start_min is None
                                  else round(start_min, 4)),
        "start_clearance_min_norm": (None if start_min is None
                                     else round(start_min / lids, 4)),
        "obstacle_top_max_m": (None if not obstacles
                               else round(max(_obstacle_top(item)
                                              for item in obstacles), 4)),
        "vertical_escape": has_vertical_escape(obstacles, altitude, config),
    }


def validate_scenario(scenario, config=None):
    """按 v3 规则校验单个场景，返回 JSON 安全的结果与失败原因。"""
    config = _merge_config(config)
    metrics = analyze_scenario(scenario, config)
    failures = []
    start_norm = metrics["start_clearance_min_norm"]
    if start_norm is not None and start_norm + 1e-9 < float(
            config["reset_min_clearance_norm"]):
        failures.append("start_clearance_below_reset_min")
    clearance = metrics["obstacle_clearance_min_m"]
    if clearance is None:
        failures.append("no_obstacles")
    else:
        if clearance + 1e-9 < float(config["min_obstacle_clearance_m"]):
            failures.append("obstacle_clearance_below_min")
        if clearance >= float(config["max_encounter_clearance_m"]):
            failures.append("obstacle_clearance_above_encounter_limit")
    result = dict(metrics)
    result.update({
        "scenario_id": scenario.get("scenario_id"),
        "split": scenario.get("split"),
        "ok": not failures,
        "failures": failures,
    })
    return result


def _footprint_support(shape, size, yaw, normal):
    """障碍轮廓沿航迹法线方向的最大半宽（横向偏移的一阶估计）。"""
    if shape == "cylinder":
        return 0.5 * max(float(size[0]), float(size[1]))
    local_x = math.cos(yaw) * normal[0] + math.sin(yaw) * normal[1]
    local_y = -math.sin(yaw) * normal[0] + math.cos(yaw) * normal[1]
    return 0.5 * (abs(local_x) * float(size[0]) +
                  abs(local_y) * float(size[1]))


def _solve_placement(trajectory, points, start, shape, size, progress, side,
                     longitudinal, target, start_min, config, z_center):
    """求解满足「航迹净空带」与「出生净空」的障碍放置。

    返回 ``(障碍, None)`` 或 ``(None, 失败原因)``；原因用于区分「该侧几何
    不可行」（换侧）与「出生净空不足」（沿航迹平移锚点）。
    """
    point, tangent, normal = trajectory_frame(trajectory, progress)
    yaw = math.atan2(tangent[1], tangent[0]) if shape == "wall" else 0.0
    # 已经低于飞行高度的障碍（低矮变体）其纵向分离量也计入净空，需从目标
    # 净空中扣除后再分配水平部分。
    vertical = max(float(z_center) + 0.5 * float(size[2]) +
                   DEFAULT_DRONE_HALF_EXTENTS[2] -
                   float(config["altitude"]), 0.0)
    horizontal_target = math.sqrt(max(target * target - vertical * vertical, 0.0))
    magnitude = horizontal_target + _footprint_support(shape, size, yaw, normal)
    limit = float(config["max_lateral_offset_m"])
    while magnitude <= limit:
        lateral = side * magnitude
        item = {
            "id": "o00",
            "group": "a",
            "type": shape,
            "center": [round(point[0] + longitudinal * tangent[0] +
                             lateral * normal[0], 3),
                       round(point[1] + longitudinal * tangent[1] +
                             lateral * normal[1], 3),
                       round(float(z_center), 3)],
            "size": [float(value) for value in size],
            "yaw": round(yaw, 6),
            "trajectory_progress": round(progress, 4),
            "lateral_offset_m": round(lateral, 3),
        }
        if (obstacle_path_clearance(points, item) >=
                target + ROUNDING_EPS):
            # 出生净空不足由调用方平移锚点解决，此处不放大横向偏移。
            if obstacle_clearance_2d(start, item) + 1e-9 < start_min:
                return None, "start_clearance"
            return item, None
        magnitude += LATERAL_STEP
    return None, "path_clearance"


def _low_profile_z(config, low_profile):
    if not low_profile:
        return float(config["altitude"])
    height = float(config["low_profile"]["height_m"])
    top = float(config["altitude"]) - float(
        config["low_profile"]["top_below_altitude_m"])
    return top - 0.5 * height


def _figure_eight_anchor(progress, trajectory):
    """8 字轨迹在 u≈0.5 处回到出生点，锚点必须避开该邻域。"""
    if trajectory["type"] != "figure_eight":
        return progress
    if 0.36 <= progress <= 0.64:
        return 0.36 if progress < 0.5 else 0.64
    return progress


def _obstacles_for_layout(trajectory, layout, variant, split, rng, config,
                          low_profile):
    obstacles = []
    unseen = split == "test" and variant % 2 == 1
    mirror = -1.0 if variant % 2 else 1.0
    points = path_points(trajectory)
    start, _ = _start_goal(trajectory)
    start_min = (float(config["reset_min_clearance_norm"]) *
                 float(config["lidar_max_range"]) +
                 float(config["start_clearance_margin_m"]))
    z_center = _low_profile_z(config, low_profile)

    def add(shape, progress, side, longitudinal=0.0):
        """放置障碍。返回 (实际锚点, 实际侧向) 供成对/走廊布局复用。

        先试布局指定的侧向，几何上不可行时换到对侧（8 字轨迹内侧在交叉区
        附近无法同时满足航迹净空）；出生净空不足则沿航迹前移锚点。
        """
        size = _size(shape, rng, config, unseen)
        if low_profile:
            size = [size[0], size[1],
                    float(config["low_profile"]["height_m"])]
        # 每个障碍在净空下限之上取确定性抖动，避免所有障碍都贴在同一条线上。
        target = (float(config["min_obstacle_clearance_m"]) +
                  rng.uniform(0.0, float(config["obstacle_clearance_jitter_m"])))
        for attempt in range(ANCHOR_TRIES):
            anchor = min(progress + ANCHOR_STEP * attempt, 0.95)
            reasons = []
            for candidate in (side, -side):
                item, reason = _solve_placement(
                    trajectory, points, start, shape, size, anchor, candidate,
                    longitudinal, target, start_min, config, z_center)
                if item is None:
                    reasons.append(reason)
                    continue
                item["id"] = "o%02d" % len(obstacles)
                item["group"] = "a" if len(obstacles) % 2 == 0 else "b"
                obstacles.append(item)
                return anchor, candidate
            if "start_clearance" not in reasons:
                # 两侧都只是几何不可行，继续平移锚点也无济于事。
                break
        raise RuntimeError(
            "无法为 %s/%s(progress=%.2f, target=%.2f) 放置满足净空约束的障碍"
            % (layout, shape, progress, target))

    def add_pair(shape, progress, side, longitudinal=0.0):
        """成对布局：优先对称放置，对侧不可行时错位保持成对（不叠放）。"""
        placed, used = add(shape, progress, side, longitudinal)
        add(shape, min(placed + ANCHOR_STEP, 0.95), -used, longitudinal)

    anchor = _figure_eight_anchor((0.30, 0.42, 0.56)[variant % 3], trajectory)
    if layout == "frontal_block":
        add("cylinder", anchor, mirror)
    elif layout == "single_side":
        placed, side = add("pillar_box", anchor, mirror)
        # 同侧薄墙略带纵向错位，构成「近柱 + 远墙」两级遭遇。
        add("wall", min(placed + 0.035, 0.95), side)
    elif layout == "mirror_pair":
        add_pair("pillar_box", anchor, mirror)
    elif layout == "dense_corridor":
        dense_progress = ((0.18, 0.32, 0.68)
                          if trajectory["type"] == "figure_eight"
                          else (0.27, 0.43, 0.59))
        for progress in dense_progress:
            add_pair("wall", _figure_eight_anchor(progress, trajectory), mirror)
        center_progress = (0.72 if trajectory["type"] == "figure_eight"
                           else 0.50)
        add("pillar_box", _figure_eight_anchor(center_progress, trajectory),
            mirror)
    elif layout == "curve_challenge":
        # 曲线航迹取其转弯段；线性轨迹提供同模板的低曲率对照。
        curve_points = (((0.12, 1.0), (0.30, 1.0), (0.70, -1.0))
                        if trajectory["type"] == "figure_eight" else
                        ((0.22, 1.0), (0.52, 1.0), (0.78, -1.0)))
        for progress, side_sign in curve_points:
            add("cylinder", _figure_eight_anchor(progress, trajectory),
                mirror * side_sign)
    else:
        raise ValueError("unknown layout: %s" % layout)
    return obstacles


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
    low_every = int(config["low_profile"]["every"])
    low_profile = bool(config["low_profile"]["enabled"] and
                       low_every > 0 and index % low_every == 0)
    obstacles = _obstacles_for_layout(
        trajectory, layout, variant, split, rng, config, low_profile)
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
            "low_profile": low_profile,
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
            "min_obstacle_clearance_m": float(
                config["min_obstacle_clearance_m"]),
        },
    }
    scenario["clearance"] = analyze_scenario(scenario, config)
    scenario["feasibility"] = check_feasibility(scenario)
    identity = {key: scenario[key] for key in (
        "schema_version", "split", "index", "layout", "bounds", "start",
        "goal", "trajectory", "obstacles", "planner")}
    scenario["scenario_id"] = SCENARIO_PREFIX + hashlib.sha1(
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
    # 任一场景不达标即拒绝产出，避免把已知缺陷带进训练。
    failed = {}
    for item in scenarios:
        result = validate_scenario(item, config)
        if not result["ok"]:
            failed[item["scenario_id"]] = result["failures"]
    if failed:
        raise RuntimeError("generated scenarios failed validation: %s" % failed)
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


def _atomic_json_write(path, value):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".scenario-v3-",
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
                if (name.startswith(SCENARIO_PREFIX) and
                        name.endswith(".json") and name not in keep):
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


def validate_manifests(source, config=None):
    """校验一个场景集目录（或单个 manifest 文件），返回 JSON 安全报告。"""
    source = os.path.abspath(source)
    if os.path.isdir(source):
        index_path = os.path.join(source, "manifests", "index.json")
        index = None
        if os.path.exists(index_path):
            with open(index_path, encoding="utf-8") as stream:
                index = json.load(stream)
        paths = [os.path.join(source, "manifests", split + ".json")
                 for split in SPLITS]
    else:
        index, paths = None, [source]
    report = {"source": source, "splits": {}, "failures": {}, "ok": True}
    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as stream:
            manifest = json.load(stream)
        split = manifest.get("split", os.path.basename(path))
        results = [validate_scenario(item, config)
                   for item in manifest.get("scenarios", [])]
        failures = {item["scenario_id"]: item["failures"]
                    for item in results if not item["ok"]}
        clearances = [item["obstacle_clearance_min_m"] for item in results
                      if item["obstacle_clearance_min_m"] is not None]
        starts = [item["start_clearance_min_norm"] for item in results
                  if item["start_clearance_min_norm"] is not None]
        report["splits"][split] = {
            "count": len(results),
            "hash_valid": bool(verify_manifest(manifest)),
            "index_hash_valid": (None if index is None else
                                 index.get("manifests", {}).get(split) ==
                                 manifest.get("manifest_sha256")),
            "failures": len(failures),
            "vertical_escape": sum(1 for item in results
                                   if item["vertical_escape"]),
            "obstacle_clearance_min_m": min(clearances) if clearances else None,
            "obstacle_clearance_max_m": max(clearances) if clearances else None,
            "start_clearance_min_norm": min(starts) if starts else None,
        }
        report["failures"].update(failures)
        report["ok"] = report["ok"] and not failures
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="生成/校验轨迹匹配的 Stage-2 v3 场景清单")
    parser.add_argument("--output-dir", default="experiments/stage2_v3")
    parser.add_argument("--scenario-seed", type=int, default=20270116)
    parser.add_argument("--disturbance-seed", type=int, default=20270117)
    parser.add_argument("--policy-seed", type=int)
    parser.add_argument("--train-count", type=int)
    parser.add_argument("--validation-count", type=int)
    parser.add_argument("--test-count", type=int)
    parser.add_argument("--min-obstacle-clearance-m", type=float,
                        help="障碍离参考航迹净空下限（米）")
    parser.add_argument("--start-clearance-margin-m", type=float,
                        dest="start_clearance_margin_m",
                        help="出生净空在 reset 门限之上的额外余量（米）")
    parser.add_argument("--low-profile-every", type=int,
                        help="每 N 个场景生成一个垂直逃逸变体（0=关闭）")
    parser.add_argument("--validate", metavar="DIR_OR_MANIFEST",
                        help="只校验场景集/清单，不生成")
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
    config = {}
    if args.min_obstacle_clearance_m is not None:
        config["min_obstacle_clearance_m"] = args.min_obstacle_clearance_m
    if args.start_clearance_margin_m is not None:
        config["start_clearance_margin_m"] = args.start_clearance_margin_m
    if args.low_profile_every is not None:
        config["low_profile"] = {"every": args.low_profile_every}
    if args.validate:
        report = validate_manifests(args.validate, config or None)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ok"] else 1
    counts = {"train": args.train_count,
              "validation": args.validation_count,
              "test": args.test_count}
    counts = {key: value for key, value in counts.items()
              if value is not None}
    manifests = write_manifests(
        args.output_dir, seed=args.scenario_seed,
        disturbance_seed=args.disturbance_seed,
        policy_seed=args.policy_seed, config=config or None,
        counts=counts, force=args.force)
    if args.freeze:
        marker = os.path.join(os.path.abspath(args.output_dir),
                              "manifests", ".frozen")
        with open(marker, "w", encoding="utf-8") as stream:
            stream.write("Frozen by scenario_generator_v3.py\n")
    for split in SPLITS:
        print("%s: %d scenarios sha256=%s" % (
            split, len(manifests[split]["scenarios"]),
            manifests[split]["manifest_sha256"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())