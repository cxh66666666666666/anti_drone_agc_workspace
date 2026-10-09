#!/usr/bin/env python3
"""生成 Stage-2 v4 场景集：难度与 v2 一致，仅做布局合理性优化 + 组合覆盖扩展。

与 v2（scenario_generator_v2.py）的差异：
1. 索引映射改为显式组合枚举：combo = index % 15、layout = combo % 5、
   trajectory = combo // 5、variant = index // 15。v2 用 index % 5 / index // 5
   映射，导致每个 layout 恒定绑定同一种轨迹，15 个组合只覆盖到 5 个；
2. 布局唯一调整手段 = 沿航迹平移 trajectory_progress，使出生点到障碍的
   水平净空 ≥ 2.6 m（reset 门限 0.24×10=2.4 m + 0.2 m 余量）。
   阶段 1 用统一 δ 平移全部障碍，保持布局刚性；阶段 1 无解时阶段 2 按
   progress 相同的 gate 分组微调（组内偏移量一致）。
   lateral_offset_m、尺寸、高度、bounds 等布置参数与 v2 逐值一致，
   train/validation 的夹缝布局改用「train 可解版」参数（figure_eight 例外，
   自交轨迹沿用 v2 原参数），并新增航迹可通行性度量与 train 门槛
   （见 passability_metrics / validate_scenario，figure_eight 不适用该门槛）。
   自交轨迹 × 夹缝布局（figure_eight × dense_corridor / curve_challenge）实测
   不可通过，直接不生成（见 EXCLUDED_COMBOS），故组合覆盖为 13/15。
   两阶段都无解则报错退出，绝不静默降级。
"""

import argparse
import copy
import hashlib
import json
import math
import os
import random
import tempfile

from geometry_metrics import minimum_geometry_clearance
from scenario_generator import check_feasibility
# 轨迹/起终点几何与 hash 工具直接复用 v2，避免两处定义漂移。
from scenario_generator_v2 import (LAYOUT_TYPES, SPLITS, TRAJECTORY_TYPES,
                                   _start_goal, canonical_json, sha256_json,
                                   trajectory_frame, verify_manifest)


SCHEMA_VERSION = "stage2-scenarios-v4"
SCENARIO_PREFIX = "s2v4-"
COMBO_COUNT = len(LAYOUT_TYPES) * len(TRAJECTORY_TYPES)   # 15 个 (trajectory, layout) 组合
PATH_SAMPLES = 400              # 航迹净空采样点数
SPAWN_SHIFT_STEP_M = 0.01       # 出生净空求解的 δ 搜索步长
SPAWN_SHIFT_MAX_M = 0.30        # 阶段 1 统一 δ 搜索上限
SPAWN_GROUP_SHIFT_MAX_M = 0.35  # 阶段 2 分组微调的单组 |Δprogress| 上限
SPAWN_CLEARANCE_TARGET_M = 2.6  # reset 门限 2.4 m + 0.2 m 余量
PROGRESS_MIN = 0.05             # 平移后 progress 下界
PROGRESS_MAX = 0.95             # 平移后 progress 上界

COLLISION_DIST_M = 0.65         # 风险带阈值，与 rule_avoidance.yaml 的 collision_dist 一致
TRAIN_MIN_CLEARANCE_M = 0.75    # train 级：最优偏移下的全程最小净空下限（风险带 0.65 m + 0.10 m 余量）
TRAIN_MIN_BAND_M = 0.6          # train 级：可行横向偏移带宽度下限（容忍约 ±0.3 m 跟踪误差）
PASSABILITY_OFFSET_RANGE_M = 2.5  # 可通行性度量的横向偏移搜索范围
PASSABILITY_OFFSET_STEP_M = 0.05  # 可通行性度量的横向偏移搜索步长
PASSABILITY_SAMPLES = 120       # 可通行性度量的航迹采样点数

# 不生成的自交轨迹 × 夹缝布局：实测逐点可达到的最大横向净空仅 0.09~0.39 m，
# 远小于实测跟踪误差（0.35~1.08 m），不加宽基本无法通过、加宽又会把障碍推到
# 另一叶航迹上（可行带从 0.1~0.4 m 掉到 0），故一律不生成（不入任何 split）。
EXCLUDED_COMBOS = frozenset({("figure_eight", "dense_corridor"),
                             ("figure_eight", "curve_challenge")})

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
        "train": {"count": 60},
        "validation": {"count": 15},
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


def combo_parts(index):
    """把 index 拆成 (combo, layout, trajectory 类型, variant)。"""
    index = int(index)
    combo = index % COMBO_COUNT
    layout = LAYOUT_TYPES[combo % len(LAYOUT_TYPES)]
    trajectory_type = TRAJECTORY_TYPES[combo // len(LAYOUT_TYPES)]
    variant = index // COMBO_COUNT
    return combo, layout, trajectory_type, variant


def _trajectory(trajectory_type, variant, config):
    """轨迹几何参数仍按 variant 变化，取值与 v2 完全相同。"""
    altitude = float(config["altitude"])
    duration = float(config["trajectory_duration"])
    omega = float(config["angular_velocity"])
    variation = variant % 3
    if trajectory_type == "linear":
        y = (-0.5, 0.0, 0.5)[variation]
        return {
            "type": trajectory_type,
            "start": [-8.0, y, altitude],
            "end": [8.0, y, altitude],
            "duration_s": duration,
            "loop": False,
        }
    if trajectory_type == "circular":
        radius = (4.2, 4.6, 5.0)[variation]
        return {
            "type": trajectory_type,
            "center": [0.0, 0.0, altitude],
            "radius_m": radius,
            "height_m": altitude,
            "angular_velocity_rad_s": omega,
            "duration_s": 2.0 * math.pi / omega,
            "loop": True,
        }
    return {
        "type": trajectory_type,
        "center": [0.0, 0.0, altitude],
        "radius_x_m": (4.2, 4.6, 5.0)[variation],
        "radius_y_m": (2.4, 2.7, 3.0)[variation],
        "height_m": altitude,
        "angular_velocity_rad_s": omega,
        "duration_s": 2.0 * math.pi / omega,
        "loop": True,
    }


def _size(shape, rng, config, unseen):
    if unseen:
        return [float(value) for value in config["test_unseen_sizes"][shape]]
    choices = config["sizes"][shape]
    return [float(value) for value in choices[rng.randrange(len(choices))]]


def obstacle_clearance_2d(point, obstacle):
    """点到障碍水平轮廓的净空（点在轮廓内记为 0）。

    cylinder：中心距 − max(size[0], size[1]) / 2；
    pillar_box / wall：到带 yaw 的 size[0]×size[1] 矩形的距离。
    注意：不含机体包络，仅用于出生净空与记录；航迹可通行性用
    passability_metrics（minimum_geometry_clearance，含包络）。
    """
    dx = float(point[0]) - float(obstacle["center"][0])
    dy = float(point[1]) - float(obstacle["center"][1])
    size = obstacle["size"]
    if obstacle["type"] == "cylinder":
        return max(math.hypot(dx, dy) - 0.5 * max(float(size[0]),
                                                  float(size[1])), 0.0)
    yaw = float(obstacle.get("yaw", 0.0))
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    outside_x = max(abs(local_x) - 0.5 * float(size[0]), 0.0)
    outside_y = max(abs(local_y) - 0.5 * float(size[1]), 0.0)
    return math.hypot(outside_x, outside_y)


def _difficulty(split, variant):
    """按 split/variant 选布局参数版本：train/validation 一律 train 版，
    test 的偶数 variant 用 extreme 版（极端版只出现在 test）。"""
    if split == "test" and variant % 2 == 0:
        return "extreme"
    return "train"


def passability_metrics(trajectory, obstacles, threshold_m=COLLISION_DIST_M,
                        offset_range_m=PASSABILITY_OFFSET_RANGE_M,
                        offset_step_m=PASSABILITY_OFFSET_STEP_M,
                        samples=PASSABILITY_SAMPLES):
    """扫描常值横向偏移，度量航迹可通行性（纯函数、确定性、无 RNG）。

    对每个偏移量把整条航迹沿法向平移，取全程最小净空。净空语义为
    minimum_geometry_clearance（已扣除 drone_half_extents），与安全滤波器一致。
    返回 band_width_m（全程净空 ≥ threshold_m 的最宽连续偏移区间宽度，无可行
    区间为 0.0）、band_center_m（该区间中心，无可行区间为 None）、
    best_min_clearance_m（max_over_offset(min_over_path(clearance))）。
    """
    if not obstacles:
        return {"band_width_m": 0.0, "band_center_m": None,
                "best_min_clearance_m": None}
    altitude = float(obstacles[0]["center"][2])  # 障碍与航迹同高
    frames = [(frame[0], frame[2]) for frame in
              (trajectory_frame(trajectory, index / float(samples - 1))
               for index in range(samples))]
    steps = int(round(offset_range_m / offset_step_m))
    offsets = [step * offset_step_m for step in range(-steps, steps + 1)]

    worst_per_offset = []
    for offset in offsets:
        worst = math.inf
        for point, normal in frames:
            shifted = (point[0] + offset * normal[0],
                       point[1] + offset * normal[1], altitude)
            value = minimum_geometry_clearance(shifted, obstacles)
            if value < worst:
                worst = value
        worst_per_offset.append(worst)

    # 取最宽连续可行区间（可能有多段，例如绕行障碍的左右两侧）。
    band_width, band_center, run_start = 0.0, None, None
    for position in range(len(offsets) + 1):
        feasible = (position < len(offsets) and
                    worst_per_offset[position] + 1e-9 >= threshold_m)
        if feasible and run_start is None:
            run_start = position
        elif not feasible and run_start is not None:
            width = offsets[position - 1] - offsets[run_start]
            if width > band_width + 1e-12:
                band_width = width
                band_center = 0.5 * (offsets[run_start] +
                                     offsets[position - 1])
            run_start = None
    return {"band_width_m": round(band_width, 4),
            "band_center_m": (None if band_center is None
                              else round(band_center, 4)),
            "best_min_clearance_m": round(max(worst_per_offset), 4)}


def _obstacle_specs(trajectory, layout, variant, difficulty="extreme"):
    """按布局给出障碍规格，每项为
    (shape, progress, lateral_offset_m, longitudinal, wall_alignment)。

    difficulty="extreme" 保留 v2 的夹缝参数（仅供 test），
    difficulty="train" 用可解版参数（加宽夹缝 / 去掉中心柱）。
    例外：figure_eight 是自交轨迹，加宽会把障碍推到另一叶航迹上（可行带
    反而从 0.1~0.4 m 掉到 0，且出生净空求解失败），故 figure_eight 一律
    沿用 v2 的原始夹缝参数。
    """
    mirror = -1.0 if variant % 2 else 1.0
    anchor = (0.30, 0.42, 0.56)[variant % 3]
    tight = difficulty == "extreme" or trajectory["type"] == "figure_eight"
    if layout == "frontal_block":
        return [("cylinder", anchor, 0.0, 0.0, "tangent")]
    if layout == "single_side":
        return [("pillar_box", anchor, 0.15 * mirror, 0.0, "tangent"),
                ("wall", anchor + 0.035, 1.05 * mirror, 0.0, "tangent")]
    if layout == "mirror_pair":
        # train 版 ±1.75（最大柱包络后可行带约 1.1 m），extreme 版保留 ±0.95。
        lateral = 0.95 if tight else 1.75
        return [("pillar_box", anchor, lateral, 0.0, "tangent"),
                ("pillar_box", anchor, -lateral, 0.0, "tangent")]
    if layout == "dense_corridor":
        dense_progress = ((0.18, 0.32, 0.68)
                          if trajectory["type"] == "figure_eight"
                          else (0.27, 0.43, 0.59))
        # train 版两侧墙加宽到 ±1.90 且去掉中心柱，extreme 版保留原参数。
        lateral = 1.10 if tight else 1.90
        specs = []
        for progress in dense_progress:
            specs.append(("wall", progress, lateral, 0.0, "tangent"))
            specs.append(("wall", progress, -lateral, 0.0, "tangent"))
        if tight:
            center_progress = (0.72 if trajectory["type"] == "figure_eight"
                               else 0.50)
            specs.append(("pillar_box", center_progress, 0.35 * mirror, 0.0,
                          "tangent"))
        return specs
    if layout == "curve_challenge":
        curve_points = (((0.12, 0.0), (0.30, 0.25 * mirror),
                         (0.70, -0.20 * mirror))
                        if trajectory["type"] == "figure_eight" else
                        ((0.18, 0.0), (0.52, 0.25 * mirror),
                         (0.78, -0.20 * mirror)))
        return [("cylinder", progress, lateral, 0.0, "tangent")
                for progress, lateral in curve_points]
    raise ValueError("unknown layout: %s" % layout)


def _build_obstacles(trajectory, specs, sizes, progress_offsets, altitude):
    """按逐障碍 progress 偏移生成障碍（相对几何与 lateral 完全不变）。

    progress_offsets 与 specs 等长，第 i 项只加到第 i 个 spec 的 progress 上。
    """
    obstacles = []
    for item_index, (shape, progress, lateral, longitudinal,
                     wall_alignment) in enumerate(specs):
        offset = float(progress_offsets[item_index])
        point, tangent, normal = trajectory_frame(trajectory,
                                                  progress + offset)
        x = point[0] + longitudinal * tangent[0] + lateral * normal[0]
        y = point[1] + longitudinal * tangent[1] + lateral * normal[1]
        yaw = math.atan2(tangent[1], tangent[0])
        if wall_alignment == "normal":
            yaw += math.pi / 2.0
        obstacles.append({
            "id": "o%02d" % item_index,
            "group": "a" if item_index % 2 == 0 else "b",
            "type": shape,
            "center": [round(x, 3), round(y, 3), altitude],
            "size": sizes[item_index],
            "yaw": round(yaw if shape == "wall" else 0.0, 6),
            "trajectory_progress": round(progress + offset, 4),
            "lateral_offset_m": round(lateral, 3),
        })
    return obstacles


def _shift_candidates(max_shift_m):
    """偏移候选序列，按 |Δ| 升序：0, +step, -step, ..., +max, -max。"""
    steps = int(round(max_shift_m / SPAWN_SHIFT_STEP_M))
    for step in range(steps + 1):
        magnitude = step * SPAWN_SHIFT_STEP_M
        if magnitude == 0.0:
            yield 0.0
            continue
        yield magnitude
        yield -magnitude


def _spawn_shift_candidates():
    """阶段 1 的统一 δ 候选序列（|δ| ≤ SPAWN_SHIFT_MAX_M）。"""
    return _shift_candidates(SPAWN_SHIFT_MAX_M)


def _solve_spawn_clearance(trajectory, specs, sizes, altitude,
                           target_m=SPAWN_CLEARANCE_TARGET_M, *,
                           layout=None, variant=None):
    """求逐障碍 progress 偏移，使出生点水平净空 ≥ target_m。

    返回 (uniform_shift, progress_offsets, obstacles, clearance)。

    阶段 1：统一平移全部 progress（保持布局刚性），对 _spawn_shift_candidates()
    的 δ 升序搜索，平移后所有 progress 须落在 [PROGRESS_MIN, PROGRESS_MAX]，
    首个达标者立即返回。
    阶段 2（阶段 1 无解）：按 progress 值分组（同一 gate 的对称障碍同组，
    key=round(progress, 6)），以「各障碍净空截断和」为目标做分组坐标上升，
    每组取使目标严格增大的最佳 Δ（平局取 |Δ| 更小者）；任意时刻达标即返回。
    两阶段都无解则抛 RuntimeError，绝不返回降级结果。
    """
    spawn, _, _ = trajectory_frame(trajectory, 0.0)
    count = len(specs)
    progress_list = [float(spec[1]) for spec in specs]

    def build(offsets):
        items = _build_obstacles(trajectory, specs, sizes, offsets, altitude)
        value = min(obstacle_clearance_2d(spawn, item) for item in items)
        return items, value

    # 阶段 1：统一 δ 平移，保持布局刚性。
    for shift in _spawn_shift_candidates():
        if any(not (PROGRESS_MIN - 1e-9 <= progress + shift <=
                    PROGRESS_MAX + 1e-9) for progress in progress_list):
            continue
        obstacles, clearance = build([shift] * count)
        if clearance + 1e-9 >= target_m:
            return shift, [shift] * count, obstacles, clearance

    # 阶段 2：按 progress 分组做坐标上升微调。
    groups = {}
    for position, progress in enumerate(progress_list):
        groups.setdefault(round(progress, 6), []).append(position)
    keys = sorted(groups)
    offsets = [0.0] * count
    obstacles, clearance = build(offsets)

    def objective(items):
        # 截断和：避免只盯全局最小值而卡在局部最优。
        return sum(min(obstacle_clearance_2d(spawn, item), target_m)
                   for item in items)

    current = objective(obstacles)
    for _ in range(4 * len(keys) + 4):
        accepted = False
        for key in keys:
            members = groups[key]
            best_delta, best_value, best_items = None, current, None
            for delta in _shift_candidates(SPAWN_GROUP_SHIFT_MAX_M):
                if any(not (PROGRESS_MIN - 1e-9 <=
                            progress_list[pos] + offsets[pos] + delta <=
                            PROGRESS_MAX + 1e-9) for pos in members):
                    continue
                trial = list(offsets)
                for position in members:
                    trial[position] += delta
                items = _build_obstacles(trajectory, specs, sizes, trial,
                                         altitude)
                value = objective(items)
                # 候选按 |Δ| 升序，严格大于才替换 -> 平局取 |Δ| 更小者。
                if value > best_value + 1e-9:
                    best_delta, best_value, best_items = delta, value, items
            if best_delta is None:
                continue
            offsets = [value + best_delta if position in members else value
                       for position, value in enumerate(offsets)]
            current, obstacles = best_value, best_items
            accepted = True
            clearance = min(obstacle_clearance_2d(spawn, item)
                            for item in obstacles)
            if clearance + 1e-9 >= target_m:
                return 0.0, list(offsets), obstacles, clearance
        if not accepted:
            break

    clearance = min(obstacle_clearance_2d(spawn, item) for item in obstacles)
    if clearance + 1e-9 < target_m:
        limiting = min(obstacles, key=lambda item: obstacle_clearance_2d(spawn,
                                                                        item))
        raise RuntimeError(
            "出生净空不可达：trajectory=%s layout=%s variant=%s 最小净空="
            "%.4f m < 目标 %.4f m，受限障碍 id=%s progress=%.4f lateral=%.3f"
            "（两阶段均无解，拒绝降级）" % (
                trajectory["type"], layout, variant, clearance, target_m,
                limiting["id"], limiting["trajectory_progress"],
                limiting["lateral_offset_m"]))
    return 0.0, list(offsets), obstacles, clearance


def validate_scenario(scenario, spawn_shift=0.0, progress_adjustments=None,
                      path_samples=PATH_SAMPLES):
    """记录单个场景的布局指标，返回 JSON 安全 dict。

    记录出生净空与航迹可通行性度量（含机体包络），并对 train/validation
    施加 train 门槛；不达标一律抛错，不静默降级。
    """
    trajectory = scenario["trajectory"]
    obstacles = scenario.get("obstacles", [])
    spawn, _, _ = trajectory_frame(trajectory, 0.0)
    spawn_clearances = [obstacle_clearance_2d(spawn, item)
                        for item in obstacles]
    path_clearances = []
    if obstacles:
        points = [trajectory_frame(trajectory, index / float(path_samples - 1))[0]
                  for index in range(path_samples)]
        path_clearances = [obstacle_clearance_2d(point, item)
                           for point in points for item in obstacles]
    adjustments = ([0.0] * len(obstacles) if progress_adjustments is None
                  else list(progress_adjustments))
    spawn_clearance = (None if not spawn_clearances
                       else round(min(spawn_clearances), 4))
    # 最后一道防线：出生净空必须达标，否则拒绝输出降级场景。
    if spawn_clearance is None or spawn_clearance < SPAWN_CLEARANCE_TARGET_M:
        raise RuntimeError(
            "出生净空未达标：scenario_id=%s clearance=%s target=%.4f" % (
                scenario.get("scenario_id"), spawn_clearance,
                SPAWN_CLEARANCE_TARGET_M))
    layout = scenario.get("layout", {})
    metrics = passability_metrics(trajectory, obstacles, COLLISION_DIST_M)
    # train/validation 的场景必须留出足够宽的可行带，否则拒绝输出。
    # figure_eight 例外：自交轨迹下任何侧向障碍布置都落在另一叶航迹附近，
    # band_width 结构性趋近 0，该门槛不适用（仍受出生净空这一硬门限约束）。
    if (scenario.get("split") in ("train", "validation") and
            trajectory.get("type") != "figure_eight"):
        if (metrics["best_min_clearance_m"] < TRAIN_MIN_CLEARANCE_M or
                metrics["band_width_m"] < TRAIN_MIN_BAND_M):
            raise RuntimeError(
                "航迹可通行性未达标：scenario_id=%s layout=%s variant=%s "
                "best_min_clearance=%.4f m < %.4f m 或 band_width=%.4f m < "
                "%.4f m（train 门槛，拒绝降级）" % (
                    scenario.get("scenario_id"), layout.get("type"),
                    layout.get("variant"), metrics["best_min_clearance_m"],
                    TRAIN_MIN_CLEARANCE_M, metrics["band_width_m"],
                    TRAIN_MIN_BAND_M))
    return {
        "spawn_clearance_m": spawn_clearance,
        "spawn_clearance_target_m": SPAWN_CLEARANCE_TARGET_M,
        "min_path_clearance_m": (None if not path_clearances
                                 else round(min(path_clearances), 4)),
        "band_width_m": metrics["band_width_m"],
        "band_center_m": metrics["band_center_m"],
        "best_min_clearance_m": metrics["best_min_clearance_m"],
        "difficulty": layout.get("difficulty"),
        "lateral_offsets": [item["lateral_offset_m"] for item in obstacles],
        "combo": layout.get("combo"),
        "obstacle_count": len(obstacles),
        "spawn_shift": round(float(spawn_shift), 4),
        "spawn_progress_adjustments": [round(float(value), 4)
                                       for value in adjustments],
    }


def generate_layout(seed, split="train", index=0, config=None,
                    disturbance_seed=20270117, policy_seed=None):
    if split not in SPLITS:
        raise ValueError("split must be one of %s" % ", ".join(SPLITS))
    config = _merge_config(config)
    index = int(index)
    effective_seed = int(seed) + {"train": 0, "validation": 1000003,
                                  "test": 2000003}[split] + index * 7919
    rng = random.Random(effective_seed)
    combo, layout, trajectory_type, variant = combo_parts(index)
    trajectory = _trajectory(trajectory_type, variant, config)
    difficulty = _difficulty(split, variant)
    specs = _obstacle_specs(trajectory, layout, variant, difficulty)
    unseen = split == "test" and variant % 2 == 1
    # 尺寸一次性抽样，使 δ 搜索不影响尺寸序列。
    sizes = [_size(spec[0], rng, config, unseen) for spec in specs]
    altitude = float(config["altitude"])
    shift, offsets, obstacles, _ = _solve_spawn_clearance(
        trajectory, specs, sizes, altitude, layout=layout, variant=variant)
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
            "combo": combo,
            "mirror_sign": -1 if variant % 2 else 1,
            "uses_unseen_size": bool(obstacles and unseen),
            "difficulty": difficulty,
        },
        "bounds": [float(value) for value in config["bounds"]],
        "start": start,
        "goal": goal,
        "altitude": altitude,
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
    scenario["layout_validation"] = validate_scenario(scenario, shift,
                                                      offsets)
    # scenario_id 只由 v2 那组固定键计算，新增顶层键不影响 id。
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
    scenarios = []
    for index in range(int(count)):
        _, layout, trajectory_type, _ = combo_parts(index)
        if (trajectory_type, layout) in EXCLUDED_COMBOS:
            # 自交轨迹 × 夹缝布局不生成（理由见 EXCLUDED_COMBOS 注释），
            # 其余索引保持原编号，只影响最终条数与组合覆盖数。
            continue
        scenarios.append(generate_layout(seed, split, index, config,
                                         disturbance_seed, policy_seed))
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


def coverage_summary(manifests):
    """统计每个 split 的组合覆盖与净空范围，作为生成结果的校验证据。"""
    report = {}
    for split, manifest in manifests.items():
        distribution = {}
        combos = set()
        spawns, paths = [], []
        adjusted = []
        for scenario in manifest["scenarios"]:
            layout = scenario["layout"]
            key = "%s|%s|%+d|%s" % (
                scenario["trajectory"]["type"], layout["type"],
                layout["mirror_sign"], layout["uses_unseen_size"])
            distribution[key] = distribution.get(key, 0) + 1
            combos.add((scenario["trajectory"]["type"], layout["type"]))
            validation = scenario["layout_validation"]
            spawns.append(validation["spawn_clearance_m"])
            paths.append(validation["min_path_clearance_m"])
            adjustments = validation["spawn_progress_adjustments"]
            if any(abs(value) > 1e-9 for value in adjustments):
                adjusted.append({
                    "scenario_id": scenario["scenario_id"],
                    "spawn_progress_adjustments": adjustments,
                })
        # 出生净空是硬约束：任何 split 出现不达标即视为生成失败。
        if min(spawns) < SPAWN_CLEARANCE_TARGET_M:
            raise RuntimeError(
                "%s 出生净空未达标：min=%.4f < target=%.4f" % (
                    split, min(spawns), SPAWN_CLEARANCE_TARGET_M))
        report[split] = {
            "count": len(manifest["scenarios"]),
            "combo_count": len(combos),
            "trajectory_layout_combos": sorted(
                "%s|%s" % item for item in combos),
            "distribution": distribution,
            "spawn_clearance_min_m": min(spawns),
            "spawn_clearance_max_m": max(spawns),
            "path_clearance_min_m": min(paths),
            "spawn_clearance_target_m": SPAWN_CLEARANCE_TARGET_M,
            "spawn_progress_adjusted_count": len(adjusted),
            "spawn_progress_adjusted": adjusted,
        }
    return report


def _atomic_json_write(path, value):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".scenario-v4-",
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
            # 只清理本版本前缀的旧文件，绝不触碰 v2/v3 数据。
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


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="生成轨迹/布局全组合覆盖的 Stage-2 v4 场景清单")
    parser.add_argument("--output-dir", default="experiments/stage2_v4")
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
            stream.write("Frozen by scenario_generator_v4.py\n")
    for split in SPLITS:
        print("%s: %d scenarios sha256=%s" % (
            split, len(manifests[split]["scenarios"]),
            manifests[split]["manifest_sha256"]))
    print(json.dumps(coverage_summary(manifests), ensure_ascii=False,
                     indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())