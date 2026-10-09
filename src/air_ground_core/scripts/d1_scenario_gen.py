#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""d1_scenario_gen.py —— 阶段3 D1 随机化场景生成器与 manifest 构建（纯离线，无 ROS 依赖）。

用途：
    由单一主种子(master_seed)确定性生成评估用随机场景族与配套感知配置，并产出 manifest，
    供阶段3 D 评估按冻结协议（eval_protocol_v1）消费。仅依赖 PyYAML；jsonschema 可选，用于
    按 eval_manifest.schema.json 校验（不可用时走内置等价校验）。

场景族（族名与协议 sample_size_plan.per_family_independent_seeds 一致）：
    nominal            —— 静态或匀速/多航点(转向/机动)目标 + 少量静态障碍，期望可达可跟踪(TRACKING_SUCCESS)。
    obstacle_layout    —— 多种命名障碍布局；布局集合按 split 隔离（不同 split 不相交）。
    unreachable_closed —— 故意不可达：复用 N3 scenario_truly_blocked 语义（四道墙围住目标）→ PLAN_UNREACHABLE。
    boundary           —— 边界组：目标贴近障碍保护距离边界 / 速度贴近车上限 / 起点净空贴近下限。
    perception_fault   —— 较高强度随机化感知参数（噪声/视场/丢包/延迟），期望多数可完成或按设计失败。

确定性来源：
    - 一切随机数由 master_seed 派生：entry_seed = split_base + (sha256("master|split|family|index") 低31位)；
      split_base = split_index * (协议 frequency_split.train_seed_base_spacing)，做命名空间隔离；断言无
      seed/scenario_id 跨 split 重复。
    - 浮点统一四舍五入到固定小数位；YAML/JSON 采用固定键序与缩进；generated_at 由 master_seed 派生
      （保证同一主种子逐字节可重建）。如需真实时间戳可 --generated-at 覆盖，但会破坏 --rebuild-check。

CLI 用法：
    python3 d1_scenario_gen.py --split test --master-seed 20261007 \
        --out experiments/d/manifest_test_v1.json
    python3 d1_scenario_gen.py --split train --families nominal,boundary --master-seed 1
    python3 d1_scenario_gen.py --split test --master-seed 20261007 --rebuild-check
    python3 d1_scenario_gen.py --ablation fixed_vs_randomized --master-seed 20261007

消融清单（协议 sample_size_plan.ablation_arms，--ablation <name>）：
    fixed_vs_randomized      —— fixed 臂：引用包内 scenario_flat/scenario_linear_target（各 4 种子，共 8 条），
                                world/sensors 用包内默认文件；randomized 臂复用 test 集 nominal 族。
    remove_tracking_mechanism —— paired 臂：取 test 集 nominal 前 8 条为 baseline，产出 8 条同场景/世界/种子、
                                仅 target_source=air_fusion 的配对条目，用 pair_id 关联 baseline。
    --scenarios-dir / --perception-dir / --worlds-dir 指定产物目录，
    默认 experiments/d/scenarios | experiments/d/perception | experiments/d/worlds。

world 生成：
    每个场景额外产出一份配套 Gazebo world（world_<scenario_id>.world），以
    src/air_ground_core/worlds/agc_world.world 为模板，仅把模板内 pole_1/pole_2 两个
    <model> 块替换为该场景 obstacles 生成的 <model name="..."> 块（static box，名称/位姿/尺寸
    与 scenario YAML 的 obstacles 逐字段一致），其余内容（sun/ground_plane/asphalt_plane/
    agc_car/intruder_uav/physics 等）保持不变，使物理世界与规划栅格几何一致。
"""

import argparse
import hashlib
import json
import math
import os
import random
import re
import shutil
import sys
import tempfile
import time
from collections import deque

import yaml

# ---------------------------------------------------------------- 路径与冻结输入
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
CONFIG_DIR = os.path.join(WORKSPACE, "src", "air_ground_core", "config")
PROTOCOL_PATH = os.path.join(CONFIG_DIR, "eval_protocol_v1.yaml")
SCHEMA_PATH = os.path.join(CONFIG_DIR, "eval_manifest.schema.json")
BASE_SENSORS_PATH = os.path.join(CONFIG_DIR, "perception_sensors.yaml")
WORLD_TEMPLATE_PATH = os.path.join(WORKSPACE, "src", "air_ground_core", "worlds", "agc_world.world")

CREATED_FROM = "src/air_ground_core/scripts/d1_scenario_gen.py@stage3-d1"
MANIFEST_VERSION = "d1-manifest-v1"
PROTOCOL_VERSION = "eval_protocol_v1"

SPLIT_ORDER = ["train", "validation", "test"]
FAMILY_ORDER = ["nominal", "obstacle_layout", "unreachable_closed", "boundary", "perception_fault"]
# train/validation 为调参用小集（协议未规定，按 test 基数的固定比例取整）
TRAIN_RATIO = 0.45
VALIDATION_RATIO = 0.35

# ---------------------------------------------------------------- 契约常量（对齐 agc_params.yaml / 现有场景）
MAP = {"resolution_m": 0.5, "size_x_m": 40.0, "size_y_m": 40.0,
       "origin_x": -20.0, "origin_y": -20.0, "unknown_is_obstacle": True}
THRESHOLDS = {"approach_radius_m": 3.0, "hold_window_s": 5.0, "confidence_min": 0.5,
              "search_timeout_s": 60.0, "approach_timeout_s": 60.0,
              "mission_timeout_s": 120.0, "target_lost_tolerance_s": 2.0}
RES = 0.5                     # 地图/规划栅格分辨率
MAP_OX, MAP_OY = -20.0, -20.0
MAP_N = int(round(40.0 / RES))   # 80
CAR_RADIUS = 0.30
EMERGENCY_M = 0.35
PROTECTION_M = 0.80
SAFE_M = 2.4                  # spawn 净空下限
QUANT_ERR = RES / math.sqrt(2.0)          # ≈0.354 地图量化误差
TARGET_MIN_CLEAR = PROTECTION_M + QUANT_ERR  # ≈1.154 目标地面投影最小净空
CAR_MAX_SPEED = 1.0           # commander.max_linear_speed_mps
GOAL_ADJUST_M = 1.0           # planner.goal_adjust_max_m（N3 目标格可行化）
TARGET_HEIGHT_M = 2.0
CAR_Z = 0.11
DRONE_START = [0.0, 0.0, 0.0]
MAX_ATTEMPTS = 80             # 单场景重采样上限

# 传感器通道模板（id/朝向/量程下限保持固定，其余参数随机化）
SENSOR_TEMPLATES = [
    {"id": "air_eo_front", "boresight_body": [1.0, 0.0, 0.0], "range_min_m": 0.30},
    {"id": "air_ir_top", "boresight_body": [1.0, 0.0, -0.10], "range_min_m": 0.30},
]
# 感知参数随机区间：(sigma_xy, sigma_z, latency, drop, hfov, vfov, range_max)
SENSOR_RANGE_NORMAL = ((0.05, 0.35), (0.06, 0.40), (0.00, 0.15), (0.00, 0.15),
                       (60.0, 150.0), (45.0, 100.0), (9.0, 20.0))
SENSOR_RANGE_FAULT = ((0.40, 0.90), (0.50, 1.10), (0.10, 0.35), (0.15, 0.45),
                      (35.0, 70.0), (30.0, 55.0), (6.0, 9.0))
# v2 标定版：σ 上界使 conf=1-exp(-0.25²/(2σ²)) 有 ≥0.5 的可能（有信息量），
# 仍保留扰动（成功可能但不保证）；latency 上界 0.25 < fusion obs_max_age_s 0.35。
SENSOR_RANGE_FAULT_V2 = ((0.10, 0.30), (0.12, 0.35), (0.00, 0.25), (0.00, 0.35),
                         (55.0, 100.0), (40.0, 70.0), (7.0, 12.0))

# ---------------------------------------------------------------- A0 观测模型随机化区间
# 工程取值（非任何真实传感器标定）：nominal 族温和、perception_fault 族更强，覆盖通道内/间
#   相关、时间相关、确定性偏置、成簇丢失、虚警。仅在 --a0-model 打开时使用；关闭时这些字段
#   完全不参与随机序列，保证默认输出与现状逐字节一致。
A0_NOMINAL = {
    "corr_xy": (-0.3, 0.3), "corr_rho": (0.1, 0.4), "noise_tau_s": (0.0, 0.3),
    "bias_sigma": 0.05, "drop_burst_prob": (0.0, 0.5),
    "drop_burst_mean_s": (0.3, 0.8), "drop_burst_mean_gap_s": (3.0, 8.0),
    "false_alarm_rate_hz": (0.0, 0.3),
}
A0_FAULT = {
    "corr_xy": (-0.6, 0.6), "corr_rho": (0.2, 0.7), "noise_tau_s": (0.0, 0.6),
    "bias_sigma": 0.15, "drop_burst_prob": (0.0, 0.5),
    "drop_burst_mean_s": (0.3, 0.8), "drop_burst_mean_gap_s": (3.0, 8.0),
    "false_alarm_rate_hz": (0.0, 0.6),
}
A0_CORR_GROUP = "a0_common"   # 两通道同组：构造通道间公共模态相关
A0_DROP_GROUP = "a0_burst"    # 两通道同组：构造共同成簇丢失

# ---------------------------------------------------------------- 障碍布局目录（按 split 隔离，互不相交）
def _pole(dx, dy):
    return (dx, dy, 0.3, 0.3)


LAYOUT_PATTERNS = {
    "corridor_poles": [_pole(0.0, 0.0), _pole(0.0, 1.2), _pole(0.0, -1.2), _pole(0.0, 2.4)],
    "staggered_gates": [_pole(-1.2, 0.0), _pole(1.2, 1.4), _pole(-1.2, 2.8), _pole(1.2, 4.2)],
    "l_shape": [(-1.0, -1.0, 2.0, 0.3), (1.0, -0.2, 0.3, 1.6), (-1.0, -2.0, 0.3, 1.0)],
    "u_pocket": [(-1.0, 0.0, 0.3, 2.0), (1.0, 0.0, 0.3, 2.0), (0.0, -1.0, 2.0, 0.3)],
    "diagonal_wall": [_pole(0.0, 0.0), _pole(1.0, 1.0), _pole(2.0, 2.0)],
    "twin_boxes": [(-1.0, 0.0, 1.2, 1.2), (1.5, 1.5, 1.2, 1.2)],
    "zigzag_3": [_pole(0.0, 0.0), _pole(1.2, 1.0), _pole(0.6, 2.2)],
    "wide_field": [_pole(0.0, 0.0), _pole(2.0, 1.0), _pole(-2.0, 1.0), _pole(1.0, -2.0), _pole(-1.0, -2.0)],
    "sparse_pair": [_pole(0.0, 0.0), _pole(1.5, 1.5)],
}
# split -> 布局名集合（三者互不相交）
LAYOUT_SPLIT = {
    "test": ["corridor_poles", "staggered_gates", "l_shape", "u_pocket", "diagonal_wall"],
    "validation": ["twin_boxes", "zigzag_3"],
    "train": ["wide_field", "sparse_pair"],
}


# ---------------------------------------------------------------- 基础工具
class RejectError(Exception):
    """场景不满足可行性约束，触发重采样。"""


def sha256_hex(data):
    if isinstance(data, str):
        data = data.encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def r3(v):
    """定点四舍五入，保证浮点序列化确定性。"""
    return round(float(v), 3)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist_point_segment(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    L2 = vx * vx + vy * vy
    if L2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def point_obb_clearance(px, py, ob):
    """点(x,y)到障碍 OBB 表面的有符号距离（内部为负）；与 agc_common.cpp 一致（2D 足迹）。"""
    dx, dy = px - ob["x"], py - ob["y"]
    c, s = math.cos(-ob["yaw"]), math.sin(-ob["yaw"])
    lx = c * dx - s * dy
    ly = s * dx + c * dy
    ex = abs(lx) - ob["hx"]
    ey = abs(ly) - ob["hy"]
    if ex > 0.0 and ey > 0.0:
        return math.hypot(ex, ey)
    return max(ex, ey)


def min_clearance(px, py, obstacles):
    """点到全部障碍的最小净空。"""
    return min((point_obb_clearance(px, py, o) for o in obstacles), default=float("inf"))


def encode_manifest(obj):
    """manifest 文本编码：固定缩进与键序、末尾换行，保证逐字节可重建。"""
    return (json.dumps(obj, ensure_ascii=False, indent=2, sort_keys=False) + "\n").encode("utf-8")


# ---------------------------------------------------------------- 栅格可达性（近似 planner 膨胀栅格）
class Reachability:
    """按保护距离膨胀的栅格 BFS，近似 planner 膨胀栅格；用于“非故意不可达场景必须可达”校验。"""

    def __init__(self, obstacles):
        self.n = MAP_N
        self.blocked = bytearray(MAP_N * MAP_N)
        for j in range(MAP_N):
            cy = MAP_OY + (j + 0.5) * RES
            for i in range(MAP_N):
                cx = MAP_OX + (i + 0.5) * RES
                if min_clearance(cx, cy, obstacles) <= PROTECTION_M:
                    self.blocked[j * MAP_N + i] = 1

    def _cell(self, x, y):
        return int((x - MAP_OX) / RES), int((y - MAP_OY) / RES)

    def reachable_set(self, start_xy):
        n = self.n
        si, sj = self._cell(*start_xy)
        if not (0 <= si < n and 0 <= sj < n) or self.blocked[sj * n + si]:
            return None
        seen = bytearray(n * n)
        seen[sj * n + si] = 1
        q = deque([(si, sj)])
        while q:
            i, j = q.popleft()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if 0 <= ni < n and 0 <= nj < n:
                    idx = nj * n + ni
                    if not self.blocked[idx] and not seen[idx]:
                        seen[idx] = 1
                        q.append((ni, nj))
        return seen

    def any_goal_reachable(self, car_xy, goal_xy, goal_radius=GOAL_ADJUST_M):
        """车起点可达的栅格中，是否存在距目标 ≤ goal_adjust 的可行格（含 N3 目标格调整）。"""
        seen = self.reachable_set(car_xy)
        if seen is None:
            return False
        n = self.n
        gi, gj = self._cell(*goal_xy)
        rad = int(goal_radius / RES) + 1
        for j in range(max(0, gj - rad), min(n, gj + rad + 1)):
            for i in range(max(0, gi - rad), min(n, gi + rad + 1)):
                if seen[j * n + i]:
                    cx = MAP_OX + (i + 0.5) * RES
                    cy = MAP_OY + (j + 0.5) * RES
                    if math.hypot(cx - goal_xy[0], cy - goal_xy[1]) <= goal_radius:
                        return True
        return False


# ---------------------------------------------------------------- 障碍字典
def make_obstacle(name, x, y, sx, sy, sz=3.0, yaw=0.0):
    return {"name": name, "x": r3(x), "y": r3(y), "sx": r3(sx), "sy": r3(sy),
            "sz": r3(sz), "yaw": r3(yaw), "hx": sx / 2.0, "hy": sy / 2.0}


def obstacle_to_yaml(ob):
    return {"name": ob["name"],
            "pose": [ob["x"], ob["y"], r3(ob["sz"] / 2.0), 0.0, 0.0, ob["yaw"]],
            "size": [ob["sx"], ob["sy"], ob["sz"]]}


# ---------------------------------------------------------------- 随机采样原语
def sample_car(rng):
    """车起始位姿：位于无人机 spawn 与目标区的对侧。"""
    x = rng.uniform(-9.0, -1.5)
    y = rng.uniform(-9.0, -1.5)
    yaw = rng.uniform(0.0, 2.0 * math.pi)
    return [r3(x), r3(y), CAR_Z, r3(yaw)]


def sample_target(rng, obstacles=None, car=None, min_clear=0.0, tries=200):
    """目标初始位置：采样于地图另一侧，可附加净空/距离约束。"""
    for _ in range(tries):
        x = rng.uniform(1.5, 9.0)
        y = rng.uniform(-9.0, 9.0)
        if car is not None and dist((x, y), car[:2]) < 5.0:
            continue
        if obstacles is not None and min_clearance(x, y, obstacles) < min_clear:
            continue
        return [r3(x), r3(y), TARGET_HEIGHT_M, 0.0]
    raise RejectError("target placement failed")


def sample_motion(rng, target, mode=None, speed_range=(0.25, 0.7), max_wp=3, bounds=9.0,
                  allow_maneuver=False):
    """目标运动：static / linear（1~3 个额外航点）或（仅 allow_maneuver）sinusoid/circle。

    返回 (mode, speed, wps, man)；man 为机动扩展字典（默认 {}），仅 sinusoid/circle 非空。
    allow_maneuver 默认关：关闭时随机序列与返回值与现状逐字节一致（默认输出不变）。
    """
    tx, ty = target[0], target[1]
    if mode is None:
        if allow_maneuver and rng.random() < 0.4:
            mode = "sinusoid" if rng.random() < 0.5 else "circle"
        else:
            mode = "linear" if rng.random() < 0.6 else "static"
    if mode == "static":
        stub = [r3(tx + rng.uniform(0.3, 0.8)), r3(ty)]
        return "static", 0.0, [stub], {}
    if mode == "sinusoid":
        # 合速度 = sqrt(speed² + (amplitude·ω)²) < 0.9（车上限）；横向速度上界取 0.6
        speed = r3(rng.uniform(0.25, 0.55))
        period = r3(rng.uniform(3.0, 6.0))
        w = 2.0 * math.pi / period
        amp_max = min(0.6, 0.6 / w)
        amp = r3(rng.uniform(min(0.2, amp_max), amp_max))
        man = {"heading_deg": r3(rng.uniform(0.0, 360.0)), "amplitude_m": amp, "period_s": period}
        return "sinusoid", speed, [], man
    if mode == "circle":
        # 圆周运动合速度即 speed，直接低于车上限
        speed = r3(rng.uniform(0.3, 0.7))
        man = {"radius_m": r3(rng.uniform(1.0, 2.5))}
        return "circle", speed, [], man
    speed = r3(rng.uniform(*speed_range))
    k = rng.randint(1, max_wp)
    heading = rng.uniform(0.0, 2.0 * math.pi)
    cx, cy = tx, ty
    wps = []
    for _ in range(k):
        heading += rng.uniform(-1.2, 1.2)           # 转向/机动
        step = rng.uniform(1.0, 3.0)
        cx = min(bounds, max(-bounds, cx + step * math.cos(heading)))
        cy = min(bounds, max(-bounds, cy + step * math.sin(heading)))
        wps.append([r3(cx), r3(cy)])
    return "linear", speed, wps, {}


def place_obstacles_avoiding(rng, count, car, target_pts, polyline, size_choices=None):
    """在避开“车起点/目标航点/链路折线”的位置撒障碍，降低重采样率。"""
    obstacles = []
    choices = size_choices or [(0.3, 0.3), (0.3, 0.3), (0.6, 0.6), (1.0, 1.0)]
    attempts = 0
    while len(obstacles) < count and attempts < 300:
        attempts += 1
        sx, sy = choices[rng.randrange(len(choices))]
        x = rng.uniform(-9.0, 9.0)
        y = rng.uniform(-9.0, 9.0)
        half = math.hypot(sx, sy) / 2.0
        if math.hypot(x, y) < PROTECTION_M + half:                 # 无人机 spawn 净空
            continue
        if dist((x, y), car[:2]) < SAFE_M + half:                  # 车起点净空
            continue
        if any(dist((x, y), p) < TARGET_MIN_CLEAR + half for p in target_pts):
            continue
        if any(dist_point_segment(x, y, polyline[i][0], polyline[i][1],
                                  polyline[i + 1][0], polyline[i + 1][1]) < 1.6 + half
               for i in range(len(polyline) - 1)):
            continue
        obstacles.append(make_obstacle("obs_%d" % (len(obstacles) + 1), x, y, sx, sy))
    return obstacles


def make_layout_obstacles(rng, layout):
    """按命名布局在随机锚点生成障碍。"""
    ax = rng.uniform(-5.0, 5.0)
    ay = rng.uniform(-5.0, 5.0)
    obs = []
    for i, (dx, dy, sx, sy) in enumerate(LAYOUT_PATTERNS[layout]):
        obs.append(make_obstacle("%s_%d" % (layout, i + 1), ax + dx, ay + dy, sx, sy))
    return obs, [r3(ax), r3(ay)]


# ---------------------------------------------------------------- 感知配置
def make_sensors(rng, fault=False, ranges=None, a0=False, a0_seed=None, occlusion=None,
                 a0_override=None):
    """随机化传感器参数；fault=True 取较高强度区间，ranges 可显式覆盖。

    a0=True 时在基底字段之后追加 A0 观测模型字段（相关/偏置/成簇丢失/虚警），其随机数取自
    独立的 a0_seed 派生流，不触碰基底字段的抽样序列；关闭时输出与现状逐字节一致。
    occlusion=(enabled, pad_m) 仅在 a0=True 时写入顶层 perception_model.occlusion。
    a0_override 给定通道级 A0 字段时以显式值覆盖（用于保证特征非零，如重放复测场景）。
    """
    (sxy, sz, lat, drop, hfov, vfov, rmax) = ranges or (SENSOR_RANGE_FAULT if fault else SENSOR_RANGE_NORMAL)
    sensors = []
    for t in SENSOR_TEMPLATES:
        sensors.append({
            "id": t["id"],
            "sigma_xy_m": r3(rng.uniform(*sxy)),
            "sigma_z_m": r3(rng.uniform(*sz)),
            "latency_s": r3(rng.uniform(*lat)),
            "drop_prob": r3(rng.uniform(*drop)),
            "boresight_body": list(t["boresight_body"]),
            "hfov_deg": round(rng.uniform(*hfov), 1),
            "vfov_deg": round(rng.uniform(*vfov), 1),
            "range_min_m": t["range_min_m"],
            "range_max_m": round(rng.uniform(*rmax), 1),
        })
    if not a0:
        return {"sensors": sensors}
    # ---- A0 字段（独立流，按 split/family 派生 seed），base 字段已在上面抽样完毕 ----
    rng_a0 = random.Random(a0_seed)
    rr = A0_FAULT if fault else A0_NOMINAL
    for s in sensors:
        s["corr_xy"] = r3(rng_a0.uniform(*rr["corr_xy"]))
        s["corr_group"] = A0_CORR_GROUP
        s["corr_rho"] = r3(rng_a0.uniform(*rr["corr_rho"]))
        s["noise_tau_s"] = r3(rng_a0.uniform(*rr["noise_tau_s"]))
        s["bias_m"] = [r3(rng_a0.gauss(0.0, rr["bias_sigma"])) for _ in range(3)]
        s["drop_burst_prob"] = r3(rng_a0.uniform(*rr["drop_burst_prob"]))
        s["drop_burst_mean_s"] = r3(rng_a0.uniform(*rr["drop_burst_mean_s"]))
        s["drop_burst_mean_gap_s"] = r3(rng_a0.uniform(*rr["drop_burst_mean_gap_s"]))
        s["drop_group"] = A0_DROP_GROUP
        s["false_alarm_rate_hz"] = r3(rng_a0.uniform(*rr["false_alarm_rate_hz"]))
        if a0_override:  # 显式覆盖（保证相关/偏置/成簇/虚警特征非零）
            for k, v in a0_override.items():
                s[k] = list(v) if isinstance(v, list) else v
    occ_en, occ_pad = occlusion if occlusion else (False, 0.0)
    # A0 模式一律开启确定性重播种：每拍噪声按 (seed,通道,bucket) 派生，抽样成为 bucket 的纯函数
    return {"sensors": sensors,
            "perception_model": {"occlusion": {"enabled": bool(occ_en), "pad_m": r3(occ_pad)},
                                 "deterministic_reseed": True}}


# ---------------------------------------------------------------- 场景装配
def assemble(scenario_id, note, car, target, mode, speed, wps, obstacles, man=None):
    """装配场景 YAML；man 非空时在 target_motion 追加机动扩展键（仅 A0 机动用）。"""
    tm = {"mode": mode, "speed_mps": speed, "waypoints": wps}
    if man:
        tm.update(man)   # 追加 heading_deg/amplitude_m/period_s 或 radius_m
    return {"scenario_id": scenario_id, "locked": True, "note": note,
            "car_start": car, "target_start": target, "drone_start": list(DRONE_START),
            "target_motion": tm,
            "obstacles": [obstacle_to_yaml(o) for o in obstacles],
            "map": dict(MAP), "thresholds": dict(THRESHOLDS)}


def _check_common(obstacles, car, target, wps):
    """公共可行性：车起点/无人机 spawn/目标及航点净空。返回 (car_clr, target_min_clr, tgt_pts)。"""
    car_clr = min_clearance(car[0], car[1], obstacles)
    drone_clr = min_clearance(0.0, 0.0, obstacles)
    if car_clr < SAFE_M:
        raise RejectError("car clearance %.3f < safe %.2f" % (car_clr, SAFE_M))
    if drone_clr < PROTECTION_M:
        raise RejectError("drone spawn clearance %.3f < protection" % drone_clr)
    tgt_pts = [(target[0], target[1])] + [(w[0], w[1]) for w in wps]
    tgt_min = min(min_clearance(p[0], p[1], obstacles) for p in tgt_pts)
    return car_clr, tgt_min, tgt_pts


def _require_reachable(car, tgt_pts, obstacles):
    """非故意不可达场景：目标与全部航点必须可达。"""
    reach = Reachability(obstacles)
    for p in tgt_pts:
        if not reach.any_goal_reachable((car[0], car[1]), p):
            raise RejectError("goal %s unreachable" % (p,))


def _require_unreachable(car, target, obstacles):
    """故意不可达场景：确认目标附近确实不可达（防止四墙未真正封闭）。"""
    reach = Reachability(obstacles)
    if reach.any_goal_reachable((car[0], car[1]), (target[0], target[1])):
        raise RejectError("intended-unreachable scenario is actually reachable")


# ---------------------------------------------------------------- 各族场景构建
def build_nominal(rng, fault=False, allow_maneuver=False):
    """nominal / perception_fault 共用：可达可跟踪场景 + 少量障碍。"""
    car = sample_car(rng)
    target = sample_target(rng, car=car)
    mode, speed, wps, man = sample_motion(rng, target, max_wp=3, allow_maneuver=allow_maneuver)
    polyline = [(car[0], car[1]), (target[0], target[1])] + [(w[0], w[1]) for w in wps]
    obstacles = place_obstacles_avoiding(rng, rng.randint(2, 4), car, polyline[1:], polyline)
    note = ("D1 随机化 %s：期望可达可跟踪(TRACKING_SUCCESS)；目标模式=%s"
            % ("perception_fault" if fault else "nominal", mode))
    return car, target, mode, speed, wps, obstacles, note, man


def build_layout(rng, layout, allow_maneuver=False):
    """obstacle_layout：命名布局障碍 + 可达目标。"""
    obstacles, anchor = make_layout_obstacles(rng, layout)
    car = None
    for _ in range(200):
        c = sample_car(rng)
        if min_clearance(c[0], c[1], obstacles) >= SAFE_M:
            car = c
            break
    if car is None:
        raise RejectError("no car start with safe clearance in layout")
    target = sample_target(rng, obstacles=obstacles, car=car, min_clear=TARGET_MIN_CLEAR + 0.2)
    mode, speed, wps, man = sample_motion(rng, target, max_wp=3, allow_maneuver=allow_maneuver)
    note = "D1 随机化 obstacle_layout：布局=%s；期望可达可跟踪(TRACKING_SUCCESS)" % layout
    return car, target, mode, speed, wps, obstacles, note, anchor, man


def build_unreachable(rng):
    """unreachable_closed：复用 N3 语义，四道墙围住目标 -> PLAN_UNREACHABLE。"""
    car = sample_car(rng)
    cx = cy = None
    for _ in range(200):
        cx = rng.uniform(-2.0, 6.0)
        cy = rng.uniform(-5.0, 7.0)
        if dist((cx, cy), car[:2]) < 5.0 or math.hypot(cx, cy) < 3.5:
            continue
        break
    if cx is None:
        raise RejectError("no pocket center")
    obstacles = [
        make_obstacle("box_top", cx, cy + 1.3, 3.0, 0.3),
        make_obstacle("box_bottom", cx, cy - 1.3, 3.0, 0.3),
        make_obstacle("box_left", cx - 1.5, cy, 0.3, 3.0),
        make_obstacle("box_right", cx + 1.5, cy, 0.3, 3.0),
        make_obstacle("pole_1", cx - 2.6, cy - 2.2, 0.3, 0.3),
        make_obstacle("pole_2", cx + 2.2, cy - 2.6, 0.3, 0.3),
    ]
    target = [r3(cx), r3(cy), TARGET_HEIGHT_M, 0.0]
    wps = [[r3(cx), r3(cy)]]
    note = "D1 随机化 unreachable_closed：四道墙围住目标(N3 语义) -> 期望 PLAN_UNREACHABLE"
    return car, target, "static", 0.0, wps, obstacles, note, {}


def build_boundary(rng, allow_maneuver=False):
    """boundary：目标贴障碍保护边界 / 速度贴上限 / 起点净空贴下限。"""
    subtype = rng.choice(["target_protection_edge", "speed_upper_edge", "start_clearance_edge"])
    car = sample_car(rng)
    target = sample_target(rng, car=car)
    obstacles = []
    if subtype == "speed_upper_edge":
        mode, speed, wps, man = sample_motion(rng, target, mode="linear", speed_range=(0.85, 0.89), max_wp=3)
    else:
        mode, speed, wps, man = sample_motion(rng, target, speed_range=(0.25, 0.7), max_wp=2,
                                              allow_maneuver=allow_maneuver)
    polyline = [(car[0], car[1]), (target[0], target[1])] + [(w[0], w[1]) for w in wps]
    if subtype == "target_protection_edge":
        # 目标中心到障碍表面净空 ≈ 保护距离下限 + 量化误差 + 小量
        want = TARGET_MIN_CLEAR + rng.uniform(0.0, 0.12)
        ang = rng.uniform(0.0, 2.0 * math.pi)
        d = want + 0.15
        obstacles.append(make_obstacle("edge_pole", target[0] + d * math.cos(ang),
                                       target[1] + d * math.sin(ang), 0.3, 0.3))
    elif subtype == "start_clearance_edge":
        # 车起点净空 ≈ 安全下限
        want = SAFE_M + rng.uniform(0.0, 0.15)
        ang = rng.uniform(0.0, 2.0 * math.pi)
        d = want + 0.15
        obstacles.append(make_obstacle("start_pole", car[0] + d * math.cos(ang),
                                       car[1] + d * math.sin(ang), 0.3, 0.3))
    obstacles += place_obstacles_avoiding(rng, rng.randint(1, 2), car, polyline[1:], polyline)
    note = ("D1 随机化 boundary：边界子类=%s；净空偏序 emergency<protection<safe 保持" % subtype)
    return car, target, mode, speed, wps, obstacles, note, subtype, man


# ---------------------------------------------------------------- 单场景生成（含可行性校验）
def make_scenario(family, rng, layout, scenario_id, seed=None, a0_model=False):
    """生成一个满足约束的场景；返回 (scenario_dict, sensors_dict, params, layout_name)。

    a0_model=True 时启用 A0 观测模型随机化与可选机动（seed 用于派生 A0 字段随机流）；
    默认 False 时输出与现状逐字节一致。
    """
    subtype = None
    anchor = None
    if family == "nominal":
        car, target, mode, speed, wps, obstacles, note, man = build_nominal(rng, allow_maneuver=a0_model)
        layout_name = None
    elif family == "perception_fault":
        car, target, mode, speed, wps, obstacles, note, man = build_nominal(rng, fault=True, allow_maneuver=a0_model)
        layout_name = None
    elif family == "obstacle_layout":
        car, target, mode, speed, wps, obstacles, note, anchor, man = build_layout(rng, layout, allow_maneuver=a0_model)
        layout_name = layout
    elif family == "unreachable_closed":
        car, target, mode, speed, wps, obstacles, note, man = build_unreachable(rng)
        layout_name = None
    elif family == "boundary":
        car, target, mode, speed, wps, obstacles, note, subtype, man = build_boundary(rng, allow_maneuver=a0_model)
        layout_name = None
    else:
        raise RejectError("unknown family %s" % family)

    scenario = assemble(scenario_id, note, car, target, mode, speed, wps, obstacles, man)
    car_clr, tgt_min, tgt_pts = _check_common(obstacles, car, target, wps)

    if family == "unreachable_closed":
        _require_unreachable(car, target, obstacles)
    else:
        if tgt_min < TARGET_MIN_CLEAR:
            raise RejectError("target clearance %.3f < %.3f" % (tgt_min, TARGET_MIN_CLEAR))
        _require_reachable(car, tgt_pts, obstacles)

    # 目标速度必须低于车上限（linear 时）
    if mode == "linear" and not (0.2 < speed < CAR_MAX_SPEED):
        raise RejectError("target speed %.3f out of range" % speed)

    # A0 字段随机流种子：按 split(内嵌 seed)/family 派生（opt-in）
    a0_seed = hash31("a0|%d|%s" % (seed, family)) if (a0_model and seed is not None) else None
    sensors = make_sensors(rng, fault=(family == "perception_fault"),
                           a0=a0_model, a0_seed=a0_seed)
    params = {
        "target_motion_mode": mode,
        "target_speed_mps": speed,
        "num_obstacles": len(obstacles),
        "intended_unreachable": family == "unreachable_closed",
        "boundary": family == "boundary",
        # 扩展字段（schema 对 params 未设 additionalProperties:false，允许）
        "target_start": [target[0], target[1]],
        "car_start": [car[0], car[1]],
        "waypoints_number": len(wps),
        "boundary_subtype": subtype if family == "boundary" else None,
        "min_target_clearance_m": r3(tgt_min),
        "car_start_clearance_m": r3(car_clr),
        "layout_anchor": anchor if family == "obstacle_layout" else None,
        "perception": {
            "sigma_xy_m": [s["sigma_xy_m"] for s in sensors["sensors"]],
            "sigma_z_m": [s["sigma_z_m"] for s in sensors["sensors"]],
            "latency_s": [s["latency_s"] for s in sensors["sensors"]],
            "drop_prob": [s["drop_prob"] for s in sensors["sensors"]],
            "hfov_deg": [s["hfov_deg"] for s in sensors["sensors"]],
            "vfov_deg": [s["vfov_deg"] for s in sensors["sensors"]],
            "range_max_m": [s["range_max_m"] for s in sensors["sensors"]],
        },
    }
    return scenario, sensors, params, layout_name


# ---------------------------------------------------------------- YAML 输出（固定风格/键序）
class _Dumper(yaml.SafeDumper):
    pass


def _is_scalar(v):
    return v is None or isinstance(v, (str, int, float, bool))


def _is_scalar_seq(v):
    return isinstance(v, list) and len(v) > 0 and all(_is_scalar(x) for x in v)


def _represent_seq(dumper, data):
    flow = bool(data) and all(_is_scalar(x) or _is_scalar_seq(x) for x in data)
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow)


def _represent_map(dumper, data):
    flow = all(_is_scalar(v) or _is_scalar_seq(v) for v in data.values())
    return dumper.represent_mapping("tag:yaml.org,2002:map", data, flow_style=flow)


_Dumper.add_representer(list, _represent_seq)
_Dumper.add_representer(dict, _represent_map)


def dump_yaml(obj):
    return yaml.dump(obj, Dumper=_Dumper, sort_keys=False, allow_unicode=True,
                     default_flow_style=False).encode("utf-8")


# ---------------------------------------------------------------- world 生成
# 匹配模板中的 pole_* 注释与 <model> 块（含注释一起删除）
_WORLD_POLE_RE = re.compile(r'\n\s*<!-- 静态障碍 pole_\d+.*?-->\n\s*<model name="pole_\d+">.*?</model>', re.S)


def load_world_template():
    with open(WORLD_TEMPLATE_PATH, "r", encoding="utf-8") as f:
        return f.read()


def _fmt_num(v):
    """世界文件数字格式：整数不带小数点（3 而非 3.0）。"""
    return "%g" % float(v)


def render_world(scenario, template):
    """按 scenario obstacles 生成 world 文本：替换模板 pole_* 块，其余内容保持不变。"""
    blocks = []
    for ob in scenario["obstacles"]:
        pose = ob["pose"]
        sx, sy, sz = ob["size"]
        size = "%s %s %s" % (_fmt_num(sx), _fmt_num(sy), _fmt_num(sz))
        blocks.append(
            '    <!-- 静态障碍 %s：与 scenario %s 的 obstacles 逐字段一致 -->\n'
            '    <model name="%s">\n'
            '      <static>true</static>\n'
            '      <pose>%s %s %s 0 0 %s</pose>\n'
            '      <link name="link">\n'
            '        <collision name="collision">\n'
            '          <geometry><box><size>%s</size></box></geometry>\n'
            '        </collision>\n'
            '        <visual name="visual">\n'
            '          <geometry><box><size>%s</size></box></geometry>\n'
            '          <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>\n'
            '        </visual>\n'
            '      </link>\n'
            '    </model>' % (ob["name"], scenario["scenario_id"], ob["name"],
                             _fmt_num(pose[0]), _fmt_num(pose[1]), _fmt_num(pose[2]),
                             _fmt_num(pose[5] if len(pose) > 5 else 0.0), size, size))
    body = _WORLD_POLE_RE.sub("", template)
    idx = body.rfind("  </world>")
    return (body[:idx] + "\n".join(blocks) + "\n" + body[idx:]).encode("utf-8")


# ---------------------------------------------------------------- 协议与种子
def load_protocol():
    with open(PROTOCOL_PATH) as f:
        return yaml.safe_load(f)


def planned_count(split, family, proto):
    """按协议样本量规划计算条目数：base(test 协议值 / train|validation 比例) × over_provision(向下取整)。"""
    base = int(proto["sample_size_plan"]["per_family_independent_seeds"][family])
    if split == "train":
        base = max(1, int(base * TRAIN_RATIO))
    elif split == "validation":
        base = max(1, int(base * VALIDATION_RATIO))
    ratio = float(proto["sample_size_plan"].get("over_provision_ratio", 1.0))
    return max(1, int(base * ratio))


def hash31(text):
    return int(hashlib.sha256(text.encode("utf-8")).hexdigest(), 16) & 0x7FFFFFFF


def entry_seed(master_seed, split, family, index, spacing, used):
    """派生 entry 种子：split 命名空间基址 + sha256 低31位；保证全局唯一。"""
    split_base = SPLIT_ORDER.index(split) * int(spacing)
    salt = 0
    while True:
        key = "%d|%s|%s|%d" % (master_seed, split, family, index)
        if salt:
            key += "|%d" % salt
        seed = split_base + hash31(key)
        if seed not in used:
            used.add(seed)
            return seed
        salt += 1


def attempt_rng(seed, attempt):
    return random.Random(hash31("%d|attempt|%d" % (seed, attempt)))


def derived_generated_at(master_seed):
    """由主种子派生确定性时间戳，保证同一主种子逐字节可重建。"""
    base = 1767225600  # 2026-01-01T00:00:00Z
    off = hash31("ts|%d" % master_seed) % (365 * 86400)
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(base + off))


def config_hash():
    """冻结输入的组合哈希（协议 + manifest schema + 感知模板）。"""
    parts = []
    for rel in ["src/air_ground_core/config/eval_protocol_v1.yaml",
                "src/air_ground_core/config/eval_manifest.schema.json",
                "src/air_ground_core/config/perception_sensors.yaml"]:
        parts.append("%s=%s" % (rel, sha256_file(os.path.join(WORKSPACE, rel))))
    return sha256_hex("\n".join(parts))


# ---------------------------------------------------------------- manifest 构建
def build_manifest(master_seed, split, families, logical_scen_dir, logical_perc_dir,
                   logical_world_dir, generated_at=None, a0_model=False):
    """生成 manifest 对象与全部文件记录（不落盘）。a0_model=True 时启用 A0 观测模型随机化。"""
    proto = load_protocol()
    spacing = proto["sample_size_plan"].get("frequency_split", {}).get("train_seed_base_spacing", 1000)
    template = load_world_template()
    notes = []
    notes.append("协议=%s; 主种子=%d; split=%s; families=%s" %
                 (PROTOCOL_VERSION, master_seed, split, ",".join(families)))
    notes.append("确定性: entry_seed=split基址(spacing=%s)+sha256低31位; 浮点四舍五入; "
                 "generated_at 由主种子派生; 同一主种子逐字节可重建" % spacing)
    notes.append("world: 以 agc_world.world 为模板，pole_* 块替换为该场景 obstacles 生成的 "
                 "static box，名称/位姿/尺寸与 scenario obstacles 逐字段一致")
    if a0_model:
        notes.append("A0 观测模型(opt-in): 通道追加 corr_xy/corr_group/corr_rho/noise_tau_s/"
                     "bias_m/drop_burst_*/drop_group/false_alarm_rate_hz（工程取值，非标定）；"
                     "目标可选 sinusoid/circle 机动（合速度<0.9 m/s）")

    entries = []
    records = []
    used_seeds = set()
    used_ids = set()
    rejected = []

    for family in families:
        n = planned_count(split, family, proto)
        layout_pool = LAYOUT_SPLIT.get(split, []) if family == "obstacle_layout" else None
        if layout_pool:
            notes.append("布局分配: split=%s family=obstacle_layout -> %s（split 间布局集合不相交）"
                         % (split, ",".join(layout_pool)))
        made = 0
        for i in range(n):
            seed = entry_seed(master_seed, split, family, i, spacing, used_seeds)
            layout = layout_pool[i % len(layout_pool)] if layout_pool else None
            scenario_id = "%s_%s_%03d_%09d" % (split, family, i, seed)
            if scenario_id in used_ids:
                raise AssertionError("scenario_id 重复: %s" % scenario_id)
            used_ids.add(scenario_id)

            built = None
            for attempt in range(MAX_ATTEMPTS):
                rng = attempt_rng(seed, attempt)
                try:
                    built = make_scenario(family, rng, layout, scenario_id,
                                          seed=seed, a0_model=a0_model)
                    break
                except RejectError:
                    continue
            if built is None:
                rejected.append("%s/%s#%d: 重采样 %d 次仍不满足约束，已剔除" % (split, family, i, MAX_ATTEMPTS))
                continue

            scenario, sensors, params, layout_name = built
            scen_name = "scenario_%s.yaml" % scenario_id
            perc_name = "perception_%s.yaml" % scenario_id
            scen_bytes = dump_yaml(scenario)
            perc_bytes = dump_yaml(sensors)
            world_name = "world_%s.world" % scenario_id
            world_bytes = render_world(scenario, template)
            scen_rel = os.path.relpath(os.path.join(logical_scen_dir, scen_name), WORKSPACE)
            perc_rel = os.path.relpath(os.path.join(logical_perc_dir, perc_name), WORKSPACE)
            world_rel = os.path.relpath(os.path.join(logical_world_dir, world_name), WORKSPACE)

            # 感知随机化经逐场景 sensors_file 实现，运行期不声明注入（inject=null）
            inject = None

            entry = {
                "scenario_id": scenario_id,
                "split": split,
                "family": family,
                "seed": seed,
                "scenario_file": scen_rel,
                "scenario_hash": sha256_hex(scen_bytes),
                "sensors_file": perc_rel,
                "sensors_hash": sha256_hex(perc_bytes),
                "world_file": world_rel,
                "world_hash": sha256_hex(world_bytes),
                "layout": layout_name,
                "target_source": "air_fusion" if family == "perception_fault" else "gt_adapter",
                "obstacle_source": "scenario",
                "inject": inject,
                "params": params,
            }
            entries.append(entry)
            records.append({"scenario_id": scenario_id, "scenario": (scen_rel, scen_bytes),
                            "sensors": (perc_rel, perc_bytes),
                            "world": (world_rel, world_bytes)})
            made += 1

        notes.append("规模: split=%s family=%s planned=%d actual=%d (base×over_provision=%s)"
                     % (split, family, n, made, proto["sample_size_plan"].get("over_provision_ratio")))
    if rejected:
        notes.append("被拒场景(未进入 entries): %d 条" % len(rejected))
        notes.extend(rejected)
    else:
        notes.append("被拒场景: 0 条（全部通过可行性校验）")

    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "protocol": PROTOCOL_VERSION,
        "master_seed": int(master_seed),
        "created_from": CREATED_FROM,
        "generated_at": generated_at if generated_at else derived_generated_at(master_seed),
        "config_hash": config_hash(),
        "notes": notes,
        "entries": entries,
    }

    # 断言：seed / scenario_id 无重复（含跨 split）
    assert len({e["seed"] for e in entries}) == len(entries), "存在重复 seed"
    assert len({e["scenario_id"] for e in entries}) == len(entries), "存在重复 scenario_id"
    return manifest, records


# ---------------------------------------------------------------- schema 校验
def validate_manifest_schema(obj):
    """按 eval_manifest.schema.json 校验；jsonschema 不可用时走内置等价校验。"""
    schema = None
    if os.path.isfile(SCHEMA_PATH):
        with open(SCHEMA_PATH) as f:
            schema = json.load(f)
    try:
        import jsonschema  # 可选依赖
    except ImportError:
        return _validate_builtin(obj, schema), "builtin"
    if schema is None:
        return _validate_builtin(obj, None), "builtin"
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(validator.iter_errors(obj), key=lambda e: list(e.path))
    return ([str(e.message) + " @ " + "/".join(str(p) for p in e.path) for e in errors], "jsonschema")


def _validate_builtin(obj, schema):
    """内置等价校验（jsonschema 缺失时兜底）。"""
    import re
    errors = []
    allowed = {"manifest_version", "protocol", "master_seed", "created_from",
               "generated_at", "config_hash", "notes", "entries"}
    required = ["manifest_version", "protocol", "master_seed", "created_from", "generated_at", "entries"]
    for k in required:
        if k not in obj:
            errors.append("缺少顶层字段: %s" % k)
    for k in obj:
        if k not in allowed:
            errors.append("顶层多出字段: %s" % k)
    if not isinstance(obj.get("master_seed"), int):
        errors.append("master_seed 必须为整数")
    if not isinstance(obj.get("entries"), list):
        errors.append("entries 必须为数组")
        return errors
    ereq = ["scenario_id", "split", "family", "seed", "scenario_file", "scenario_hash",
            "sensors_file", "sensors_hash", "params"]
    eallowed = set(ereq) | {"layout", "target_source", "obstacle_source", "inject"}
    pat = re.compile(r"^[A-Za-z0-9_.-]+$")
    for i, e in enumerate(obj["entries"]):
        for k in ereq:
            if k not in e:
                errors.append("entry[%d] 缺少字段 %s" % (i, k))
        for k in e:
            if k not in eallowed:
                errors.append("entry[%d] 多出字段 %s" % (i, k))
        if not isinstance(e.get("scenario_id"), str) or not pat.match(e.get("scenario_id", "")):
            errors.append("entry[%d].scenario_id 不匹配 pattern" % i)
        if e.get("split") not in ("train", "validation", "test"):
            errors.append("entry[%d].split 非法" % i)
        if not isinstance(e.get("seed"), int):
            errors.append("entry[%d].seed 必须为整数" % i)
        if e.get("target_source") not in ("gt_adapter", "air_fusion"):
            errors.append("entry[%d].target_source 非法" % i)
        if e.get("obstacle_source") not in ("scenario", "air_fusion"):
            errors.append("entry[%d].obstacle_source 非法" % i)
        if not isinstance(e.get("params"), dict):
            errors.append("entry[%d].params 必须为对象" % i)
    return errors


# ---------------------------------------------------------------- 落盘
def write_outputs(manifest, records, abs_scen_dir, abs_perc_dir, abs_world_dir, out_path):
    os.makedirs(abs_scen_dir, exist_ok=True)
    os.makedirs(abs_perc_dir, exist_ok=True)
    os.makedirs(abs_world_dir, exist_ok=True)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    for rec in records:
        scen_rel, scen_bytes = rec["scenario"]
        perc_rel, perc_bytes = rec["sensors"]
        world_rel, world_bytes = rec["world"]
        with open(os.path.join(abs_scen_dir, os.path.basename(scen_rel)), "wb") as f:
            f.write(scen_bytes)
        with open(os.path.join(abs_perc_dir, os.path.basename(perc_rel)), "wb") as f:
            f.write(perc_bytes)
        with open(os.path.join(abs_world_dir, os.path.basename(world_rel)), "wb") as f:
            f.write(world_bytes)
    with open(out_path, "wb") as f:
        f.write(encode_manifest(manifest))


def default_out_path(split):
    return os.path.join(WORKSPACE, "experiments", "d", "manifest_%s_v1.json" % split)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="阶段3 D1 随机化场景生成器与 manifest 构建")
    ap.add_argument("--split", choices=SPLIT_ORDER, default="test")
    ap.add_argument("--families", default=",".join(FAMILY_ORDER))
    ap.add_argument("--master-seed", type=int, default=20261007)
    ap.add_argument("--out", default=None)
    ap.add_argument("--rebuild-check", action="store_true")
    ap.add_argument("--scenarios-dir", default="experiments/d/scenarios")
    ap.add_argument("--perception-dir", default="experiments/d/perception")
    ap.add_argument("--worlds-dir", default="experiments/d/worlds")
    ap.add_argument("--ablation", choices=ABLATION_CHOICES, default=None)
    ap.add_argument("--percfault-v2", action="store_true",
                    help="生成 perception_fault 标定版清单（复用 v1 场景/世界/种子，仅换感知文件）")
    ap.add_argument("--a0-model", action="store_true",
                    help="启用 A0 观测模型随机化（通道相关/偏置/成簇丢失/虚警 + 可选机动）；默认关")
    ap.add_argument("--a0-manifest", action="store_true",
                    help="生成 A0 专用小 manifest（4 条：nominal/occlusion/maneuver/fault）到 experiments/a0/")
    ap.add_argument("--generated-at", default=None)
    return ap.parse_args(argv)


def resolve_dir(p):
    return p if os.path.isabs(p) else os.path.join(WORKSPACE, p)


def parse_families(s):
    fams = [x.strip() for x in s.split(",") if x.strip()]
    for f in fams:
        if f not in FAMILY_ORDER:
            raise SystemExit("未知 family: %s（可选: %s）" % (f, ",".join(FAMILY_ORDER)))
    return [f for f in FAMILY_ORDER if f in fams]


# ---------------------------------------------------------------- 子命令
def cmd_generate(args):
    families = parse_families(args.families)
    abs_scen = resolve_dir(args.scenarios_dir)
    abs_perc = resolve_dir(args.perception_dir)
    abs_world = resolve_dir(args.worlds_dir)
    out_path = resolve_dir(args.out) if args.out else default_out_path(args.split)

    print("=== D1 生成 ===")
    print("split=%s master_seed=%d families=%s" % (args.split, args.master_seed, ",".join(families)))
    manifest, records = build_manifest(args.master_seed, args.split, families, abs_scen, abs_perc,
                                       abs_world, args.generated_at, a0_model=args.a0_model)
    write_outputs(manifest, records, abs_scen, abs_perc, abs_world, out_path)

    errs, backend = validate_manifest_schema(manifest)
    fam_counts = {}
    for e in manifest["entries"]:
        fam_counts[e["family"]] = fam_counts.get(e["family"], 0) + 1
    print("manifest: %s" % out_path)
    print("entries: %d" % len(manifest["entries"]))
    print("按 family 计数: %s" % json.dumps(fam_counts, ensure_ascii=False))
    print("场景文件: %s (%d 个)" % (abs_scen, len(records)))
    print("感知文件: %s (%d 个)" % (abs_perc, len(records)))
    print("world 文件: %s (%d 个)" % (abs_world, len(records)))
    if errs:
        print("SCHEMA 校验: FAIL (%s 后端, %d 处)" % (backend, len(errs)))
        for e in errs[:10]:
            print("  - %s" % e)
        return 1
    print("SCHEMA 校验: PASS (%s 后端, %s)" % (backend, SCHEMA_PATH))
    return 0


def cmd_rebuild_check(args):
    out_path = resolve_dir(args.out) if args.out else default_out_path(args.split)
    if not os.path.isfile(out_path):
        print("FAIL: 未找到 manifest: %s" % out_path)
        return 1
    with open(out_path, "rb") as f:
        existing_bytes = f.read()
    existing = json.loads(existing_bytes.decode("utf-8"))
    entries = existing["entries"]
    master = existing["master_seed"]
    splits = {e["split"] for e in entries}
    split = entries[0]["split"]
    families = [f for f in FAMILY_ORDER if f in {e["family"] for e in entries}]
    abs_scen = resolve_dir(args.scenarios_dir)
    abs_perc = resolve_dir(args.perception_dir)
    abs_world = resolve_dir(args.worlds_dir)

    print("=== D1 rebuild-check ===")
    print("已有 manifest: %s" % out_path)
    print("master_seed=%d split=%s families=%s" % (master, split, ",".join(families)))
    if len(splits) != 1:
        print("FAIL: manifest 含多 split: %s" % splits)
        return 1

    ok = True
    tmp = tempfile.mkdtemp(prefix="d1_rebuild_")
    try:
        # 重新生成：逻辑路径保持与已有 manifest 相同（默认默认目录），物理写临时目录
        regen, records = build_manifest(master, split, families, abs_scen, abs_perc, abs_world, None)
        regen_bytes = encode_manifest(regen)
        if regen_bytes == existing_bytes:
            print("PASS: manifest 逐字节一致 (%d 字节)" % len(regen_bytes))
        else:
            ok = False
            print("FAIL: manifest 不一致 (regen=%d existing=%d 字节)" % (len(regen_bytes), len(existing_bytes)))

        tmp_dirs = {"scenario": os.path.join(tmp, "scenarios"),
                    "sensors": os.path.join(tmp, "perception"),
                    "world": os.path.join(tmp, "worlds")}
        for d in tmp_dirs.values():
            os.makedirs(d, exist_ok=True)
        by_id = {e["scenario_id"]: e for e in entries}
        file_bad = 0
        for rec in records:
            e = by_id.get(rec["scenario_id"])
            if e is None:
                print("FAIL: 重新生成出现新 scenario_id: %s" % rec["scenario_id"])
                ok = False
                file_bad += 1
                continue
            for kind, entry_key in (("scenario", "scenario_file"), ("sensors", "sensors_file"),
                                    ("world", "world_file")):
                rel, content = rec[kind]
                base = os.path.basename(rel)
                tmp_path = os.path.join(tmp_dirs[kind], base)
                with open(tmp_path, "wb") as f:
                    f.write(content)
                orig_path = os.path.join(WORKSPACE, e[entry_key])
                if not os.path.isfile(orig_path):
                    print("FAIL: 缺少已有文件 %s" % orig_path)
                    ok = False
                    file_bad += 1
                    continue
                with open(orig_path, "rb") as f:
                    if content != f.read():
                        print("FAIL: 文件不一致 %s" % rel)
                        ok = False
                        file_bad += 1
        if file_bad == 0:
            print("PASS: %d 个场景/感知/world 文件逐字节一致" % (len(records) * 3))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------- 消融清单（协议 sample_size_plan.ablation_arms）
# fixed 臂：引用包内固定场景（各 4 个独立种子，共 8 条），world/sensors 用包内默认文件
FIXED_ARM_SCENARIOS = [
    ("src/air_ground_core/config/scenario_flat.yaml", 4),
    ("src/air_ground_core/config/scenario_linear_target.yaml", 4),
]
FIXED_ARM_WORLD = "src/air_ground_core/worlds/agc_world.world"
FIXED_ARM_SENSORS = "src/air_ground_core/config/perception_sensors.yaml"
ABLATION_CHOICES = ["fixed_vs_randomized", "remove_tracking_mechanism"]


def default_ablation_out_path(name):
    return os.path.join(WORKSPACE, "experiments", "d", "manifest_ablation_%s_v1.json" % name)


def build_ablation_manifest(name, master_seed, logical_scen_dir, logical_perc_dir, logical_world_dir):
    """产出消融清单（引用既有文件，不新写场景/感知/world）。"""
    proto = load_protocol()
    spacing = proto["sample_size_plan"].get("frequency_split", {}).get("train_seed_base_spacing", 1000)
    notes = ["消融清单: name=%s; 主种子=%d; split=test" % (name, master_seed),
             "协议=%s sample_size_plan.ablation_arms.%s" % (PROTOCOL_VERSION, name)]
    entries = []
    used_seeds = set()

    if name == "fixed_vs_randomized":
        notes.append("fixed 臂: 引用包内固定场景 %s；world=%s；sensors=%s"
                     % (",".join(x for x, _ in FIXED_ARM_SCENARIOS), FIXED_ARM_WORLD, FIXED_ARM_SENSORS))
        world_hash = sha256_file(os.path.join(WORKSPACE, FIXED_ARM_WORLD))
        sensors_hash = sha256_file(os.path.join(WORKSPACE, FIXED_ARM_SENSORS))
        idx = 0
        for rel, count in FIXED_ARM_SCENARIOS:
            sc = yaml.safe_load(open(os.path.join(WORKSPACE, rel), encoding="utf-8"))
            sc_hash = sha256_file(os.path.join(WORKSPACE, rel))
            for _ in range(count):
                seed = entry_seed(master_seed, "test", "ablation_fixed", idx, spacing, used_seeds)
                entries.append({
                    "scenario_id": "test_ablation_fixed_%03d_%09d" % (idx, seed),
                    "split": "test", "family": "ablation_fixed", "seed": seed,
                    "scenario_file": rel, "scenario_hash": sc_hash,
                    "sensors_file": FIXED_ARM_SENSORS, "sensors_hash": sensors_hash,
                    "world_file": FIXED_ARM_WORLD, "world_hash": world_hash,
                    "layout": None, "target_source": "gt_adapter", "obstacle_source": "scenario",
                    "inject": None, "pair_id": None,
                    "params": {"target_motion_mode": sc["target_motion"]["mode"],
                               "target_speed_mps": sc["target_motion"]["speed_mps"],
                               "num_obstacles": len(sc["obstacles"]),
                               "intended_unreachable": False, "boundary": False,
                               "fixed_source": rel},
                })
                idx += 1

    elif name == "remove_tracking_mechanism":
        # baseline = test 集 nominal 前 8 条（已存在于 manifest_test_v1.json）；此处只产出 fusion 配对臂
        nom, _ = build_manifest(master_seed, "test", ["nominal"], logical_scen_dir,
                                logical_perc_dir, logical_world_dir)
        base = nom["entries"][:8]
        assert len(base) == 8, "nominal baseline 不足 8 条: %d" % len(base)
        assert all(e["target_source"] == "gt_adapter" for e in base), "baseline 必须为 gt_adapter"
        notes.append("paired 臂: baseline=test 集 nominal 前 8 条(gt_adapter，见 manifest_test_v1.json)；"
                     "removed 臂=同场景/同世界/同种子改 target_source=air_fusion，pair_id 关联 baseline")
        for b in base:
            e = dict(b)
            e["scenario_id"] = b["scenario_id"] + "_fus"
            e["family"] = "ablation_track_mech"
            e["target_source"] = "air_fusion"
            e["pair_id"] = b["scenario_id"]
            entries.append(e)
    else:
        raise SystemExit("未知消融名称: %s" % name)

    assert len({e["seed"] for e in entries}) == len(entries), "存在重复 seed"
    assert len({e["scenario_id"] for e in entries}) == len(entries), "存在重复 scenario_id"
    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "protocol": PROTOCOL_VERSION,
        "master_seed": int(master_seed),
        "created_from": CREATED_FROM,
        "generated_at": derived_generated_at(master_seed),
        "config_hash": config_hash(),
        "notes": notes,
        "entries": entries,
    }
    return manifest, []


def cmd_ablation(args):
    name = args.ablation
    abs_scen = resolve_dir(args.scenarios_dir)
    abs_perc = resolve_dir(args.perception_dir)
    abs_world = resolve_dir(args.worlds_dir)
    out_path = resolve_dir(args.out) if args.out else default_ablation_out_path(name)

    print("=== D1 消融清单生成 ===")
    print("ablation=%s master_seed=%d" % (name, args.master_seed))
    manifest, _ = build_ablation_manifest(name, args.master_seed, abs_scen, abs_perc, abs_world)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(encode_manifest(manifest))

    errs, backend = validate_manifest_schema(manifest)
    counts = {}
    for e in manifest["entries"]:
        counts[e["family"]] = counts.get(e["family"], 0) + 1
    print("manifest: %s" % out_path)
    print("entries: %d  按 family: %s" % (len(manifest["entries"]), json.dumps(counts, ensure_ascii=False)))
    if errs:
        print("SCHEMA 校验: FAIL (%s 后端, %d 处)" % (backend, len(errs)))
        for e in errs[:10]:
            print("  - %s" % e)
        return 1
    print("SCHEMA 校验: PASS (%s 后端)" % backend)
    return 0


# ---------------------------------------------------------------- perception_fault 标定版 v2
# 缺陷：v1 把 sigma_xy 随机到 ~0.47-0.66，而 conf=1-exp(-tol²/(2σ²))（tol=0.25），
# 要 conf≥0.5 需 σ≤0.25/√(2ln2)≈0.2124，故 v1 该族构造上必然 conf<0.5（0/14 恒失效）。
# v2 保留 v1 原结果不动，另起版本：同场景/同世界/同种子，仅更换 sensors_file（配对对照）。
PERCFAULT_V2_DIR = "experiments/d/perception_v2"
PERCFAULT_V2_OUT = "experiments/d/manifest_percfault_v2.json"
TOL_M = 0.25  # calibration.confidence_tolerance_m


def _sensors_summary(sensors):
    ks = ("sigma_xy_m", "sigma_z_m", "latency_s", "drop_prob", "hfov_deg", "vfov_deg", "range_max_m")
    return {k: [s[k] for s in sensors["sensors"]] for k in ks}


def build_percfault_v2():
    """构建 v2 清单与感知文件（确定性，复用 v1 场景/世界/种子）。"""
    with open(os.path.join(WORKSPACE, "experiments", "d", "manifest_test_v1.json"), encoding="utf-8") as f:
        v1 = json.load(f)
    v1_entries = [e for e in v1["entries"] if e["family"] == "perception_fault"]
    assert v1_entries, "v1 manifest 缺少 perception_fault 条目"
    abs_v2 = resolve_dir(PERCFAULT_V2_DIR)
    notes = [
        "perception_fault 标定版 v2（split=test, family=perception_fault_v2）",
        "修正依据: conf=1-exp(-tol²/(2σ²)), tol=%.2f; conf≥0.5 需 σ≤%.4f(≈0.2124). "
        "v1 该族 σ_xy≈0.47-0.66 使 conf<0.5，构造上必然 0/14（退化臂）；"
        "v2 把 σ_xy∈[0.10,0.30] 等，成功可能但不保证（有信息量）。" % (TOL_M, TOL_M / math.sqrt(2 * math.log(2))),
        "配对关系: 每条 v2 复用 v1 同 scenario_id(+_v2)/scenario_file/world_file/seed，仅 sensors_file 不同；"
        "pair_id 指向 v1 scenario_id，params.base_id 记录基准；v1 原结果保持不变。",
        "声明: 本清单是对构造性缺陷(σ 超解析上界致恒失效)的修正，非对结果的事后调参。",
    ]
    entries = []
    files = []
    for e in v1_entries:
        rng = random.Random(hash31("%d|percfault_v2" % e["seed"]))
        sensors = make_sensors(rng, ranges=SENSOR_RANGE_FAULT_V2)
        body = dump_yaml(sensors)
        sid2 = e["scenario_id"] + "_v2"
        rel = os.path.relpath(os.path.join(abs_v2, "perception_%s.yaml" % sid2), WORKSPACE)
        ne = dict(e)
        ne["scenario_id"] = sid2
        ne["family"] = "perception_fault_v2"
        ne["sensors_file"] = rel
        ne["sensors_hash"] = sha256_hex(body)
        ne["pair_id"] = e["scenario_id"]
        params = dict(e["params"])
        params["base_id"] = e["scenario_id"]
        params["perception"] = _sensors_summary(sensors)
        ne["params"] = params
        entries.append(ne)
        files.append((rel, body))

    assert len({e["scenario_id"] for e in entries}) == len(entries), "存在重复 scenario_id"
    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "protocol": PROTOCOL_VERSION,
        "master_seed": int(v1["master_seed"]),
        "created_from": CREATED_FROM,
        "generated_at": derived_generated_at(v1["master_seed"]),
        "config_hash": config_hash(),
        "notes": notes,
        "entries": entries,
    }
    return manifest, files


def cmd_percfault_v2(args):
    out_path = os.path.join(WORKSPACE, PERCFAULT_V2_OUT)
    manifest, files = build_percfault_v2()
    regen_bytes = encode_manifest(manifest)

    if args.rebuild_check:
        if not os.path.isfile(out_path):
            print("FAIL: 未找到 manifest: %s" % out_path)
            return 1
        with open(out_path, "rb") as f:
            existing = f.read()
        ok = existing == regen_bytes
        print("=== percfault v2 rebuild-check ===")
        print("%s: manifest 逐字节一致 (%d 字节)" % ("PASS" if ok else "FAIL", len(regen_bytes)))
        bad = 0
        for rel, body in files:
            p = os.path.join(WORKSPACE, rel)
            if not os.path.isfile(p) or open(p, "rb").read() != body:
                print("FAIL: 文件不一致 %s" % rel); bad += 1
        if bad == 0:
            print("PASS: %d 个 v2 感知文件逐字节一致" % len(files))
        ok = ok and bad == 0
        print("RESULT: %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1

    os.makedirs(resolve_dir(PERCFAULT_V2_DIR), exist_ok=True)
    for rel, body in files:
        with open(os.path.join(WORKSPACE, rel), "wb") as f:
            f.write(body)
    with open(out_path, "wb") as f:
        f.write(regen_bytes)

    errs, backend = validate_manifest_schema(manifest)
    print("=== perception_fault 标定版 v2 ===")
    print("manifest: %s" % out_path)
    print("entries: %d  感知文件: %s (%d 个)" % (len(manifest["entries"]), PERCFAULT_V2_DIR, len(files)))
    if errs:
        print("SCHEMA 校验: FAIL (%s 后端, %d 处)" % (backend, len(errs)))
        for e in errs[:10]:
            print("  - %s" % e)
        return 1
    print("SCHEMA 校验: PASS (%s 后端)" % backend)
    return 0


# ---------------------------------------------------------------- A0 专用小 manifest（Task 6.3/7）
# 4 条场景（各 1 种子），输出到 experiments/a0/（不污染 experiments/d/），供 Task 8 冒烟与核验。
A0_DIR = "experiments/a0"
A0_SCEN_DIR = "experiments/a0/scenarios"
A0_PERC_DIR = "experiments/a0/perception"
A0_WORLD_DIR = "experiments/a0/worlds"
A0_MANIFEST_OUT = "experiments/a0/manifest_a0.json"
A0_CAR_START = [-4.0, -3.0, CAR_Z, 0.0]
A0_DRONE_HOVER = [0.0, 0.0, 2.0]   # 无人机悬停点（≈ takeoff_altitude 2.0）
MANIFEST_VERSION_A0 = "a0-manifest-v1"


def _a0_sensors(sid, fault=False, occlusion=None, override=None):
    """按场景 id 派生确定性随机流构造 A0 感知文件（基底字段 + A0 字段）。"""
    base_rng = random.Random(hash31("a0base|%s" % sid))
    ranges = SENSOR_RANGE_FAULT_V2 if fault else None
    return make_sensors(base_rng, fault=fault, ranges=ranges, a0=True,
                        a0_seed=hash31("a0|%s" % sid), occlusion=occlusion, a0_override=override)


# a0_replay：重放复测专用——目标 static 使真值恒定；A0 特征全部非零（显式覆盖保证）
A0_REPLAY_OVERRIDE = {
    "corr_xy": 0.25, "corr_rho": 0.5, "noise_tau_s": 0.2,
    "bias_m": [0.05, -0.03, 0.02], "drop_burst_prob": 0.3, "false_alarm_rate_hz": 0.25,
}


def build_a0_manifest(master_seed):
    """构建 A0 专用小 manifest 与全部文件记录（不落盘）。"""
    template = load_world_template()
    entries, records = [], []
    notes = [
        "A0 观测模型专用小集：5 条场景（nominal/occlusion/maneuver/fault/replay），各 1 种子；输出 experiments/a0/",
        "nominal: A0 字段随机化(相关/偏置/成簇丢失/虚警)，无障碍；occlusion: 视线遮挡门控开(pad=0.05) + 遮挡障碍；"
        "maneuver: 目标 sinusoid 机动(合速度<0.9)；fault: 强 A0 字段 + 成簇丢失 + 虚警；"
        "replay: 目标 static + A0 特征全开(非零)，用于重放可复现性复测",
        "target_source=air_fusion（启用 air_perception）；obstacle_source=scenario；A0 字段为工程取值，非真实标定",
    ]

    # 场景规格：(id, target_xy, motion, obstacles, a0_fault, occlusion, a0_override)
    occ_block = make_obstacle("occ_block", 2.5, 0.0, 0.6, 0.6, sz=3.0)
    # 校验遮挡障碍落在 无人机悬停点(0,0,2.0) → 目标穿越点(5,0,2.0) 连线上（目标跨 y=0 时被挡）
    _h = {"sx": occ_block["sx"], "sy": occ_block["sy"], "sz": occ_block["sz"],
          "x": occ_block["x"], "y": occ_block["y"], "z": occ_block["sz"] / 2.0, "yaw": occ_block["yaw"]}
    a = ((0.0 - _h["x"]), (0.0 - _h["y"]), (2.0 - _h["z"]))
    b = ((5.0 - _h["x"]), (0.0 - _h["y"]), (2.0 - _h["z"]))
    assert min(a[0], b[0]) <= _h["sx"] / 2 + 0.05 and max(a[0], b[0]) >= -(_h["sx"] / 2 + 0.05) \
        and abs(a[2]) <= _h["sz"] / 2, "a0_occlusion 遮挡障碍未落在视线连线上"

    specs = [
        # id, target_xy, mode, speed, wps, man, obstacles, a0_fault, occlusion, a0_override
        ("a0_nominal", [5.0, 3.0], "static", 0.0, [[5.0, 3.0]], None, [], False, None, None),
        ("a0_occlusion", [5.0, -3.0], "linear", 0.5, [[5.0, 3.0]], None, [occ_block], False, (True, 0.05), None),
        ("a0_maneuver", [2.0, -2.0], "sinusoid", 0.45, [],
         {"heading_deg": 0.0, "amplitude_m": 0.35, "period_s": 4.0}, [], False, None, None),
        ("a0_fault", [5.0, 3.0], "static", 0.0, [[5.0, 3.0]], None, [], True, None, None),
        # a0_replay：static 目标（真值恒定）+ 全 A0 特征非零，供重放可复现性复测
        ("a0_replay", [5.0, 3.0], "static", 0.0, [[5.0, 3.0]], None, [], False, None, A0_REPLAY_OVERRIDE),
    ]

    for sid, txy, mode, speed, wps, man, obstacles, a0_fault, occlusion, a0_override in specs:
        seed = hash31("a0|seed|%s" % sid)
        target = [r3(txy[0]), r3(txy[1]), TARGET_HEIGHT_M, 0.0]
        note = ("A0 专用场景 %s：观测模型对应场景（%s）" % (sid, mode))
        scenario = assemble(sid, note, list(A0_CAR_START), target, mode, speed, wps, obstacles, man)
        sensors = _a0_sensors(sid, fault=a0_fault, occlusion=occlusion, override=a0_override)
        scen_bytes = dump_yaml(scenario)
        perc_bytes = dump_yaml(sensors)
        world_bytes = render_world(scenario, template)
        scen_rel = os.path.relpath(os.path.join(resolve_dir(A0_SCEN_DIR), "scenario_%s.yaml" % sid), WORKSPACE)
        perc_rel = os.path.relpath(os.path.join(resolve_dir(A0_PERC_DIR), "perception_%s.yaml" % sid), WORKSPACE)
        world_rel = os.path.relpath(os.path.join(resolve_dir(A0_WORLD_DIR), "world_%s.world" % sid), WORKSPACE)
        params = {
            "target_motion_mode": mode, "target_speed_mps": speed,
            "num_obstacles": len(obstacles),
            "a0_observation_model": True, "a0_fault_strength": bool(a0_fault),
            "occlusion_enabled": bool(occlusion and occlusion[0]),
            "false_alarm_rate_hz": [s["false_alarm_rate_hz"] for s in sensors["sensors"]],
            "drop_burst_prob": [s["drop_burst_prob"] for s in sensors["sensors"]],
            "corr_rho": [s["corr_rho"] for s in sensors["sensors"]],
        }
        entries.append({
            "scenario_id": sid, "split": "a0", "family": "a0_%s" % sid.replace("a0_", ""),
            "seed": seed,
            "scenario_file": scen_rel, "scenario_hash": sha256_hex(scen_bytes),
            "sensors_file": perc_rel, "sensors_hash": sha256_hex(perc_bytes),
            "world_file": world_rel, "world_hash": sha256_hex(world_bytes),
            "layout": None, "target_source": "air_fusion", "obstacle_source": "scenario",
            "inject": None, "params": params,
        })
        records.append({"scenario_id": sid, "scenario": (scen_rel, scen_bytes),
                        "sensors": (perc_rel, perc_bytes), "world": (world_rel, world_bytes)})

    assert len({e["scenario_id"] for e in entries}) == 5, "A0 manifest 场景数不为 5"
    manifest = {
        "manifest_version": MANIFEST_VERSION_A0,
        "protocol": PROTOCOL_VERSION,
        "master_seed": int(master_seed),
        "created_from": CREATED_FROM,
        "generated_at": derived_generated_at(master_seed),
        "config_hash": config_hash(),
        "notes": notes,
        "entries": entries,
    }
    return manifest, records


def cmd_a0_manifest(args):
    out_path = resolve_dir(A0_MANIFEST_OUT)
    manifest, records = build_a0_manifest(args.master_seed)
    os.makedirs(resolve_dir(A0_SCEN_DIR), exist_ok=True)
    os.makedirs(resolve_dir(A0_PERC_DIR), exist_ok=True)
    os.makedirs(resolve_dir(A0_WORLD_DIR), exist_ok=True)
    for rec in records:
        for kind, d in (("scenario", A0_SCEN_DIR), ("sensors", A0_PERC_DIR), ("world", A0_WORLD_DIR)):
            rel, body = rec[kind]
            with open(os.path.join(resolve_dir(d), os.path.basename(rel)), "wb") as f:
                f.write(body)
    with open(out_path, "wb") as f:
        f.write(encode_manifest(manifest))
    print("=== A0 专用 manifest ===")
    print("manifest: %s" % out_path)
    print("场景文件: %s (%d 个)  感知文件: %s  世界文件: %s"
          % (A0_SCEN_DIR, len(records), A0_PERC_DIR, A0_WORLD_DIR))
    for e in manifest["entries"]:
        print("  %-14s seed=%-11d scenario_hash=%s sensors_hash=%s"
              % (e["scenario_id"], e["seed"], e["scenario_hash"][:12], e["sensors_hash"][:12]))
    return 0


def main(argv=None):
    args = parse_args(argv)
    if args.a0_manifest:
        return cmd_a0_manifest(args)
    if args.percfault_v2:
        return cmd_percfault_v2(args)
    if args.ablation:
        return cmd_ablation(args)
    if args.rebuild_check:
        return cmd_rebuild_check(args)
    return cmd_generate(args)


if __name__ == "__main__":
    sys.exit(main())
