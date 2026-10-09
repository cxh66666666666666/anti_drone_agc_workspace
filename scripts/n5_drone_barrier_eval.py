#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5-A 屏障对比评估编排（离线实验工具，Python）。

串行评估 low_barrier / tall_barrier 两个场景组（绝不并发启动两个仿真）：
  1) 用该组 manifest 启动一次无人机单体仿真（scripts/start_sim.sh ...）；
  2) 等待就绪后调用 m7_drone_eval.py（现有 DQN 模型、greedy 策略）；
  3) 停止仿真，解析轨迹统计「越顶 / 水平绕行」并汇总报告。

用法:
  python3 scripts/n5_drone_barrier_eval.py --headless \
      --model training_runs/dqn_v5_raw_s101/best_model.pt
"""

import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime


WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(WS, "src/rl_obstacle_avoidance/scripts")
RUNNER = os.path.join(SCRIPTS, "m7_drone_eval.py")
START_SIM = os.path.join(WS, "scripts/start_sim.sh")
sys.path.insert(0, SCRIPTS)

from n5_barrier_scenarios import (GROUPS, barrier_top_m, build_groups,
                                  manifest_sha256, write_manifests)

# 显式判据阈值（同时写入报告，便于复核）
NEAR_WINDOW_M = 0.75     # 屏障 x 邻域窗口（|pose_x - barrier_x| <= 该值）
CLIMB_MARGIN_M = 0.1     # 越顶：pose_z > barrier_top + 0.1
LATERAL_SMALL_M = 0.45   # 越顶时横向偏移须小（< 屏障半跨 0.4 + 裕度）
LATERAL_LARGE_M = 0.7    # 绕行时横向偏移须大（> 屏障半跨 0.4 + 机体半宽 0.35）


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    return env


def _run_shell(cmd, timeout=None):
    return subprocess.run(["bash", "-c", cmd], cwd=WS, env=_env(),
                          capture_output=True, text=True, timeout=timeout)


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def wait_ready(max_wait=240):
    """等待无人机单体栈就绪：lqr_tracker_node + obstacle_features + 起飞到位。"""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        nodes = _run_shell("source /opt/ros/noetic/setup.bash && "
                           "source devel/setup.bash && rosnode list").stdout
        topics = _run_shell("source /opt/ros/noetic/setup.bash && "
                            "source devel/setup.bash && rostopic list").stdout
        if ("/lqr_tracker_node" in nodes and "/obstacle_features" in topics
                and "/perception_node" in nodes):
            print("[N5] 无人机单体栈就绪", flush=True)
            return True
        time.sleep(3)
    return False


def start_sim(manifest, gui, log_path):
    """按该组 manifest 启动一次仿真（与 run_m7_drone_evidence 一致）。"""
    cmd = ("bash %s lqr --training --altitude 5.0 --trajectory "
           "fixed_horizontal --scenario-manifest %s %s"
           % (START_SIM, manifest, "--gui" if gui else ""))
    print("[N5] 启动仿真: %s" % cmd, flush=True)
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(cmd, shell=True, cwd=WS, env=_env(),
                            stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, log


def stop_sim(proc, log):
    """干净停止仿真进程（与 run_m7_drone_evidence 相同的兜底清理）。"""
    print("[N5] 停止仿真", flush=True)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(8)
    _run_shell("pkill -9 -f gzclient; pkill -9 -f gzserver; "
               "pkill -9 -f px4; pkill -9 -f mavros; "
               "pkill -9 -f lqr_tracker_node; "
               "pkill -9 -f perception_node; "
               "pkill -9 -f drone_launch_controller")
    if log is not None:
        log.close()


def run_eval(model, manifest, out_dir, max_steps):
    cmd = [sys.executable, RUNNER, "--model", model, "--manifest", manifest,
           "--split", "test", "--max-steps", str(max_steps),
           "--out-dir", out_dir]
    print("[N5] 评估: %s" % " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=WS, env=_env(), capture_output=True,
                          text=True)
    print("\n".join(proc.stdout.splitlines()[-6:]))
    metrics_path = os.path.join(out_dir, "metrics.json")
    if not os.path.isfile(metrics_path):
        print("[N5] 缺少 metrics.json，stderr 尾部:\n%s"
              % "\n".join(proc.stderr.splitlines()[-6:]))
        return False
    return True


def _barrier_lookup():
    """scenario_id -> {x, y, top}，用于越顶/绕行判定。"""
    lookup = {}
    for scenarios in build_groups().values():
        for scenario in scenarios:
            obstacle = scenario["obstacles"][0]
            lookup[scenario["scenario_id"]] = {
                "x": float(obstacle["center"][0]),
                "y": float(obstacle["center"][1]),
                "top": float(obstacle["center"][2]) + 0.5 * float(
                    obstacle["size"][2]),
            }
    return lookup


def _load_jsonl(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def analyze_group(out_dir, lookup):
    """解析该组 trajectories/*.jsonl + episodes.jsonl 得越顶/绕行与聚合。"""
    records = _load_jsonl(os.path.join(out_dir, "episodes.jsonl"))
    denominator = len(records)
    overfly = detour = 0
    vertical_total = steps_total = 0
    successes = collisions = timeouts = 0
    clearances = []
    for rec in records:
        steps_total += int(rec.get("steps") or 0)
        vertical_total += int(rec.get("vertical_action_count") or 0)
        successes += int(bool(rec.get("episode_success")))
        collisions += int(rec.get("done_reason") == "collision")
        timeouts += int(rec.get("done_reason") == "timeout")
        clearance = rec.get("min_geometry_clearance")
        if clearance is not None:
            clearances.append(float(clearance))

        barrier = lookup.get(str(rec.get("scenario_id")))
        traj_file = rec.get("trajectory_file")
        if not barrier or not traj_file:
            continue
        rows = [r for r in _load_jsonl(os.path.join(out_dir, traj_file))
                if r.get("pose_x") is not None and r.get("pose_y") is not None
                and r.get("pose_z") is not None]
        near = [r for r in rows
                if abs(float(r["pose_x"]) - barrier["x"]) <= NEAR_WINDOW_M]
        if not near:
            continue
        threshold = barrier["top"] + CLIMB_MARGIN_M
        max_lateral = max(abs(float(r["pose_y"]) - barrier["y"])
                          for r in near)
        max_z = max(float(r["pose_z"]) for r in near)
        is_overfly = any(
            float(r["pose_z"]) > threshold and
            abs(float(r["pose_y"]) - barrier["y"]) <= LATERAL_SMALL_M
            for r in near)
        if is_overfly:
            overfly += 1
        elif max_lateral >= LATERAL_LARGE_M and max_z <= threshold:
            detour += 1

    return {
        "episodes": denominator,
        "overfly_episodes": overfly,
        "detour_episodes": detour,
        "aggregates": {
            "success_rate": (successes / denominator if denominator else 0.0),
            "collision_rate": (collisions / denominator if denominator else 0.0),
            "timeout_rate": (timeouts / denominator if denominator else 0.0),
            "failure_denominator": denominator,
            "vertical_action_rate": (vertical_total / steps_total
                                     if steps_total else 0.0),
            "min_clearance": (min(clearances) if clearances else None),
        },
    }


def main():
    parser = argparse.ArgumentParser(description="N5-A 屏障对比评估编排")
    parser.add_argument("--model", default="training_runs/dqn_v5_raw_s101/"
                                           "best_model.pt")
    parser.add_argument("--max-steps", type=int, default=450)
    parser.add_argument("--out-root", default="experiments/n5/drone")
    gui = parser.add_mutually_exclusive_group()
    gui.add_argument("--gui", action="store_true", help="启动 Gazebo GUI")
    gui.add_argument("--headless", action="store_true",
                     help="无头（默认；保留为显式写法）")
    args = parser.parse_args()

    out_root = os.path.join(WS, args.out_root)
    model = os.path.join(WS, args.model)
    os.makedirs(out_root, exist_ok=True)
    manifest_dir = os.path.join(out_root, "manifests")
    if not all(os.path.isfile(os.path.join(manifest_dir, "%s.json" % g))
               for g in GROUPS):
        write_manifests(manifest_dir)

    lookup = _barrier_lookup()
    report = {
        "created_at": datetime.now().isoformat(),
        "model": os.path.abspath(model),
        "model_sha256": _sha256(model),
        "groups": {},
        "manifest_sha256": {g: manifest_sha256(g) for g in GROUPS},
        "thresholds": {
            "near_window_m": NEAR_WINDOW_M,
            "climb_margin_m": CLIMB_MARGIN_M,
            "lateral_small_m": LATERAL_SMALL_M,
            "lateral_large_m": LATERAL_LARGE_M,
            "overfly_rule": "屏障 x 邻域内存在帧 pose_z > barrier_top + "
                            "climb_margin 且横向偏移 <= lateral_small",
            "detour_rule": "非越顶，且邻域最大横向偏移 >= lateral_large 且"
                           "邻域最大 pose_z <= barrier_top + climb_margin",
        },
    }

    # 串行处理：每个组完整「启动→评估→停止」，绝不并发两个仿真。
    for group in GROUPS:
        manifest = os.path.join(manifest_dir, "%s.json" % group)
        group_dir = os.path.join(out_root, group)
        print("[N5] ===== 组 %s (barrier_top=%.2f m) ====="
              % (group, barrier_top_m(group)), flush=True)
        proc = log = None
        try:
            proc, log = start_sim(
                manifest, args.gui,
                os.path.join(out_root, "%s_sim.log" % group))
            if not wait_ready():
                print("[N5] %s 仿真未在时限内就绪" % group)
                report["groups"][group] = {"error": "sim_not_ready",
                                           "barrier_top_m": barrier_top_m(group)}
                continue
            run_eval(model, manifest, group_dir, args.max_steps)
        finally:
            if proc is not None:
                stop_sim(proc, log)
        result = analyze_group(group_dir, lookup)
        result["barrier_top_m"] = barrier_top_m(group)
        result["manifest"] = os.path.abspath(manifest)
        report["groups"][group] = result

    # 两组对比（越顶差异 / 绕行差异 / 竖向动作差异）。
    low = report["groups"].get("low_barrier", {})
    tall = report["groups"].get("tall_barrier", {})
    report["comparison"] = {
        "overfly_episodes_delta_low_minus_tall":
            low.get("overfly_episodes", 0) - tall.get("overfly_episodes", 0),
        "detour_episodes_delta_tall_minus_low":
            tall.get("detour_episodes", 0) - low.get("detour_episodes", 0),
        "vertical_action_rate_delta_low_minus_tall":
            (low.get("aggregates", {}).get("vertical_action_rate", 0.0) -
             tall.get("aggregates", {}).get("vertical_action_rate", 0.0)),
    }

    report_path = os.path.join(out_root, "n5_barrier_report.json")
    with open(report_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[N5] 报告: %s" % report_path)


if __name__ == "__main__":
    main()