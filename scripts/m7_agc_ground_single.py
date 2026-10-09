#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 AGC 单体性能：地面车跟踪与局部避障（C++ 协同实时链路）。

运行固定静态障碍场景一轮（`start_agc_collab.sh --gui --scenario
scenario_flat.yaml`），从回放日志提取地面单体指标：跟踪误差（车到目标地面
投影距离）、局部避障净空、重规划触发、安全触发与终态，落盘 JSON。

用法:
  python3 scripts/m7_agc_ground_single.py --gui --out-dir experiments/m7/agc/ground
"""

import argparse
import json
import math
import os
import re
import subprocess
from datetime import datetime

WS = "/home/ubuntu/anti_drone_agc_workspace"
START = os.path.join(WS, "scripts/start_agc_collab.sh")
SCENARIO = os.path.join(WS, "src/air_ground_core/config/scenario_flat.yaml")
PROTECTION_M = 0.80  # 契约 §5.2 地面车保护距离


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    env.setdefault("DISPLAY", ":0")
    return env


def parse_jsonl(path):
    rows = []
    try:
        with open(path, encoding="utf-8", errors="ignore") as stream:
            for line in stream:
                line = line.strip()
                if line:
                    try:
                        rows.append(json.loads(line))
                    except ValueError:
                        pass
    except OSError:
        pass
    return rows


def stats(values):
    vals = [v for v in values if v is not None and math.isfinite(v)]
    if not vals:
        return {"n": 0}
    n = len(vals)
    ordered = sorted(vals)
    return {"n": n, "mean_m": sum(vals) / n, "max_m": max(vals),
            "p95_m": ordered[min(n - 1, int(0.95 * n))]}


def run_scenario(run_timeout, out_dir):
    log_path = os.path.join(out_dir, "agc_single.log")
    cmd = ["bash", START, "--gui", "--cycles", "1", "--repeat", "1",
           "--run-timeout", str(run_timeout), "--scenario", SCENARIO,
           "--batch-prefix", "_m7_single_"]
    print("[M7] 运行 AGC 静态场景: %s" % " ".join(cmd), flush=True)
    with open(log_path, "wb") as logf:
        subprocess.run(cmd, cwd=WS, env=_env(), stdout=logf,
                       stderr=subprocess.STDOUT)
    with open(log_path, encoding="utf-8", errors="ignore") as stream:
        match = re.search(r"batch_dir=(\S+)", stream.read())
    return match.group(1) if match else None


def extract(run_dir):
    tracking = parse_jsonl(os.path.join(run_dir, "tracking_frames.jsonl"))
    planner = parse_jsonl(os.path.join(run_dir, "planner_log.jsonl"))
    states = parse_jsonl(os.path.join(run_dir, "state_events.jsonl"))
    drive = parse_jsonl(os.path.join(run_dir, "drive_log.jsonl"))
    try:
        with open(os.path.join(run_dir, "summary.json"), encoding="utf-8") as s:
            summary = json.load(s)
    except OSError:
        summary = {}

    approach_t = None
    for ev in states:
        if ev.get("to") == "GROUND_APPROACH":
            approach_t = ev.get("t_sim")
            break
    window = [r for r in tracking if approach_t is None
              or r.get("t_sim", 0) >= approach_t]

    triggers = {}
    ok_replans = 0
    for row in planner:
        trig = row.get("trigger", "?")
        triggers[trig] = triggers.get(trig, 0) + 1
        if row.get("success"):
            ok_replans += 1

    clearances = [r.get("clearance_m") for r in window]
    violations = sum(1 for c in clearances
                     if c is not None and c < PROTECTION_M)
    distances = [r.get("distance_m") for r in window]
    # 末段（最后 5s）持续跟踪距离与末帧距离
    tail_vals = []
    last_distance = None
    if window:
        t_end = window[-1].get("t_sim", 0.0)
        tail_vals = [r.get("distance_m") for r in window
                     if r.get("t_sim", 0.0) >= t_end - 5.0]
        last_distance = window[-1].get("distance_m")

    return {
        "run_dir": os.path.basename(run_dir),
        "final_state": summary.get("final_state"),
        "final_state_name": summary.get("final_state_name"),
        "final_reason_code": summary.get("final_reason_code"),
        "ground_approach_t_sim": approach_t,
        "tracking_frames": len(window),
        "tracking_error": stats(distances),
        "tracking_error_tail5s": stats(tail_vals),
        "last_distance_m": last_distance,
        "min_clearance_m": (min(clearances) if clearances else None),
        "clearance_violations": violations,
        "protection_m": PROTECTION_M,
        "planner_calls": len(planner),
        "planner_ok_replans": ok_replans,
        "planner_triggers": triggers,
        "cmd_vel_samples": len(drive),
    }


def main():
    parser = argparse.ArgumentParser(description="M7 AGC 地面单体验证")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--run-timeout", type=int, default=160)
    parser.add_argument("--batch-dir", default=None,
                        help="复用已存在的批跑目录（跳过重新运行）")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    batch_dir = args.batch_dir or run_scenario(args.run_timeout, args.out_dir)
    report = {"created_at": datetime.now().isoformat(), "batch_dir": batch_dir,
              "scenario": os.path.basename(SCENARIO)}
    if batch_dir and os.path.isdir(batch_dir):
        # run 目录是 batch_dir 的同级目录，由 records.jsonl 的 attempt.run_dir 给出
        attempted = [r for r in parse_jsonl(os.path.join(batch_dir,
                                                         "records.jsonl"))
                     if r.get("type") == "attempt" and r.get("run_dir")]
        report["attempts"] = len(attempted)
        report["runs"] = [extract(r["run_dir"]) for r in attempted
                          if os.path.isfile(os.path.join(r["run_dir"],
                                                         "summary.json"))]
    else:
        report["error"] = "no_batch_dir"

    with open(os.path.join(args.out_dir, "agc_ground_report.json"), "w",
              encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[M7] 报告: %s" % os.path.join(args.out_dir,
                                          "agc_ground_report.json"))


if __name__ == "__main__":
    main()