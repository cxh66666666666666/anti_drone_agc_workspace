#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 单体性能与三维避障边界 —— 一体编排（离线实验工具，Python）。

按顺序执行各部分（各部分的仿真栈互斥，必须串行）：
  drone      无人机 RL 单体（固定 train/val/test 场景，三维动作可用性）
  ground     地面车 RL 单体（短训 DQN + 固定 val/test 评估）
  agc_lqr    AGC 无人机 LQR 轨迹跟踪误差（C++ 实时链路）
  agc_ground AGC 地面车跟踪与局部避障（C++ 协同链路）

用法:
  python3 scripts/run_m7_single_agent.py --parts ground,agc_lqr,agc_ground --gui
  python3 scripts/run_m7_single_agent.py --parts drone --gui
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_ROOT = os.path.join(WS, "experiments/m7")
PARTS = ("drone", "ground", "agc_lqr", "agc_ground")


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    env["PYTHONUNBUFFERED"] = "1"
    env.setdefault("DISPLAY", ":0")
    return env


def _bash(cmd):
    return subprocess.run(["bash", "-c", cmd], cwd=WS, env=_env(),
                          capture_output=True, text=True)


def _src():
    return "source /opt/ros/noetic/setup.bash && source devel/setup.bash"


def run(cmd, log_name):
    print("[M7] $ %s" % " ".join(cmd), flush=True)
    with open(os.path.join(OUT_ROOT, log_name), "wb") as logf:
        return subprocess.run(cmd, cwd=WS, env=_env(), stdout=logf,
                              stderr=subprocess.STDOUT).returncode


def wait_service(name, max_wait=90):
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if name in _bash("%s && rosservice list" % _src()).stdout:
            return True
        time.sleep(2)
    return False


def part_drone(gui):
    return run([sys.executable, "scripts/run_m7_drone_evidence.py",
                "--model", "training_runs/dqn_v5_raw_s101/best_model.pt",
                "--train-limit", "13", "--out-root", "experiments/m7/drone",
                "--gui" if gui else "--headless"], "drone_orchestrator.log")


def part_ground(gui):
    sim = subprocess.Popen(
        ["bash", "scripts/start_ground_sim.sh", "gui:=%s" % ("true" if gui else "false")],
        cwd=WS, env=_env(), stdout=open(os.path.join(OUT_ROOT, "ground_sim.log"), "w"),
        stderr=subprocess.STDOUT, start_new_session=True)
    code = 1
    try:
        if not wait_service("/ground_gazebo/set_model_state"):
            print("[M7] 地面仿真未就绪", flush=True)
            return 1
        code = run([sys.executable,
                    "src/rl_ground_avoidance/scripts/m7_ground_train_eval.py",
                    "--episodes", "200", "--max-steps", "150",
                    "--run-dir", "training_runs/car_m7",
                    "--out-dir", os.path.join(OUT_ROOT, "ground")],
                   "ground_train_eval.log")
    finally:
        try:
            os.killpg(os.getpgid(sim.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        time.sleep(5)
        _bash("pkill -9 -f ground_gazebo; pkill -9 -f ground_perception; "
              "pkill -9 -f gzclient; pkill -9 -f rosmaster; pkill -9 -f rosout")
    return code


def part_agc_lqr(gui):
    return run([sys.executable, "scripts/m7_lqr_tracking_eval.py",
                "--out-dir", os.path.join(OUT_ROOT, "agc/lqr")]
               + (["--gui"] if gui else []), "agc_lqr.log")


def part_agc_ground(gui):
    return run([sys.executable, "scripts/m7_agc_ground_single.py",
                "--out-dir", os.path.join(OUT_ROOT, "agc/ground")]
               + ([] if gui else []), "agc_ground.log")


def main():
    parser = argparse.ArgumentParser(description="M7 一体编排")
    parser.add_argument("--parts", default="drone,ground,agc_lqr,agc_ground")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    os.makedirs(OUT_ROOT, exist_ok=True)

    selected = [p for p in args.parts.split(",") if p in PARTS]
    report = {"created_at": datetime.now().isoformat(), "gui": bool(args.gui),
              "parts": {}}
    dispatch = {"drone": part_drone, "ground": part_ground,
                "agc_lqr": part_agc_lqr, "agc_ground": part_agc_ground}
    for part in selected:
        print("=" * 60)
        print("[M7] 开始 part: %s" % part, flush=True)
        try:
            report["parts"][part] = {"returncode": dispatch[part](args.gui)}
        except Exception as exc:  # noqa: BLE001
            report["parts"][part] = {"error": str(exc)}
    with open(os.path.join(OUT_ROOT, "m7_run_report.json"), "w",
              encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[M7] 完成: %s" % os.path.join(OUT_ROOT, "m7_run_report.json"))


if __name__ == "__main__":
    main()