#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 无人机 RL 单体证据编排（离线实验工具，Python）。

单命令：
  1) 以 `scripts/start_sim.sh lqr --training --gui` 启动无人机单体仿真；
  2) 等待就绪后，对固定 train/val/test 场景清单分别调用 m7_drone_eval；
  3) 干净退出，汇总 `drone_report.json`。

用法:
  python3 scripts/run_m7_drone_evidence.py \
      --model training_runs/dqn_v5_raw_s101/best_model.pt \
      --train-limit 13 --gui
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
RUNNER = os.path.join(WS, "src/rl_obstacle_avoidance/scripts/m7_drone_eval.py")
START_SIM = os.path.join(WS, "scripts/start_sim.sh")


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    return env


def _run_shell(cmd, timeout=None):
    return subprocess.run(["bash", "-c", cmd], cwd=WS, env=_env(),
                          capture_output=True, text=True, timeout=timeout)


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
            print("[M7] 无人机单体栈就绪", flush=True)
            return True
        time.sleep(3)
    return False


def run_eval(model, manifest, split, limit, out_dir, hidden):
    cmd = [sys.executable, RUNNER, "--model", model, "--manifest", manifest,
           "--split", split, "--max-steps", "450", "--hidden-dims", hidden,
           "--out-dir", out_dir]
    if limit > 0:
        cmd += ["--limit", str(limit)]
    print("[M7] 评估 %s: %s" % (split, " ".join(cmd)))
    proc = subprocess.run(cmd, cwd=WS, env=_env(), capture_output=True,
                          text=True)
    tail = "\n".join(proc.stdout.splitlines()[-6:])
    print(tail)
    metrics_path = os.path.join(out_dir, "metrics.json")
    if not os.path.isfile(metrics_path):
        print("[M7] %s 缺少 metrics.json，stdout 尾部:\n%s" % (split, tail))
        return None
    with open(metrics_path, encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser = argparse.ArgumentParser(description="M7 无人机 RL 证据编排")
    parser.add_argument("--model", required=True)
    parser.add_argument("--manifests-dir",
                        default="experiments/stage2_v4/manifests")
    parser.add_argument("--train-limit", type=int, default=13)
    parser.add_argument("--limit", type=int, default=0,
                        help="val/test 场景上限（0=全部）")
    parser.add_argument("--hidden-dims", default="128,128")
    parser.add_argument("--out-root", default="experiments/m7/drone")
    parser.add_argument("--gui", action="store_true",
                        help="启动 Gazebo GUI（非无头）")
    parser.add_argument("--keep-sim", action="store_true",
                        help="保留仿真进程（调试用）")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    out_root = os.path.join(WS, args.out_root, stamp)
    os.makedirs(out_root, exist_ok=True)

    sim_cmd = ("bash %s lqr --training --altitude 5.0 "
               "--trajectory fixed_horizontal --scenario-manifest %s %s"
               % (START_SIM,
                  os.path.join(args.manifests_dir, "train.json"),
                  "--gui" if args.gui else ""))
    print("[M7] 启动仿真: %s" % sim_cmd)
    sim_log = open(os.path.join(out_root, "sim.log"), "w")
    sim_proc = subprocess.Popen(sim_cmd, shell=True, cwd=WS, env=_env(),
                                stdout=sim_log, stderr=subprocess.STDOUT,
                                start_new_session=True)

    report = {"created_at": stamp, "gui": bool(args.gui), "model": args.model}
    try:
        if not wait_ready():
            print("[M7] 仿真未在时限内就绪")
            report["error"] = "sim_not_ready"
        else:
            manifest = os.path.join(args.manifests_dir, "%s.json")
            report["train"] = run_eval(
                args.model, manifest % "train", "train", args.train_limit,
                os.path.join(out_root, "train"), args.hidden_dims)
            report["validation"] = run_eval(
                args.model, manifest % "validation", "validation", args.limit,
                os.path.join(out_root, "validation"), args.hidden_dims)
            report["test"] = run_eval(
                args.model, manifest % "test", "test", args.limit,
                os.path.join(out_root, "test"), args.hidden_dims)
    finally:
        if not args.keep_sim:
            print("[M7] 停止仿真")
            try:
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            time.sleep(8)
            _run_shell("pkill -9 -f gzclient; pkill -9 -f gzserver; "
                       "pkill -9 -f px4; pkill -9 -f mavros; "
                       "pkill -9 -f lqr_tracker_node; "
                       "pkill -9 -f perception_node; "
                       "pkill -9 -f drone_launch_controller")
        sim_log.close()

    with open(os.path.join(out_root, "drone_report.json"), "w",
              encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[M7] 报告: %s" % os.path.join(out_root, "drone_report.json"))


if __name__ == "__main__":
    main()