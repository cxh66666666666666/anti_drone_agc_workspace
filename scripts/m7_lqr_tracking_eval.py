#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 AGC 单体性能：无人机 LQR 轨迹跟踪误差（C++ 实时链路）。

对多种轨迹类型启动 `scripts/start_sim.sh lqr`（C++ 的 lqr_tracker_node 发布
`/lqr_tracker/position_error`），记录误差序列并给出整段统计量，落盘 JSON。

用法:
  python3 scripts/m7_lqr_tracking_eval.py --gui --out-dir experiments/m7/agc/lqr
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import time
from datetime import datetime

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
START_SIM = os.path.join(WS, "scripts/start_sim.sh")

# 轨迹 -> (额外参数, 记录时长秒)
TRAJECTORIES = {
    "linear": (["--trajectory-duration", "20"], 20),
    "circular": (["--trajectory-duration", "40", "--angular-velocity", "0.2"], 30),
    "fixed_horizontal": (["--trajectory-duration", "40"], 30),
    "figure_eight": (["--trajectory-duration", "103", "--angular-velocity", "0.1"], 40),
}


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    return env


def _bash(cmd, timeout=None):
    return subprocess.run(["bash", "-c", cmd], cwd=WS, env=_env(),
                          capture_output=True, text=True, timeout=timeout)


def _source():
    return "source /opt/ros/noetic/setup.bash && source devel/setup.bash"


def wait_ready(topic, max_wait=240):
    """等待 lqr_tracker_node 节点与误差话题均就绪（避免过早采样）。"""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        nodes = _bash("%s && rosnode list" % _source()).stdout
        topics = _bash("%s && rostopic list" % _source()).stdout
        if "/lqr_tracker_node" in nodes and topic in topics:
            return True
        time.sleep(3)
    return False


def record_error(topic, duration):
    """用 rostopic echo 记录 duration 秒，解析 `data:` 行。"""
    cmd = ("%s && timeout %d rostopic echo %s" % (_source(), duration, topic))
    proc = _bash(cmd, timeout=duration + 20)
    values = []
    for line in proc.stdout.splitlines():
        m = re.match(r"\s*data:\s*([-0-9.eE+]+)", line)
        if m:
            try:
                values.append(float(m.group(1)))
            except ValueError:
                pass
    return values


def stats(values):
    if not values:
        return {"n": 0}
    n = len(values)
    mean = sum(values) / n
    rms = math.sqrt(sum(v * v for v in values) / n)
    ordered = sorted(values)
    p95 = ordered[min(n - 1, int(0.95 * n))]
    return {"n": n, "mean_m": mean, "rms_m": rms, "max_m": max(values),
            "p95_m": p95}


def stop_sim():
    # 用 bracket 模式避免 pkill 匹配到执行它的 shell 自身
    _bash("pkill -9 -f '[l]qr_tracker_node'; "
          "pkill -9 -f '[d]rone_launch_controller'; pkill -9 -f '[g]zclient'; "
          "pkill -9 -f '[g]zserver'; pkill -9 -f '[p]x4'; "
          "pkill -9 -f '[m]avros'; pkill -9 -f '[r]osmaster'; "
          "pkill -9 -f '[r]osout'")
    time.sleep(6)


def main():
    parser = argparse.ArgumentParser(description="M7 LQR 跟踪误差评估")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--topic", default="/lqr_tracker/position_error")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    report = {"created_at": datetime.now().isoformat(),
              "topic": args.topic, "gui": bool(args.gui), "trajectories": {}}
    for name, (extra, dur) in TRAJECTORIES.items():
        print("[M7] 轨迹 %s ..." % name, flush=True)
        sim = subprocess.Popen(
            ["bash", START_SIM, "lqr", "--altitude", "3.0",
             "--trajectory", name, "--gui" if args.gui else "--headless"]
            + extra,
            cwd=WS, env=_env(), stdout=open(
                os.path.join(args.out_dir, "sim_%s.log" % name), "w"),
            stderr=subprocess.STDOUT, start_new_session=True)
        entry = {"duration_s": dur}
        try:
            if not wait_ready(args.topic):
                entry["error"] = "topic_not_ready"
            else:
                time.sleep(8)  # 让轨迹越过起步瞬态进入稳态
                values = record_error(args.topic, dur)
                entry.update(stats(values))
                if entry.get("n", 0) <= 0:
                    entry["error"] = "no_samples"
                print("  %s: n=%d mean=%.4f rms=%.4f max=%.4f"
                      % (name, entry.get("n", 0), entry.get("mean_m", 0),
                         entry.get("rms_m", 0), entry.get("max_m", 0)),
                      flush=True)
        finally:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            stop_sim()
        report["trajectories"][name] = entry

    with open(os.path.join(args.out_dir, "lqr_tracking_report.json"), "w",
              encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[M7] 报告: %s" % os.path.join(args.out_dir,
                                          "lqr_tracking_report.json"))


if __name__ == "__main__":
    main()