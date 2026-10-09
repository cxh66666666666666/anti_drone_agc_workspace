#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5 D：LQR 轨迹跟踪误差的显式稳态窗口统计。

在 M7 整段统计之上，额外给出三类窗口统计，排除起步瞬态与分段切换瞬态：
  full         : 全部采样
  post_startup : 丢弃记录起点后的前 startup_exclude_s 秒
  steady_tail  : 最后 tail_s 秒（有明确边界的稳态窗口）
并给出 segment_change 说明：定位最大误差尖峰，判断其落在瞬态窗内还是之后。

用法:
  python3 scripts/n5_lqr_window_eval.py --headless \
      --out-dir experiments/n5/agc/lqr
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

# 与 M7 保持同一轨迹矩阵: 轨迹 -> (额外参数, 记录时长秒)
TRAJECTORIES = {
    "linear": (["--trajectory-duration", "20"], 20),
    "circular": (["--trajectory-duration", "40", "--angular-velocity", "0.2"], 30),
    "fixed_horizontal": (["--trajectory-duration", "40"], 30),
    "figure_eight": (["--trajectory-duration", "103", "--angular-velocity", "0.1"], 40),
}

# 窗口边界规则（落盘到报告，供复核）
WINDOW_RULE = (
    "时间基准为录制时按行读取 /lqr_tracker/position_error 采样的墙钟时间（t=0 为开始 echo 的时刻）。"
    "full: 全部采样；post_startup: 仅保留 t>=startup_exclude_s（默认 10.0s，用于剔除起步与分段切换瞬态）；"
    "steady_tail: 仅保留 t>=t_end-tail_s（默认 t_end 前最后 10.0s，一段明确边界的稳态段）。"
)


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    env["PYTHONUNBUFFERED"] = "1"  # 保证 rostopic echo 逐行及时输出
    return env


def _bash(cmd, timeout=None):
    return subprocess.run(["bash", "-c", cmd], cwd=WS, env=_env(),
                          capture_output=True, text=True, timeout=timeout)


def _source():
    return "source /opt/ros/noetic/setup.bash && source devel/setup.bash"


def wait_ready(topic, max_wait=240):
    """等待 lqr_tracker_node 节点与误差话题均就绪。"""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        nodes = _bash("%s && rosnode list" % _source()).stdout
        topics = _bash("%s && rostopic list" % _source()).stdout
        if "/lqr_tracker_node" in nodes and topic in topics:
            return True
        time.sleep(3)
    return False


def record_error_ts(topic, duration):
    """流式记录 duration 秒，返回 [(相对秒, 误差), ...]（携带时间戳）。"""
    cmd = "%s && timeout %d rostopic echo %s" % (_source(), duration, topic)
    proc = subprocess.Popen(["bash", "-c", cmd], cwd=WS, env=_env(),
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, bufsize=1, start_new_session=True)
    t0 = time.time()
    samples = []
    try:
        for line in proc.stdout:
            m = re.match(r"\s*data:\s*([-0-9.eE+]+)", line)
            if m:
                try:
                    samples.append((time.time() - t0, float(m.group(1))))
                except ValueError:
                    pass
    finally:
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    return samples


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


def window_stats(samples, startup_exclude_s, tail_s):
    """按规则计算 full / post_startup / steady_tail 三窗口统计。"""
    if not samples:
        return {"full": {"n": 0}, "post_startup": {"n": 0},
                "steady_tail": {"n": 0}}
    t_end = samples[-1][0]
    full = [v for _, v in samples]
    post = [v for t, v in samples if t >= startup_exclude_s]
    tail = [v for t, v in samples if t >= (t_end - tail_s)]
    return {"full": stats(full), "post_startup": stats(post),
            "steady_tail": stats(tail)}


def segment_change_note(samples, startup_exclude_s, top_k=5):
    """定位最大误差尖峰，判断落在瞬态窗内还是之后。"""
    if not samples:
        return {"top_spikes": [], "inside_startup": 0, "after_startup": 0,
                "note": "无采样"}
    tops = sorted(samples, key=lambda s: s[1], reverse=True)[:top_k]
    spikes = [{"t_s": round(t, 3), "error_m": v,
               "phase": "startup_transient" if t < startup_exclude_s
                        else "post_startup"} for t, v in tops]
    inside = sum(1 for s in spikes if s["phase"] == "startup_transient")
    after = len(spikes) - inside
    note = ("前 %d 大误差尖峰中有 %d 个落在前 %.1fs 瞬态窗内，%d 个在其后；"
            "若尖峰集中于瞬态窗内，则 post_startup 窗口可有效剔除瞬态。"
            % (len(spikes), inside, startup_exclude_s, after))
    return {"top_spikes": spikes, "inside_startup": inside,
            "after_startup": after, "note": note}


def stop_sim():
    _bash("pkill -9 -f '[l]qr_tracker_node'; "
          "pkill -9 -f '[d]rone_launch_controller'; pkill -9 -f '[g]zclient'; "
          "pkill -9 -f '[g]zserver'; pkill -9 -f '[p]x4'; "
          "pkill -9 -f '[m]avros'; pkill -9 -f '[r]osmaster'; "
          "pkill -9 -f '[r]osout'")
    time.sleep(6)


def main():
    parser = argparse.ArgumentParser(description="N5 LQR 稳态窗口评估")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--out-dir", default="experiments/n5/agc/lqr")
    parser.add_argument("--startup-exclude-s", type=float, default=10.0)
    parser.add_argument("--tail-s", type=float, default=10.0)
    parser.add_argument("--topic", default="/lqr_tracker/position_error")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    report = {"created_at": datetime.now().isoformat(),
              "topic": args.topic, "gui": bool(args.gui),
              "startup_exclude_s": args.startup_exclude_s,
              "tail_s": args.tail_s,
              "window_boundary_rule": WINDOW_RULE,
              "trajectory_matrix": {k: {"extra_args": v[0],
                                        "duration_s": v[1]}
                                    for k, v in TRAJECTORIES.items()},
              "trajectories": {}}

    for name, (extra, dur) in TRAJECTORIES.items():
        print("[N5] 轨迹 %s ..." % name, flush=True)
        sim_args = ["bash", START_SIM, "lqr", "--altitude", "3.0",
                    "--trajectory", name,
                    "--gui" if args.gui else "--headless"] + extra
        sim = subprocess.Popen(
            sim_args, cwd=WS, env=_env(), stdout=open(
                os.path.join(args.out_dir, "sim_%s.log" % name), "w"),
            stderr=subprocess.STDOUT, start_new_session=True)
        entry = {"sim_command": " ".join(sim_args), "duration_s": dur}
        try:
            if not wait_ready(args.topic):
                entry["error"] = "topic_not_ready"
            else:
                time.sleep(8)  # 让轨迹越过起步瞬态
                samples = record_error_ts(args.topic, dur)
                entry["windows"] = window_stats(
                    samples, args.startup_exclude_s, args.tail_s)
                entry["segment_change"] = segment_change_note(
                    samples, args.startup_exclude_s)
                if entry["windows"]["full"].get("n", 0) <= 0:
                    entry["error"] = "no_samples"
                f = entry["windows"]["post_startup"]
                print("  %s: post_startup n=%d mean=%.4f rms=%.4f max=%.4f"
                      % (name, f.get("n", 0), f.get("mean_m", 0),
                         f.get("rms_m", 0), f.get("max_m", 0)), flush=True)
        finally:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            stop_sim()
        report["trajectories"][name] = entry

    out = os.path.join(args.out_dir, "n5_lqr_window_report.json")
    with open(out, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[N5] 报告: %s" % out)


if __name__ == "__main__":
    main()