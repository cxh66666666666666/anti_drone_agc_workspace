#!/usr/bin/env python3
"""m4_inject.py —— M4 感知故障在线注入（编排/离线工具，非实时链路）。

按 rosparam 在线切换 /agc/perception/fault/<sensor>/{enabled,drop_prob,latency_s,outlier_bias_m}，
并把注入事件以 JSONL 落盘，供离线核验与复盘。实时链路节点一律 C++；本工具只改参数。

用法：
  # 单次施加（编排脚本在 attempt 内调用）
  m4_inject.py apply --mode latency  --sensor air_ir_top --value 0.6 --record <dir>
  m4_inject.py apply --mode loss     --sensor all        --value 1.0 --record <dir>
  m4_inject.py apply --mode conflict --sensor air_ir_top --value 3.0 --record <dir>
  m4_inject.py apply --mode clear    --sensor all                 --record <dir>

  # 按时序施加（相对当前时刻，秒）
  m4_inject.py timeline --sensor air_ir_top --record <dir> --steps "0:clear,8:conflict:3.0,25:clear"

选项：
  --record <dir>  注入事件写入 <dir>/injections.jsonl（不指定则只打印）
  --tag <name>    给本次注入事件加标签（便于区分不同批次/场景）
"""
import argparse
import json
import os
import subprocess
import sys
import time

# 与 config/perception_sensors.yaml 的通道列表保持一致
SENSORS = ["air_eo_front", "air_ir_top"]


def rosparam_set(name, value):
    """用 rosparam 命令行设置参数：不依赖 rospy Python 环境，失败不抛出。"""
    subprocess.run(["rosparam", "set", name, value],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def sensors_of(arg):
    if arg in (None, "", "all"):
        return list(SENSORS)
    return [s for s in arg.split(",") if s]


def apply_mode(mode, sensors, value):
    """施加一种故障模式，返回事件记录。"""
    for s in sensors:
        base = "/agc/perception/fault/" + s
        if mode == "clear":
            rosparam_set(base + "/enabled", "false")
            rosparam_set(base + "/drop_prob", "0.0")
            rosparam_set(base + "/latency_s", "0.0")
            rosparam_set(base + "/outlier_bias_m", "0.0")
        elif mode == "latency":
            rosparam_set(base + "/enabled", "true")
            rosparam_set(base + "/drop_prob", "0.0")
            rosparam_set(base + "/latency_s", str(value))
            rosparam_set(base + "/outlier_bias_m", "0.0")
        elif mode == "loss":
            rosparam_set(base + "/enabled", "true")
            rosparam_set(base + "/drop_prob", str(value))
            rosparam_set(base + "/latency_s", "0.0")
            rosparam_set(base + "/outlier_bias_m", "0.0")
        elif mode == "conflict":
            rosparam_set(base + "/enabled", "true")
            rosparam_set(base + "/drop_prob", "0.0")
            rosparam_set(base + "/latency_s", "0.0")
            rosparam_set(base + "/outlier_bias_m", str(value))
        else:
            raise SystemExit("未知故障模式: " + mode)
    return {"type": "injection", "mode": mode, "sensors": sensors,
            "value": value, "t_wall": time.time()}


def record(rec, record_dir, tag, extra=None):
    if extra:
        rec.update(extra)
    if tag:
        rec["tag"] = tag
    if record_dir:
        os.makedirs(record_dir, exist_ok=True)
        with open(os.path.join(record_dir, "injections.jsonl"), "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    print(json.dumps(rec, ensure_ascii=False))


def parse_steps(steps):
    """steps 形如 '0:clear,8:conflict:3.0' -> [(t, mode, value), ...]"""
    out = []
    for item in steps.split(","):
        parts = item.split(":")
        if len(parts) < 2:
            raise SystemExit("非法 step: " + item)
        t = float(parts[0])
        mode = parts[1]
        value = float(parts[2]) if len(parts) >= 3 else None
        out.append((t, mode, value))
    return out


def main():
    ap = argparse.ArgumentParser(description="M4 感知故障在线注入工具")
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("apply", help="单次施加一种故障模式")
    a.add_argument("--mode", required=True,
                   choices=["clear", "latency", "loss", "conflict"])
    a.add_argument("--sensor", default="all", help="通道 id、逗号分隔列表或 all")
    a.add_argument("--value", type=float, default=None,
                   help="latency_s / drop_prob / outlier_bias_m；conflict 默认 3.0，loss 默认 1.0")
    a.add_argument("--record", default="")
    a.add_argument("--tag", default="")

    t = sub.add_parser("timeline", help="按时序施加（相对当前时刻，秒）")
    t.add_argument("--sensor", default="all")
    t.add_argument("--steps", required=True)
    t.add_argument("--record", default="")
    t.add_argument("--tag", default="")

    args = ap.parse_args()
    sensors = sensors_of(args.sensor)

    if args.cmd == "apply":
        value = args.value
        if value is None:
            value = 3.0 if args.mode == "conflict" else (1.0 if args.mode == "loss" else 0.0)
        rec = apply_mode(args.mode, sensors, value)
        record(rec, args.record, args.tag)
        return 0

    # timeline
    steps = parse_steps(args.steps)
    t0 = time.time()
    for (rel, mode, value) in steps:
        wait = t0 + rel - time.time()
        if wait > 0:
            time.sleep(wait)
        if value is None:
            value = 3.0 if mode == "conflict" else (1.0 if mode == "loss" else 0.0)
        rec = apply_mode(mode, sensors, value)
        record(rec, args.record, args.tag, extra={"t_rel_s": rel})
    return 0


if __name__ == "__main__":
    sys.exit(main())