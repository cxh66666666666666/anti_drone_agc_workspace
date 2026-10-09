#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""d0_freeze.py —— 冻结/校验阶段3 D 评估协议（离线工具，不依赖 ROS）。

用法：
    python3 d0_freeze.py            # 冻结（写出 experiments/d/protocol_v1_hashes.json）
    python3 d0_freeze.py --check    # 复核：被冻结文件是否与冻结时一致（改动即 FAIL）

冻结对象（任一被改动，--check 即 FAIL，用于保证“先锁协议，后跑数据”）：
  - eval_protocol_v1.yaml / eval_manifest.schema.json
  - agc_params.yaml / perception_sensors.yaml
  - scenario_flat.yaml / scenario_linear_target.yaml / scenario_truly_blocked.yaml
  - agc_collab.launch / start_agc_collab.sh / summarize_runs.py
"""

import argparse
import hashlib
import json
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
CONFIG = os.path.join(WORKSPACE, "src", "air_ground_core", "config")
LAUNCH = os.path.join(WORKSPACE, "src", "air_ground_core", "launch")
OUT_DIR = os.path.join(WORKSPACE, "experiments", "d")
HASHES_PATH = os.path.join(OUT_DIR, "protocol_v1_hashes.json")

# 冻结对象（相对 workspace 的路径）
FROZEN_FILES = [
    "src/air_ground_core/config/eval_protocol_v1.yaml",
    "src/air_ground_core/config/eval_manifest.schema.json",
    "src/air_ground_core/config/agc_params.yaml",
    "src/air_ground_core/config/perception_sensors.yaml",
    "src/air_ground_core/config/scenario_flat.yaml",
    "src/air_ground_core/config/scenario_linear_target.yaml",
    "src/air_ground_core/config/scenario_truly_blocked.yaml",
    "src/air_ground_core/worlds/agc_world.world",
    "src/air_ground_core/launch/agc_collab.launch",
    "scripts/start_agc_collab.sh",
    "src/air_ground_core/scripts/summarize_runs.py",
    "src/air_ground_core/scripts/d1_scenario_gen.py",
    "src/air_ground_core/scripts/d1_batch_runner.py",
    "src/air_ground_core/worlds/agc_world_dynamic.world",
    "src/air_ground_core/src/agc_run_logger.cpp",
    "src/air_ground_core/src/scenario_reset.cpp",
    "src/air_ground_core/src/map_adapter.cpp",
    "src/air_ground_core/src/task_manager.cpp",
    "src/air_ground_core/src/intruder_driver.cpp",
    "src/air_ground_core/src/air_perception.cpp",
]


def sha256_file(path):
    """返回文件 sha256；文件不存在返回 None。"""
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def snapshot():
    """对全部冻结对象取哈希快照。"""
    files = {}
    for rel in FROZEN_FILES:
        files[rel] = sha256_file(os.path.join(WORKSPACE, rel))
    blob = json.dumps(files, sort_keys=True, separators=(",", ":"))
    return {"files": files, "combined_hash": hashlib.sha256(blob.encode("utf-8")).hexdigest()}


def cmd_freeze():
    os.makedirs(OUT_DIR, exist_ok=True)
    snap = snapshot()
    out = {
        "protocol": "eval_protocol_v1",
        "frozen_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "combined_hash": snap["combined_hash"],
        "files": snap["files"],
    }
    with open(HASHES_PATH, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    missing = [k for k, v in snap["files"].items() if v is None]
    print("=== D0 冻结 ===")
    print("hashes 写入: %s" % HASHES_PATH)
    print("combined_hash: %s" % snap["combined_hash"])
    for k, v in snap["files"].items():
        print("  %-64s %s" % (k, v if v else "MISSING"))
    if missing:
        print("FAIL: 有冻结对象缺失: %s" % missing)
        return 1
    print("PASS: 全部冻结对象存在并已留哈希")
    return 0


def cmd_check():
    if not os.path.isfile(HASHES_PATH):
        print("FAIL: 未找到冻结文件 %s，请先执行冻结" % HASHES_PATH)
        return 1
    with open(HASHES_PATH) as f:
        frozen = json.load(f)
    cur = snapshot()
    print("=== D0 复核 ===")
    print("冻结 combined_hash: %s" % frozen.get("combined_hash"))
    print("当前 combined_hash: %s" % cur["combined_hash"])
    ok = True
    for rel in FROZEN_FILES:
        old = (frozen.get("files") or {}).get(rel)
        new = cur["files"].get(rel)
        if old == new:
            print("  PASS %-64s %s" % (rel, new))
        else:
            ok = False
            print("  FAIL %-64s frozen=%s current=%s" % (rel, old, new))
    if ok and frozen.get("combined_hash") == cur["combined_hash"]:
        print("PASS: 冻结对象未改动，协议可追溯")
        return 0
    print("FAIL: 冻结对象已改动，协议版本需另起（不得就地修改）")
    return 1


def main():
    ap = argparse.ArgumentParser(description="冻结/校验阶段3 D 评估协议")
    ap.add_argument("--check", action="store_true", help="复核冻结对象是否与冻结时一致")
    args = ap.parse_args()
    sys.exit(cmd_check() if args.check else cmd_freeze())


if __name__ == "__main__":
    main()
