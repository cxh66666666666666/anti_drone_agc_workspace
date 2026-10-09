#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""a0_freeze.py —— 冻结/校验阶段3 A0「观测模型与真值集」（离线工具，不依赖 ROS）。

用法：
    python3 a0_freeze.py            # 冻结（写出 experiments/a0/protocol_a0_hashes.json）
    python3 a0_freeze.py --check    # 复核：被冻结对象是否与冻结时一致（改动即 FAIL，非零码退出）

冻结对象（任一改动，--check 即 FAIL）：
  - 观测模型实现与配置：perception_sensors.yaml / agc_params.yaml / air_perception.cpp
  - 目标驱动与真值集生成：intruder_driver.cpp / d1_scenario_gen.py / a0_freeze.py
  - 核验脚本 verify_a0_perception.py（若尚不存在，则跳过并在归档中标注 pending）
  - A0 manifest 及其场景/感知/world 文件（自 manifest_a0.json 解析）

风格对齐 d0_freeze.py，但完全不触碰 eval_protocol_v1 与 experiments/d/protocol_v1_hashes.json。
"""

import argparse
import hashlib
import json
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
OUT_DIR = os.path.join(WORKSPACE, "experiments", "a0")
HASHES_PATH = os.path.join(OUT_DIR, "protocol_a0_hashes.json")
A0_MANIFEST = os.path.join(WORKSPACE, "experiments", "a0", "manifest_a0.json")

CREATED_FROM = "src/air_ground_core/scripts/a0_freeze.py@stage3-a0"

# 固定冻结对象（相对 workspace 路径）
SRC_FILES = [
    "src/air_ground_core/config/perception_sensors.yaml",
    "src/air_ground_core/config/agc_params.yaml",
    "src/air_ground_core/src/air_perception.cpp",
    "src/air_ground_core/src/intruder_driver.cpp",
    "src/air_ground_core/scripts/d1_scenario_gen.py",
    "src/air_ground_core/scripts/a0_freeze.py",
    "src/air_ground_core/scripts/verify_a0_perception.py",  # 可缺（pending）
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


def _a0_manifest_files():
    """从 A0 manifest 解析其引用的场景/感知/world 文件（相对 workspace 路径）。"""
    if not os.path.isfile(A0_MANIFEST):
        return []
    with open(A0_MANIFEST, encoding="utf-8") as f:
        man = json.load(f)
    rels = []
    for e in man.get("entries", []):
        for key in ("scenario_file", "sensors_file", "world_file"):
            if e.get(key):
                rels.append(e[key])
    return rels


def frozen_targets():
    """全部冻结对象（相对路径，去重且保序）。"""
    rels = list(SRC_FILES)
    rels.append("experiments/a0/manifest_a0.json")
    rels += _a0_manifest_files()
    seen, out = set(), []
    for r in rels:
        if r not in seen:
            seen.add(r)
            out.append(r)
    return out


def snapshot():
    """对全部冻结对象取哈希快照；返回 (files, pending)。"""
    files, pending = {}, {}
    for rel in frozen_targets():
        h = sha256_file(os.path.join(WORKSPACE, rel))
        files[rel] = h
        if h is None:
            pending[rel] = True
    blob = json.dumps(files, sort_keys=True, separators=(",", ":"))
    return files, pending, hashlib.sha256(blob.encode("utf-8")).hexdigest()


def cmd_freeze():
    os.makedirs(OUT_DIR, exist_ok=True)
    files, pending, combined = snapshot()
    out = {
        "protocol": "a0_observation_model_v1",
        "created_from": CREATED_FROM,
        "frozen_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "combined_hash": combined,
        "files": files,
        "pending": pending,
    }
    with open(HASHES_PATH, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("=== A0 冻结 ===")
    print("hashes 写入: %s" % HASHES_PATH)
    print("combined_hash: %s" % combined)
    for rel, h in files.items():
        print("  %-72s %s" % (rel, h if h else "PENDING(missing)"))
    if pending:
        print("NOTE: %d 项不存在，已标注 pending: %s" % (len(pending), sorted(pending)))
    print("PASS: A0 冻结对象已留哈希")
    return 0


def cmd_check():
    if not os.path.isfile(HASHES_PATH):
        print("FAIL: 未找到冻结文件 %s，请先执行冻结" % HASHES_PATH)
        return 1
    with open(HASHES_PATH) as f:
        frozen = json.load(f)
    files, pending, combined = snapshot()
    old_files = frozen.get("files") or {}
    old_pending = frozen.get("pending") or {}
    print("=== A0 复核 ===")
    print("冻结 combined_hash: %s" % frozen.get("combined_hash"))
    print("当前 combined_hash: %s" % combined)
    ok = True
    for rel in sorted(set(old_files) | set(files)):
        old, new = old_files.get(rel), files.get(rel)
        if old is None and new is None:
            print("  PENDING %-72s (仍未出现)" % rel)
        elif old == new:
            print("  PASS %-72s %s" % (rel, new))
        else:
            ok = False
            print("  FAIL %-72s frozen=%s current=%s" % (rel, old, new))
    # 新增的 pending 项在冻结后出现，需重新冻结
    for rel in pending:
        if rel not in old_pending:
            ok = False
            print("  FAIL %s 冻结时不存在、现为 pending 之外（需重新冻结）" % rel)
    if ok and frozen.get("combined_hash") == combined:
        print("PASS: A0 冻结对象未改动，协议可追溯")
        return 0
    print("FAIL: A0 冻结对象已改动，需重新冻结（不得就地修改）")
    return 1


def main():
    ap = argparse.ArgumentParser(description="冻结/校验阶段3 A0 观测模型与真值集")
    ap.add_argument("--check", action="store_true", help="复核冻结对象是否与冻结时一致")
    args = ap.parse_args()
    sys.exit(cmd_check() if args.check else cmd_freeze())


if __name__ == "__main__":
    main()