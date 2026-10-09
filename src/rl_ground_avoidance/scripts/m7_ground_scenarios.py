#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 地面车固定 train/val/test 场景集（纯数据，无 ROS 依赖）。

说明（诚实口径）：地面单体使用唯一 `worlds/ground.world`，静态障碍场固定
（pole_1(6,0)/pole_2(12,0.5)/pole_3(18,-0.3)），且 `ground_env` 的 `poles`
为节点私有参数、无法从 launch 注入。因此地面固定集只对**初始位姿与参考速度**
划分 train/val/test，障碍场不变；轨迹/净空/失败分母的统计仍按整回合口径。
"""

import hashlib
import json

# 与 ground.world / ground_env 默认一致
GROUND_POLES = [[6.0, 0.0], [12.0, 0.5], [18.0, -0.3]]


def _scn(sid, start_x, start_y, yaw=0.0, speed=1.0):
    return {"scenario_id": sid, "start_x": start_x, "start_y": start_y,
            "start_yaw": yaw, "reference_speed": speed}


def build_splits():
    train = []
    for sx in (-1.5, -3.0):
        for sy in (-0.8, -0.4, 0.0, 0.4, 0.8):
            train.append(_scn("gm7-t-x%dt-y%dt" % (int(-sx * 10), int(sy * 10)),
                              sx, sy))
    validation = [
        _scn("gm7-v-x20-y10", -2.0, 1.0),
        _scn("gm7-v-x20-ym10", -2.0, -1.0),
        _scn("gm7-v-x25-y10", -2.5, 1.0),
        _scn("gm7-v-x25-ym10", -2.5, -1.0),
    ]
    test = [
        _scn("gm7-s-x18-y13", -1.8, 1.3),
        _scn("gm7-s-x18-ym13", -1.8, -1.3),
        _scn("gm7-s-x22-y13", -2.2, 1.3),
        _scn("gm7-s-x22-ym13", -2.2, -1.3),
        _scn("gm7-s-x32-y13", -3.2, 1.3),
        _scn("gm7-s-x32-ym13", -3.2, -1.3),
    ]
    return {"train": train, "validation": validation, "test": test}


def split_sha256(splits=None):
    splits = splits or build_splits()
    blob = json.dumps(splits, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def apply_scenario(env, scenario):
    """把场景初始条件写入 GroundEnv（reset 前调用）。"""
    env.start_x = float(scenario["start_x"])
    env.start_y = float(scenario["start_y"])
    env.start_yaw = float(scenario.get("start_yaw", 0.0))
    env.reference_speed = float(scenario.get("reference_speed", 1.0))


if __name__ == "__main__":
    splits = build_splits()
    print(json.dumps({k: len(v) for k, v in splits.items()}))
    print("sha256:", split_sha256())