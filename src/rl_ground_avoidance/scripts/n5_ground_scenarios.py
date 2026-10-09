#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5 地面车多障碍布局 train/val/test 场景集（纯数据，无 ROS 依赖）。

相对 M7 的改进：M7 只变化初始位姿/参考速度、障碍场固定。N5 定义多个命名布局
并按**布局隔离**划分 train/val/test：test 使用训练与验证均未出现的布局，
以衡量对未见障碍布局的泛化能力。

布局注入方式：`GroundEnv.poles` 是运行时属性（虚拟激光/净空在其上解析计算），
`apply_scenario` 会直接把 `scn["poles"]` 写入 `env.poles`，从而彻底改变感知到的
障碍场，无需改动 world 文件或 launch。
"""

import hashlib
import json

# 命名布局：每项为 [[x, y], ...] 杆位（与 ground.world 中 0.3×0.3×3 杆体对应）
LAYOUTS = {
    # 默认三杆（= ground.world / ground_env 默认），train
    "field_a": [[6.0, 0.0], [12.0, 0.5], [18.0, -0.3]],
    # 交错散布，train
    "field_b": [[5.0, -0.7], [9.0, 0.6], [13.0, -0.6], [17.0, 0.7]],
    # 窄门：x=8 处左右夹杆，门宽 1.8m；validation
    "field_c": [[8.0, 0.9], [8.0, -0.9], [16.0, 0.0], [22.0, 0.4]],
    # 密集散布：五杆交替；test（训练未见）
    "field_d": [[5.0, 0.5], [9.0, -0.5], [13.0, 0.5], [17.0, -0.5], [21.0, 0.5]],
    # 双窄门：x=7 与 x=14 各一道门；test（训练未见）
    "field_e": [[7.0, 1.0], [7.0, -1.0], [14.0, 0.8], [14.0, -0.8]],
}

# 布局 → split 归属（每个布局只属于一个 split）
LAYOUT_SPLIT = {
    "field_a": "train",
    "field_b": "train",
    "field_c": "validation",
    "field_d": "test",
    "field_e": "test",
}


def _scn(layout, sid, start_x, start_y, yaw=0.0, speed=1.0):
    """构造单个场景（含所属布局的杆位）。"""
    return {
        "scenario_id": sid,
        "layout": layout,
        "poles": [list(p) for p in LAYOUTS[layout]],
        "start_x": float(start_x),
        "start_y": float(start_y),
        "start_yaw": float(yaw),
        "reference_speed": float(speed),
    }


def build_splits():
    """返回 {train, validation, test}；各 split 内布局互不重叠。"""
    train = []
    # train：field_a + field_b 各 4 个场景（变初始位姿/速度）
    for sx, sy in ((-1.5, -0.5), (-1.5, 0.5), (-3.0, -0.5), (-3.0, 0.5)):
        train.append(_scn("field_a", "gn5-t-a-x%dy%d" % (int(-sx * 10), int(sy * 10)),
                          sx, sy))
    for sx, sy, yaw in ((-2.0, -0.6, 0.0), (-2.0, 0.6, 0.0),
                        (-3.5, -0.6, 0.1), (-3.5, 0.6, -0.1)):
        train.append(_scn("field_b", "gn5-t-b-x%dy%d" % (int(-sx * 10), int(sy * 10)),
                          sx, sy, yaw))

    # validation：仅 field_c（窄门）
    validation = [
        _scn("field_c", "gn5-v-c-x20-y8", -2.0, 0.8),
        _scn("field_c", "gn5-v-c-x20-ym8", -2.0, -0.8),
        _scn("field_c", "gn5-v-c-x30-y10", -3.0, 1.0, 0.05),
        _scn("field_c", "gn5-v-c-x30-ym10", -3.0, -1.0, -0.05),
    ]

    # test：field_d（密集散布）+ field_e（双窄门），均为训练未见布局
    test = [
        _scn("field_d", "gn5-s-d-x18-y9", -1.8, 0.9),
        _scn("field_d", "gn5-s-d-x18-ym9", -1.8, -0.9),
        _scn("field_d", "gn5-s-d-x28-y6", -2.8, 0.6, 0.08),
        _scn("field_e", "gn5-s-e-x22-y11", -2.2, 1.1),
        _scn("field_e", "gn5-s-e-x22-ym11", -2.2, -1.1),
        _scn("field_e", "gn5-s-e-x32-y7", -3.2, 0.7, -0.06),
    ]
    return {"train": train, "validation": validation, "test": test}


def layout_of(split_name):
    """返回指定 split 使用的布局名集合（排序）。"""
    return sorted({s["layout"] for s in build_splits()[split_name]})


def split_sha256(splits=None):
    """整个场景集（含布局与杆位）的稳定指纹。"""
    splits = splits or build_splits()
    blob = json.dumps(splits, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def apply_scenario(env, scenario):
    """把场景写入 GroundEnv（reset 前调用）；含**布局注入** env.poles。"""
    env.start_x = float(scenario["start_x"])
    env.start_y = float(scenario["start_y"])
    env.start_yaw = float(scenario.get("start_yaw", 0.0))
    env.reference_speed = float(scenario.get("reference_speed", 1.0))
    # 关键：直接替换虚拟障碍布局
    env.poles = [tuple(p) for p in scenario["poles"]]


def assert_layout_isolation(splits=None):
    """校验 test 布局与 train/val 完全不相交（训练-测试按布局隔离）。"""
    splits = splits or build_splits()
    tr = {s["layout"] for s in splits["train"]}
    va = {s["layout"] for s in splits["validation"]}
    te = {s["layout"] for s in splits["test"]}
    assert not (te & tr), "test 与 train 布局重叠: %s" % (te & tr)
    assert not (te & va), "test 与 validation 布局重叠: %s" % (te & va)
    assert not (tr & va), "train 与 validation 布局重叠: %s" % (tr & va)
    return True


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="N5 地面场景集摘要")
    parser.add_argument("--print-summary", action="store_true")
    args = parser.parse_args()

    splits = build_splits()
    assert_layout_isolation(splits)
    summary = {
        "layouts": {name: LAYOUT_SPLIT[name] for name in LAYOUTS},
        "layout_of": {k: layout_of(k) for k in splits},
        "split_sizes": {k: len(v) for k, v in splits.items()},
        "scenario_split_sha256": split_sha256(splits),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.print_summary:
        print("[N5] 布局隔离校验通过：test 布局 %s 未出现在 train/validation"
              % layout_of("test"))