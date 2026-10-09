#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5 地面车障碍布局 world 生成器（纯 Python，无 ROS 依赖）。

以 `worlds/ground.world` 为模板：保留 sun/ground_plane/ground_car 等脚手架，
仅把 pole_* 模型替换为目标布局的杆体（box 0.3×0.3×3，位姿 (x, y, 1.5)）。

注意（世界覆盖能力）：当前 `launch/ground_sim.launch` 将 world 路径硬编码为
`worlds/ground.world`，`start_ground_sim.sh` 也未透传 world 参数，**不支持从
命令行覆盖 world**。因此本生成器产出的 world 仅用于 Gazebo 可视化/摆放核对；
RL 实际感知的障碍场由 `n5_ground_scenarios.apply_scenario` 注入 `env.poles`
（虚拟激光解析），二者杆位保持一致。

用法:
  python3 src/rl_ground_avoidance/scripts/n5_ground_world_gen.py --layout field_d
  python3 src/rl_ground_avoidance/scripts/n5_ground_world_gen.py --all
"""

import argparse
import os
import sys

WS = "/home/ubuntu/anti_drone_agc_workspace"
SCRIPTS = os.path.join(WS, "src/rl_ground_avoidance/scripts")
TEMPLATE = os.path.join(WS, "src/rl_ground_avoidance/worlds/ground.world")
DEFAULT_OUT_DIR = os.path.join(WS, "src/rl_ground_avoidance/worlds/n5")

sys.path.insert(0, SCRIPTS)
from n5_ground_scenarios import LAYOUTS  # noqa: E402

# 杆体配色循环（沿用模板红/橙/黄风格）
COLORS = [
    ("1 0 0 1", "1 0 0 1"),
    ("1 0.5 0 1", "1 0.5 0 1"),
    ("1 1 0 1", "1 1 0 1"),
    ("0 0.6 1 1", "0 0.6 1 1"),
    ("0.6 0.2 1 1", "0.6 0.2 1 1"),
]


def _pole_model(index, x, y):
    """生成单个杆体 SDF 片段（0.3×0.3×3，中心高 1.5）。"""
    ambient, diffuse = COLORS[index % len(COLORS)]
    return (
        '    <model name="pole_%d">\n'
        '      <static>true</static>\n'
        '      <pose>%s %s 1.5 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        '          <geometry><box><size>0.3 0.3 3</size></box></geometry>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry><box><size>0.3 0.3 3</size></box></geometry>\n'
        '          <material><ambient>%s</ambient><diffuse>%s</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n' % (index, x, y, ambient, diffuse)
    )


def _split_scaffolding(template):
    """拆分模板为 (脚手架头, 脚手架尾)，剔除原有 pole_* 模型。"""
    first = template.index('<model name="pole_')
    last_end = template.rfind('</model>') + len('</model>')
    head = template[:first]
    head = head.rstrip(" \t")  # 去掉首个 pole 模型前的残留缩进
    head += "\n"
    tail = template[last_end:]
    if not tail.endswith("\n"):
        tail += "\n"
    return head, tail


def build_world(layout):
    """按布局生成完整 world 文本。"""
    with open(TEMPLATE, "r", encoding="utf-8") as stream:
        template = stream.read()
    head, tail = _split_scaffolding(template)
    poles = LAYOUTS[layout]
    body = "".join(_pole_model(i + 1, p[0], p[1]) for i, p in enumerate(poles))
    return head + body + tail


def write_layout(layout, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, layout + ".world")
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(build_world(layout))
    return path


def parse_args():
    p = argparse.ArgumentParser(description="N5 地面布局 world 生成")
    p.add_argument("--layout", help="生成单个布局，如 field_d")
    p.add_argument("--all", action="store_true", help="生成全部布局")
    p.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    return p.parse_args()


def main():
    args = parse_args()
    targets = list(LAYOUTS) if args.all else ([args.layout] if args.layout else [])
    if not targets:
        raise SystemExit("请指定 --all 或 --layout <name>")
    for name in targets:
        if name not in LAYOUTS:
            raise SystemExit("未知布局: %s（可选 %s）" % (name, sorted(LAYOUTS)))
        path = write_layout(name, args.out_dir)
        print("[N5] 已生成 %s（%d 杆）" % (path, len(LAYOUTS[name])))


if __name__ == "__main__":
    main()