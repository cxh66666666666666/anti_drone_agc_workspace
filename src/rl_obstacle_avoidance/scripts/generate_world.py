#!/usr/bin/env python3
"""
@file: generate_world.py
@brief: 从 YAML 配置文件生成 Gazebo SDF 1.6 world 文件

支持 box 和 cylinder 两种障碍物类型。
"""

import argparse
import os
import sys

import yaml


def _build_box_model(name, pose, size):
    """构建 box 类型障碍物的 SDF model XML 片段。"""
    return f"""  <!-- 障碍物: {name} -->
  <model name="{name}">
    <static>true</static>
    <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose>
    <link name="obstacle_link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>"""


def _build_cylinder_model(name, pose, radius, height):
    """构建 cylinder 类型障碍物的 SDF model XML 片段。"""
    return f"""  <!-- 障碍物: {name} -->
  <model name="{name}">
    <static>true</static>
    <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose>
    <link name="obstacle_link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>"""


def generate_world(config, output_path):
    """读取配置字典，生成 Gazebo world 文件并写入 output_path。

    Args:
        config: 解析后的 YAML 配置字典
        output_path: 输出 world 文件的绝对路径
    """
    world_name = config["world"]["name"]

    # 逐个构建障碍物模型
    model_parts = []
    for obs in config.get("obstacles", []):
        obs_type = obs["type"]
        if obs_type == "box":
            model_parts.append(_build_box_model(obs["name"], obs["pose"], obs["size"]))
        elif obs_type == "cylinder":
            model_parts.append(
                _build_cylinder_model(obs["name"], obs["pose"], obs["radius"], obs["height"])
            )
        else:
            print(f"警告: 未知障碍物类型 '{obs_type}'，跳过 '{obs['name']}'")

    # 组装完整 SDF
    world_xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <!-- 由 generate_world.py 自动生成，场景: {world_name} -->
{chr(10).join(model_parts)}
</sdf>
"""

    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(world_xml)

    print(f"已生成 world 文件: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="从 YAML 配置生成 Gazebo world 文件（SDF 1.6）"
    )
    parser.add_argument(
        "--config", required=True,
        help="YAML 配置文件路径"
    )
    parser.add_argument(
        "--output", default=None,
        help="输出 world 文件路径（可选，默认根据 world.name 生成到 worlds/ 目录）"
    )
    args = parser.parse_args()

    # 读取 YAML 配置
    if not os.path.isfile(args.config):
        print(f"错误: 配置文件不存在: {args.config}")
        sys.exit(1)

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # 确定输出路径
    if args.output:
        output = args.output
    else:
        # 默认输出到脚本所在包目录下的 worlds/ 子目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        worlds_dir = os.path.join(script_dir, "..", "worlds")
        output = os.path.join(worlds_dir, f"{cfg['world']['name']}.world")

    generate_world(cfg, os.path.abspath(output))