#!/bin/bash
# 地面车避障仿真启动脚本（纯地面，不需要 PX4/MAVROS/LQR 堆栈）
# 用法: ./scripts/start_ground_sim.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"

echo "[清理] 清理旧地面车进程..."
pkill -9 -f ground_gazebo 2>/dev/null || true
pkill -9 -f ground_perception 2>/dev/null || true
pkill -9 -f ground_env 2>/dev/null || true
pkill -9 -f train_ground_dqn 2>/dev/null || true
pkill -9 -f evaluate_ground_policy 2>/dev/null || true
sleep 2

echo "[启动] 启动地面车 Gazebo 仿真 + 感知节点..."
roslaunch rl_ground_avoidance ground_sim.launch "$@"

echo "[清理] 已停止"
