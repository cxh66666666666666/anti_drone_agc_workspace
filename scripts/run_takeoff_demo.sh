#!/bin/bash
# drone_launch_controller 一键起飞演示
# 用法: ./scripts/run_takeoff_demo.sh [altitude]
#       默认高度 5.0m
set -e

ALTITUDE=${1:-5.0}
TIMEOUT=30.0

echo "========================================="
echo " drone_launch_controller 起飞演示"
echo " 目标高度: ${ALTITUDE}m, 超时: ${TIMEOUT}s"
echo "========================================="

source /opt/ros/noetic/setup.bash
source ~/anti_drone_agc_workspace/devel/setup.bash

# 1. 清理旧进程
echo "[1/5] 清理旧进程..."
pkill -f px4 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
sleep 2

# 2. 启动 PX4 SITL + MAVROS（后台）
echo "[2/5] 启动 PX4 SITL + MAVROS..."
cd /home/ubuntu/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/PX4_Firmware:/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH

roslaunch px4 mavros_posix_sitl.launch vehicle:=iris &
PX4_PID=$!
echo "PX4 PID: $PX4_PID"

# 3. 等待 MAVROS 连接
echo "[3/5] 等待 MAVROS 连接 (最多 60s)..."
cd ~/anti_drone_agc_workspace
source devel/setup.bash
for i in $(seq 1 60); do
    CONN=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "connected:" | awk '{print $2}')
    if [ "$CONN" = "True" ]; then
        echo "MAVROS 已连接 ($(($i))s)"
        break
    fi
    sleep 1
done

if [ "$CONN" != "True" ]; then
    echo "[警告] MAVROS 未在 60s 内连接，尝试继续..."
fi

# 4. 启动 drone_launch_controller
echo "[4/5] 启动 drone_launch_controller..."
roslaunch drone_launch_controller drone_controller.launch verbose:=true &
CTRL_PID=$!
sleep 3

# 5. 执行起飞
echo "[5/5] 执行起飞到 ${ALTITUDE}m..."
rosservice call /drone_launch_controller/takeoff "altitude: ${ALTITUDE}" "timeout: ${TIMEOUT}"

echo ""
echo "========================================="
echo " 起飞命令已发送"
echo "========================================="
echo ""
echo "监控命令:"
echo "  rostopic echo /drone_launch_controller/flight_status"
echo "  rostopic echo /mavros/local_position/pose"
echo ""
echo "按 Ctrl+C 停止所有进程"
echo "========================================="

# 等待 Ctrl+C
trap "kill $CTRL_PID $PX4_PID 2>/dev/null; exit 0" SIGINT SIGTERM
wait
