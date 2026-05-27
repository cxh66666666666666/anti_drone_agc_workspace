#!/bin/bash
# XTDrone 起飞测试 - 启动脚本
# 使用方法: ./start_drone_test.sh

echo "========================================="
echo "XTDrone 起飞测试启动脚本"
echo "========================================="

# 设置环境
source /opt/ros/noetic/setup.bash
source ~/anti_drone_agc_workspace/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/PX4_Firmware:/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH

# 清理旧进程
echo "[1/4] 清理旧进程..."
pkill -9 -f px4 2>/dev/null
pkill -9 -f gazebo 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f mavros 2>/dev/null
pkill -9 -f drone_launch_controller 2>/dev/null
sleep 2

# 启动 PX4 SITL + Gazebo + MAVROS
echo "[2/4] 启动 PX4 SITL + Gazebo + MAVROS..."
echo "请在弹出的终端中查看启动状态"
gnome-terminal --tab --title="PX4 SITL + MAVROS" -- bash -c "
    cd /home/ubuntu/PX4_Firmware
    source /opt/ros/noetic/setup.bash
    source ~/anti_drone_agc_workspace/devel/setup.bash
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/home/ubuntu/PX4_Firmware:/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo
    export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:\$GAZEBO_MODEL_PATH
    roslaunch px4 mavros_posix_sitl.launch vehicle:=iris fcu_url:=udp://:24540@localhost:34580
    exec bash
" &
PX4_TAB_PID=$!
echo "PX4 SITL + MAVROS 终端已打开"

# 等待仿真启动
echo "[3/4] 等待仿真启动..."
echo "等待 20 秒让 PX4 和 Gazebo 完全启动..."
sleep 20

# 检查 PX4 是否运行
if ! pgrep -f "px4.*sitl" > /dev/null; then
    echo "[错误] PX4 SITL 未启动成功！"
    echo "请检查 PX4 终端中的错误信息"
    exit 1
fi
echo "PX4 SITL 运行正常"

# 等待 MAVROS 连接
echo "等待 MAVROS 连接..."
for i in {1..30}; do
    CONN_STATUS=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "connected:" | awk '{print $2}')
    if [ "$CONN_STATUS" = "True" ]; then
        echo "MAVROS 连接成功！"
        break
    fi
    echo "等待 MAVROS 连接... ($i/30)"
    sleep 1
done

if [ "$CONN_STATUS" != "True" ]; then
    echo "[警告] MAVROS 可能未连接，继续尝试..."
fi

# 启动 drone_launch_controller
echo "[4/4] 启动 drone_launch_controller..."
gnome-terminal --tab --title="Controller" -- bash -c "
    source /opt/ros/noetic/setup.bash
    source ~/anti_drone_agc_workspace/devel/setup.bash
    rosrun drone_launch_controller drone_launch_controller_node
    exec bash
" &
CTRL_TAB_PID=$!
echo "Controller 终端已打开"

sleep 3

# 执行起飞
echo ""
echo "========================================="
echo "所有组件已启动！"
echo "========================================="
echo ""
echo "将在 5 秒后执行起飞命令..."
sleep 5

echo "执行起飞..."
rosservice call /drone_launch_controller/takeoff "{}" 2>/dev/null

echo ""
echo "========================================="
echo "起飞命令已发送！"
echo "========================================="
echo ""
echo "查看日志命令："
echo "  - 高度: rostopic echo /mavros/local_position/pose"
echo "  - 状态: rostopic echo /drone_launch_controller/flight_status"
echo "  - MAVROS: rostopic echo /mavros/state"
echo ""
echo "按 Enter 退出此脚本（不会停止仿真）"
echo "========================================="

read -r

echo "脚本已退出，仿真继续在后台运行"
echo "使用以下命令停止所有进程："
echo "  pkill -9 -f px4; pkill -9 -f gazebo; pkill -9 -f mavros"
