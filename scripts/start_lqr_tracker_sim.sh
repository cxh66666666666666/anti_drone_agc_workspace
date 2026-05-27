#!/bin/bash
# LQR轨迹跟踪器仿真测试启动脚本
# 使用方法: ./start_lqr_tracker_sim.sh

echo "========================================="
echo "LQR轨迹跟踪器仿真测试"
echo "========================================="

# 设置环境
source /opt/ros/noetic/setup.bash
source ~/anti_drone_agc_workspace/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/PX4_Firmware:/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH

# 清理旧进程
echo "[1/5] 清理旧进程..."
pkill -9 -f px4 2>/dev/null
pkill -9 -f gazebo 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f mavros 2>/dev/null
pkill -9 -f lqr_tracker_node 2>/dev/null
sleep 2

# 启动 PX4 SITL + Gazebo + MAVROS
echo "[2/5] 启动 PX4 SITL + Gazebo + MAVROS..."
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
echo "[3/5] 等待仿真启动..."
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

# 加载复杂障碍物
echo "[4/5] 加载复杂障碍物..."
echo "等待障碍物生成完毕..."
python3 ~/anti_drone_agc_workspace/scripts/spawn_obstacles.py
if [ $? -eq 0 ]; then
    echo "障碍物加载完成！"
else
    echo "[警告] 障碍物加载可能出现问题"
fi
sleep 3

# 启动 LQR轨迹跟踪器
echo "[5/5] 启动 LQR轨迹跟踪器..."
gnome-terminal --tab --title="LQR Tracker" -- bash -c "
    source /opt/ros/noetic/setup.bash
    source ~/anti_drone_agc_workspace/devel/setup.bash
    export ROS_LOG_DIR=/tmp/ros_logs
    roslaunch lqr_trajectory_tracker lqr_tracker_sim.launch trajectory_type:=fixed_horizontal
    exec bash
" &
TRACKER_TAB_PID=$!
echo "LQR Tracker 终端已打开"

sleep 3

echo ""
echo "========================================="
echo "所有组件已启动！"
echo "========================================="
echo ""
echo "查看信息命令："
echo "  - 无人机位置: rostopic echo /mavros/local_position/pose"
echo "  - LQR控制指令: rostopic echo /lqr_tracker/accel_cmd"
echo "  - 参考轨迹: rostopic echo /lqr_tracker/ref_trajectory"
echo ""
echo "按 Enter 退出此脚本（不会停止仿真）"
echo "========================================="

read -r

echo "脚本已退出，仿真继续在后台运行"
echo "使用以下命令停止所有进程："
echo "  pkill -9 -f px4; pkill -9 -f gazebo; pkill -9 -f mavros"
