#!/bin/bash
# 统一仿真启动脚本
# 用法: ./scripts/start_sim.sh <mode> [选项]
# 模式: sim | takeoff | lqr | perception
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/anti_drone_agc_workspace"

# 默认参数
ALTITUDE="5.0"
TIMEOUT="30.0"
TRAJECTORY="fixed_horizontal"
WITH_PERCEPTION="false"
WITH_EXPERIMENT_LOGGER="false"
OBSTACLES=()      # 障碍物坐标数组，元素格式 "x,y,z"；空数组表示不添加
GUI_MODE="true"

# 后台进程PID列表
PIDS=()

# ==========================================
# 帮助信息
# ==========================================
show_help() {
    echo "用法: ./scripts/start_sim.sh <mode> [选项]"
    echo ""
    echo "模式:"
    echo "  sim         仅启动 PX4 SITL + Gazebo + MAVROS"
    echo "  takeoff     启动仿真 + drone_launch_controller + 起飞"
    echo "  lqr         启动仿真 + LQR轨迹跟踪器"
    echo "  perception  仅启动 rl_obstacle_avoidance 感知节点"
    echo ""
    echo "选项:"
    echo "  --altitude <米>        起飞目标高度 (默认: 5.0, 用于 takeoff/lqr 模式)"
    echo "  --trajectory <类型>    轨迹类型 (默认: fixed_horizontal, 仅 lqr 模式)"
    echo "  --with-perception      启动 rl_obstacle_avoidance 感知+规划节点 (仅 lqr 模式)"
    echo "  --with-experiment-logger  启动实验数据记录节点 (仅 lqr 模式，需配合 --with-perception)"
    echo "  --obstacle <x,y,z>     在指定坐标 spawn 静态障碍物 (仅 lqr/sim/takeoff 模式)，可多次指定以生成多个障碍物"
    echo "  --headless             禁用 Gazebo GUI，仅运行仿真服务器"
    echo "  -h, --help             显示此帮助"
    echo ""
    echo "示例:"
    echo "  ./scripts/start_sim.sh sim"
    echo "  ./scripts/start_sim.sh takeoff --altitude 10.0"
    echo "  ./scripts/start_sim.sh lqr --with-perception"
    echo "  ./scripts/start_sim.sh lqr --with-perception --with-experiment-logger"
    echo "  ./scripts/start_sim.sh perception"
}

# ==========================================
# 公共函数
# ==========================================

# 设置ROS/Gazebo环境变量
setup_env() {
    source /opt/ros/noetic/setup.bash
    source "$WORKSPACE_DIR/devel/setup.bash"
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/PX4_Firmware:/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo
    export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH
    # 将 ROS 日志目录指向可写位置（避免 ~/.ros/log/ 只读报错）
    export ROS_LOG_DIR=/tmp/ros_logs
    mkdir -p "$ROS_LOG_DIR"
}

# 清理旧进程
cleanup_procs() {
    echo "[清理] 清理旧进程..."
    pkill -9 -f px4 2>/dev/null || true
    pkill -9 -f gazebo 2>/dev/null || true
    pkill -9 -f gzserver 2>/dev/null || true
    pkill -9 -f gzclient 2>/dev/null || true
    pkill -9 -f mavros 2>/dev/null || true
    pkill -9 -f drone_launch_controller 2>/dev/null || true
    pkill -9 -f lqr_tracker_node 2>/dev/null || true
    pkill -9 -f perception_node 2>/dev/null || true
    pkill -9 -f rl_planner_node 2>/dev/null || true
    pkill -9 -f experiment_logger_node 2>/dev/null || true
    sleep 2
}

# 等待MAVROS连接
wait_for_mavros() {
    local max_wait=${1:-60}
    echo "[等待] 等待 MAVROS 连接 (最多 ${max_wait}s)..."
    for i in $(seq 1 $max_wait); do
        CONN=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "connected:" | awk '{print $2}')
        if [ "$CONN" = "True" ]; then
            echo "[等待] MAVROS 已连接 ($(($i))s)"
            return 0
        fi
        sleep 1
    done
    echo "[警告] MAVROS 未在 ${max_wait}s 内连接，继续尝试..."
}

# 等待仿真时钟开始发布
wait_for_clock() {
    local max_wait=${1:-30}
    echo "[等待] 等待仿真时钟 /clock 开始发布 (最多 ${max_wait}s)..."
    for i in $(seq 1 $max_wait); do
        MSG=$(rostopic echo /clock -n 1 2>/dev/null)
        if [ -n "$MSG" ]; then
            echo "[等待] 仿真时钟已启动 ($(($i))s)"
            return 0
        fi
        sleep 1
    done
    echo "[警告] 仿真时钟未在 ${max_wait}s 内启动，继续尝试..."
}

# ==========================================
# 各模式实现
# ==========================================

# 启动PX4 SITL + MAVROS (公共)
start_px4_sitl() {
    echo "[启动] 启动 PX4 SITL + Gazebo + MAVROS (gui=${GUI_MODE})..."
    cd /home/ubuntu/PX4_Firmware
    roslaunch px4 mavros_posix_sitl.launch vehicle:=iris vehicle_sdf:=iris_3d_lidar gui:=${GUI_MODE} interactive:=false &
    PIDS+=($!)
    echo "[启动] PX4 PID: ${PIDS[-1]}"
    cd "$WORKSPACE_DIR"
}

# 模式: sim
run_sim() {
    echo "========================================="
    echo " 仿真模式 - 仅启动 PX4 SITL + MAVROS"
    echo "========================================="

    start_px4_sitl
    sleep 20

    if ! pgrep -f "px4.*sitl" > /dev/null; then
        echo "[错误] PX4 SITL 未启动成功！"
        exit 1
    fi
    echo "[状态] PX4 SITL 运行正常"

    wait_for_mavros 60

    echo ""
    echo "========================================="
    echo " 仿真已就绪"
    echo "========================================="
    echo ""
    echo "监控命令:"
    echo "  rostopic echo /mavros/local_position/pose"
    echo "  rostopic echo /mavros/state"
    echo ""
    echo "按 Ctrl+C 停止仿真"
    echo "========================================="
}

# 模式: takeoff
run_takeoff() {
    echo "========================================="
    echo " 起飞模式 - 目标高度: ${ALTITUDE}m"
    echo "========================================="

    start_px4_sitl
    sleep 20

    if ! pgrep -f "px4.*sitl" > /dev/null; then
        echo "[错误] PX4 SITL 未启动成功！"
        exit 1
    fi
    echo "[状态] PX4 SITL 运行正常"

    wait_for_mavros 60

    # 启动 drone_launch_controller
    echo "[启动] 启动 drone_launch_controller..."
    roslaunch drone_launch_controller drone_controller.launch verbose:=true &
    PIDS+=($!)
    sleep 3

    # 执行起飞
    echo "[起飞] 发送起飞指令到 ${ALTITUDE}m..."
    rosservice call /drone_launch_controller/takeoff "altitude: ${ALTITUDE}
timeout: ${TIMEOUT}"

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
}

# 模式: lqr
run_lqr() {
    echo "========================================="
    echo " LQR轨迹跟踪模式 - 轨迹: ${TRAJECTORY}"
    echo "========================================="

    start_px4_sitl
    sleep 20

    if ! pgrep -f "px4.*sitl" > /dev/null; then
        echo "[错误] PX4 SITL 未启动成功！"
        exit 1
    fi
    echo "[状态] PX4 SITL 运行正常"

    wait_for_mavros 60

    # 可选: spawn 静态障碍物（支持多个）
    if [ ${#OBSTACLES[@]} -gt 0 ]; then
        for i in "${!OBSTACLES[@]}"; do
            spawn_obstacle "${OBSTACLES[$i]}" "$i"
        done
    fi

    # 可选: 启动感知节点
    if [ "$WITH_PERCEPTION" = "true" ]; then
        wait_for_clock 30
        echo "[感知] 启动 rl_obstacle_avoidance 感知节点..."
        roslaunch rl_obstacle_avoidance perception.launch &
        PIDS+=($!)
        echo "[感知] perception_node PID: ${PIDS[-1]}"
        sleep 2

        echo "[感知] 启动 rl_planner_node..."
        roslaunch rl_obstacle_avoidance rl_planner_node.launch &
        PIDS+=($!)
        echo "[感知] rl_planner_node PID: ${PIDS[-1]}"
        sleep 2
    fi

    # 可选: 启动实验数据记录节点
    if [ "$WITH_EXPERIMENT_LOGGER" = "true" ]; then
        echo "[实验日志] 启动 experiment_logger_node..."
        roslaunch rl_obstacle_avoidance experiment_logger.launch &
        PIDS+=($!)
        echo "[实验日志] experiment_logger_node PID: ${PIDS[-1]}"
        sleep 1
    fi

    # 启动 drone_launch_controller 并起飞
    echo "[启动] 启动 drone_launch_controller..."
    roslaunch drone_launch_controller drone_controller.launch verbose:=true &
    PIDS+=($!)
    sleep 3

    echo "[起飞] 发送起飞指令到 ${ALTITUDE}m..."
    rosservice call /drone_launch_controller/takeoff "altitude: ${ALTITUDE}
timeout: ${TIMEOUT}"

    # 启动 LQR轨迹跟踪器
    echo "[启动] 启动 LQR轨迹跟踪器..."
    roslaunch lqr_trajectory_tracker lqr_tracker_sim.launch trajectory_type:=${TRAJECTORY} &
    PIDS+=($!)
    echo "[启动] LQR Tracker PID: ${PIDS[-1]}"

    sleep 3

    echo ""
    echo "========================================="
    echo " LQR轨迹跟踪已启动"
    echo "========================================="
    echo ""
    echo "监控命令:"
    echo "  rostopic echo /mavros/local_position/pose"
    echo "  rostopic echo /lqr_tracker/accel_cmd"
    echo "  rostopic echo /lqr_tracker/ref_trajectory"
    echo ""
    echo "按 Ctrl+C 停止所有进程"
    echo "========================================="
}

# 模式: perception
run_perception() {
    echo "========================================="
    echo " 感知模式 - rl_obstacle_avoidance 感知节点"
    echo "========================================="

    start_px4_sitl
    sleep 20

    if ! pgrep -f "px4.*sitl" > /dev/null; then
        echo "[错误] PX4 SITL 未启动成功！"
        exit 1
    fi
    echo "[状态] PX4 SITL 运行正常"

    wait_for_mavros 60

    # 等待仿真时钟启动（感知节点需要 sim time）
    wait_for_clock 30

    # 启动感知节点
    echo "[感知] 启动 rl_obstacle_avoidance 感知节点..."
    roslaunch rl_obstacle_avoidance perception.launch &
    PIDS+=($!)
    echo "[感知] perception_node PID: ${PIDS[-1]}"
    sleep 2

    echo ""
    echo "========================================="
    echo " 感知节点已启动"
    echo "========================================="
    echo ""
    echo "监控命令:"
    echo "  rostopic echo /obstacle_features"
    echo ""
    echo "按 Ctrl+C 停止所有进程"
    echo "========================================="
}

# 在 Gazebo 中 spawn 静态障碍物
# 参数: $1=坐标 "x,y,z"；$2=索引（用于保证名称唯一，可选）
spawn_obstacle() {
    local coords="$1"
    local idx="${2:-$(date +%s)}"
    local x=$(echo "$coords" | cut -d',' -f1)
    local y=$(echo "$coords" | cut -d',' -f2)
    local z=$(echo "$coords" | cut -d',' -f3)
    local name="obstacle_${idx}_$(date +%s%N | cut -c1-6)"

    echo "[障碍物] 在 ($x, $y, $z) 处 spawn 静态障碍物..."
    echo "
    <sdf version='1.4'>
      <model name='$name'>
        <static>true</static>
        <link name='link'>
          <collision name='collision'>
            <geometry><box><size>0.3 0.3 3</size></box></geometry>
          </collision>
          <visual name='visual'>
            <geometry><box><size>0.3 0.3 3</size></box></geometry>
            <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
          </visual>
        </link>
      </model>
    </sdf>" | rosrun gazebo_ros spawn_model -sdf -stdin -model "$name" -x "$x" -y "$y" -z "$z" && \
    echo "[障碍物] 已生成，可使用 LiDAR 检测" || \
    echo "[警告] 障碍物生成失败，请检查 Gazebo 是否已启动"
}

# ==========================================
# 信号处理 - Ctrl+C 清理
# ==========================================
cleanup_and_exit() {
    echo ""
    echo "[清理] 正在停止所有后台进程..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    # 兜底清理
    pkill -9 -f px4 2>/dev/null || true
    pkill -9 -f gazebo 2>/dev/null || true
    pkill -9 -f gzserver 2>/dev/null || true
    pkill -9 -f gzclient 2>/dev/null || true
    pkill -9 -f mavros 2>/dev/null || true
    pkill -9 -f drone_launch_controller 2>/dev/null || true
    pkill -9 -f lqr_tracker_node 2>/dev/null || true
    pkill -9 -f perception_node 2>/dev/null || true
    pkill -9 -f rl_planner_node 2>/dev/null || true
    pkill -9 -f experiment_logger_node 2>/dev/null || true
    echo "[清理] 已停止"
    exit 0
}
trap cleanup_and_exit SIGINT SIGTERM

# ==========================================
# 主入口 - 参数解析
# ==========================================

# 至少需要一个参数（或 --help/-h）
if [ $# -lt 1 ]; then
    show_help
    exit 1
fi

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

MODE="$1"
shift

# 解析选项参数
while [ $# -gt 0 ]; do
    case "$1" in
        --altitude)
            ALTITUDE="$2"
            shift 2
            ;;
        --trajectory)
            TRAJECTORY="$2"
            shift 2
            ;;
        --with-perception)
            WITH_PERCEPTION="true"
            shift
            ;;
        --obstacle)
            OBSTACLES+=("$2")
            shift 2
            ;;
        --with-experiment-logger)
            WITH_EXPERIMENT_LOGGER="true"
            shift
            ;;
        --headless)
            GUI_MODE="false"
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "[错误] 未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# 设置环境
setup_env

# 清理旧进程
cleanup_procs

# 按模式分发
case "$MODE" in
    sim)
        run_sim
        ;;
    takeoff)
        run_takeoff
        ;;
    lqr)
        run_lqr
        ;;
    perception)
        run_perception
        ;;
    *)
        echo "[错误] 未知模式: $MODE"
        show_help
        exit 1
        ;;
esac

# 等待 Ctrl+C
wait
