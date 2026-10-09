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
TRAINING_MODE="false"
WITH_EXPERIMENT_LOGGER="false"
OBSTACLES=()      # 障碍物 a（已知给 A*，红色）数组，元素格式 "x,y,z"；空数组表示不添加
OBSTACLES_B=()    # 障碍物 b（未知给 A*，蓝色）数组，元素格式 "x,y,z"；空数组表示不添加
OBSTACLE_SPECS_A=() # 场景清单障碍物: type|x,y,z|sx,sy,sz|yaw
OBSTACLE_SPECS_B=()
SCENARIO_FILE=""
SCENARIO_ID="unknown"
SCENARIO_OBSTACLES=""
SENSOR_BLACKOUT="[]"   # 场景注入的传感器失效窗口(秒)，由 scenario_shell.py 覆盖
SCENARIO_LQR_ARGS=()
SCENARIO_MANIFEST=""  # 训练场景 manifest（含多场景，启用单 run 动态轮换）
GOAL_MODE="false" # 复杂环境目标模式：A* 全局路径 + 每集随机目标
GUI_MODE="false"  # 默认不启动 Gazebo GUI（用户偏好）；--headless 为显式同义写法
TRAJECTORY_ALTITUDE=""   # LQR 轨迹目标高度(与起飞高度解耦, 可选)
TRAJECTORY_DURATION=""   # LQR 轨迹总时长(秒, 可选)
ANGULAR_VELOCITY=""      # LQR 圆/螺旋角速度(rad/s, 可选)
LOOP_TRAJECTORY="true"   # LQR 是否在轨迹末端循环
POSITION_LEAD_TIME=""    # LQR 位置设定点提前时间(秒)
SAFETY_FILTER_MODE="disabled"
SAFETY_FILTER_BASE_MARGIN=""
SAFETY_FILTER_KAPPA=""
SAFETY_FILTER_PREDICTION_ERROR=""
SAFETY_FILTER_HORIZON=""
SAFETY_FILTER_DT=""
SAFETY_FILTER_ALPHA=""
SAFETY_FILTER_EPSILON_PRED=""
SAFETY_FILTER_EMPTY_SET_TERMINAL=""  # 空安全集是否终止（true=shield_infeasible）
ACTION_INTERFACE=""      # 动作接口模式 masked/raw（空=用 gazebo_env 默认 masked）

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
    echo "  --trajectory <类型>    轨迹类型 (含 figure_eight；默认 fixed_horizontal，仅 lqr 模式)"
    echo "  --trajectory-altitude <米>  LQR 轨迹目标高度, 与起飞高度解耦 (仅 lqr 模式, 可选)"
    echo "  --trajectory-duration <秒>  LQR 轨迹总时长(仅 lqr 模式, 可选)"
    echo "  --angular-velocity <rad/s>  LQR 圆/螺旋角速度(仅 lqr 模式, 可选)"
    echo "  --no-trajectory-loop    LQR 到达轨迹末端后保持终点(仅 lqr 模式)"
    echo "  --with-perception      启动 rl_obstacle_avoidance 感知+规划节点 (仅 lqr 模式)"
    echo "  --training             仅启动感知节点，供 DQN 独占控制 offset (仅 lqr 模式)"
    echo "  --with-experiment-logger  启动实验数据记录节点 (仅 lqr 模式，需配合 --with-perception)"
    echo "  --obstacle <x,y,z>     在指定坐标 spawn 障碍物 a（红色，A* 已知）(仅 lqr/sim/takeoff 模式)，可多次指定"
    echo "  --obstacle-b <x,y,z>   在指定坐标 spawn 障碍物 b（蓝色，A* 未知）(仅 lqr/sim/takeoff 模式)，可多次指定"
    echo "  --scenario <json>       从 Stage-2 场景 JSON 加载障碍物、轨迹和高度"
    echo "  --scenario-manifest <json>  训练场景 manifest（含多场景），lqr --training 下单 run 动态轮换"
    echo "  --goal-mode            复杂环境目标模式：LQR 按 A* 航点飞行 + 每集随机目标 (仅 lqr --training)"
    echo "  --headless             不启动 Gazebo GUI（默认已如此，保留为显式写法）"
    echo "  --gui                  启动 Gazebo GUI（非无头；默认不启动）"
    echo "  --safety-filter-mode <disabled|a1|p>  运行时安全过滤模式 (仅 lqr --training)"
    echo "  --safety-filter-base-margin <米>      安全过滤基础净空裕量"
    echo "  --safety-filter-kappa <值>            P 模式跟踪误差系数"
    echo "  --safety-filter-prediction-error <米> 预测误差裕量"
    echo "  --safety-filter-horizon <秒>          预测时域"
    echo "  --safety-filter-dt <秒>               预测采样周期"
    echo "  --safety-filter-alpha <值>            离散安全条件系数"
    echo "  --safety-filter-epsilon-pred <米>     离散预测误差阈值"
    echo "  --safety-filter-empty-set-terminal <true|false>  空安全集是否终止（默认 false 降级回退）"
    echo "  --action-interface <masked|raw>       动作接口模式（raw 让距离掩码退出裁决）"
    echo "  -h, --help             显示此帮助"
    echo ""
    echo "示例:"
    echo "  ./scripts/start_sim.sh sim"
    echo "  ./scripts/start_sim.sh takeoff --altitude 10.0"
    echo "  ./scripts/start_sim.sh lqr --with-perception"
    echo "  ./scripts/start_sim.sh lqr --training"
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
    pkill -9 -f obstacle_visualizer 2>/dev/null || true
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

wait_for_node() {
    local node_name="$1"
    local max_wait=${2:-10}
    for i in $(seq 1 "$max_wait"); do
        if rosnode list 2>/dev/null | grep -qx "$node_name"; then
            echo "[等待] ROS 节点已启动: ${node_name} ($((${i}))s)"
            return 0
        fi
        sleep 1
    done
    echo "[错误] ROS 节点启动失败: ${node_name}"
    return 1
}

wait_for_topic_message() {
    local topic_name="$1"
    local max_wait=${2:-10}
    if timeout "${max_wait}s" rostopic echo "$topic_name" -n 1 >/dev/null 2>&1; then
        echo "[等待] 已收到话题数据: ${topic_name}"
        return 0
    fi
    echo "[错误] 未收到话题数据: ${topic_name} (${max_wait}s)"
    return 1
}

wait_for_service() {
    local service_name="$1"
    local max_wait=${2:-15}
    for i in $(seq 1 "$max_wait"); do
        if rosservice list 2>/dev/null | grep -qx "$service_name"; then
            echo "[等待] ROS 服务已就绪: ${service_name} ($((${i}))s)"
            return 0
        fi
        sleep 1
    done
    echo "[错误] 未发现 ROS 服务: ${service_name} (${max_wait}s)"
    return 1
}

# ==========================================
# 各模式实现
# ==========================================

# 启动PX4 SITL + MAVROS (公共)
start_px4_sitl() {
    # 机体 sdf：默认 PX4 原版 iris_3d_lidar；环境变量 SITL_SDF 可覆盖为派生模型绝对路径
    # （用于「终止后物理结局」评测探针的带接触传感机体，不改原版模型文件）。
    local sdf_args=("vehicle_sdf:=iris_3d_lidar")
    if [ -n "${SITL_SDF:-}" ]; then
        sdf_args=("sdf:=${SITL_SDF}")
        echo "[启动] 使用自定义机体 sdf: ${SITL_SDF}"
    fi
    echo "[启动] 启动 PX4 SITL + Gazebo + MAVROS (gui=${GUI_MODE})..."
    cd /home/ubuntu/PX4_Firmware
    roslaunch px4 mavros_posix_sitl.launch vehicle:=iris "${sdf_args[@]}" gui:=${GUI_MODE} interactive:=false &
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
    roslaunch drone_launch_controller drone_controller.launch quiet_flight_status:=true &
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

    if [ -n "$SCENARIO_FILE" ]; then
        rosparam set /dqn_training/scenario_obstacles_json "$SCENARIO_OBSTACLES"
        rosparam set /dqn_training/scenario_id "$SCENARIO_ID"
        rosparam set /dqn_training/sensor_blackout_s "$SENSOR_BLACKOUT"
    fi

    if [ "$TRAINING_MODE" = "true" ]; then
        local obstacle_spec=""
        if [ ${#OBSTACLES[@]} -gt 0 ]; then
            obstacle_spec=$(IFS=';'; echo "${OBSTACLES[*]}")
        fi
        rosparam set /dqn_training/scenario_obstacles "$obstacle_spec"
        if [ ${#OBSTACLES[@]} -gt 0 ]; then
            a_list="["; for o in "${OBSTACLES[@]}"; do a_list="${a_list}[${o}],"; done
            a_list="${a_list%,}]"
            rosparam set /dqn_training/obstacles_a "$a_list"
        else
            rosparam set /dqn_training/obstacles_a "[]"
        fi
        if [ ${#OBSTACLES_B[@]} -gt 0 ]; then
            b_list="["; for o in "${OBSTACLES_B[@]}"; do b_list="${b_list}[${o}],"; done
            b_list="${b_list%,}]"
            rosparam set /dqn_training/obstacles_b "$b_list"
        else
            rosparam set /dqn_training/obstacles_b "[]"
        fi
        rosparam set /dqn_training/trajectory "$TRAJECTORY"
        rosparam set /dqn_training/altitude "$ALTITUDE"
        rosparam set /gazebo_env/w_progress 1.0
        case "$SAFETY_FILTER_MODE" in
            disabled|a1|p) ;;
            *) echo "[错误] --safety-filter-mode 必须为 disabled、a1 或 p"; exit 1 ;;
        esac
        rosparam set /gazebo_env/safety_filter_mode "$SAFETY_FILTER_MODE"
        [ -n "$SAFETY_FILTER_BASE_MARGIN" ] && rosparam set /gazebo_env/safety_filter_base_margin "$SAFETY_FILTER_BASE_MARGIN"
        [ -n "$SAFETY_FILTER_KAPPA" ] && rosparam set /gazebo_env/safety_filter_kappa "$SAFETY_FILTER_KAPPA"
        [ -n "$SAFETY_FILTER_PREDICTION_ERROR" ] && rosparam set /gazebo_env/safety_filter_prediction_error "$SAFETY_FILTER_PREDICTION_ERROR"
        [ -n "$SAFETY_FILTER_HORIZON" ] && rosparam set /gazebo_env/safety_filter_horizon_s "$SAFETY_FILTER_HORIZON"
        [ -n "$SAFETY_FILTER_DT" ] && rosparam set /gazebo_env/safety_filter_dt "$SAFETY_FILTER_DT"
        [ -n "$SAFETY_FILTER_ALPHA" ] && rosparam set /gazebo_env/safety_filter_alpha "$SAFETY_FILTER_ALPHA"
        [ -n "$SAFETY_FILTER_EPSILON_PRED" ] && rosparam set /gazebo_env/safety_filter_epsilon_pred "$SAFETY_FILTER_EPSILON_PRED"
        [ -n "$SAFETY_FILTER_EMPTY_SET_TERMINAL" ] && rosparam set /gazebo_env/safety_filter_empty_set_terminal "$SAFETY_FILTER_EMPTY_SET_TERMINAL"
        case "$ACTION_INTERFACE" in
            ""|masked|raw) ;;
            *) echo "[错误] --action-interface 必须为 masked 或 raw"; exit 1 ;;
        esac
        [ -n "$ACTION_INTERFACE" ] && rosparam set /gazebo_env/action_interface "$ACTION_INTERFACE"
        if [ "$GOAL_MODE" = "true" ]; then
            rosparam set /dqn_training/goal_mode true
            rosparam set /dqn_training/goal_bounds "[-11.0, 11.0, -11.0, 11.0]"
        fi
        # 跨场景轮换：manifest 模式下障碍物由 env 动态部署，激活开关
        if [ -n "$SCENARIO_MANIFEST" ]; then
            rosparam set /dqn_training/scenario_manifest "$SCENARIO_MANIFEST"
            rosparam set /gazebo_env/scenario_rotation true
            rosparam set /gazebo_env/phase_aligned_reset true
            # 清空静态障碍物，避免随机命名干扰动态清理（manifest 模式按需站场景 spawn）
            OBSTACLES=()
            OBSTACLES_B=()
            OBSTACLE_SPECS_A=()
            OBSTACLE_SPECS_B=()
        fi
    fi

    # 可选: spawn 静态障碍物（支持多个；a 红 / b 蓝）
    if [ ${#OBSTACLES[@]} -gt 0 ]; then
        for i in "${!OBSTACLES[@]}"; do
            if [ ${#OBSTACLE_SPECS_A[@]} -gt "$i" ]; then
                spawn_obstacle_spec "${OBSTACLE_SPECS_A[$i]}" "$i" "1 0 0 1"
            else
                spawn_obstacle "${OBSTACLES[$i]}" "$i" "1 0 0 1"
            fi
        done
    fi
    if [ ${#OBSTACLES_B[@]} -gt 0 ]; then
        for i in "${!OBSTACLES_B[@]}"; do
            if [ ${#OBSTACLE_SPECS_B[@]} -gt "$i" ]; then
                spawn_obstacle_spec "${OBSTACLE_SPECS_B[$i]}" "b$i" "0 0 1 1"
            else
                spawn_obstacle "${OBSTACLES_B[$i]}" "b$i" "0 0 1 1"
            fi
        done
    fi

    # 可选: 启动感知节点
    if [ "$WITH_PERCEPTION" = "true" ]; then
        wait_for_clock 30
        echo "[感知] 启动 rl_obstacle_avoidance 感知节点..."
        roslaunch rl_obstacle_avoidance perception.launch &
        PIDS+=($!)
        echo "[感知] perception_node PID: ${PIDS[-1]}"
        wait_for_node /perception_node 10
        wait_for_topic_message /obstacle_features 10

        if [ "$GOAL_MODE" = "true" ]; then
            echo "[规划] 启动 A* 规划节点..."
            rosrun rl_obstacle_avoidance astar_planner.py &
            PIDS+=($!)
            echo "[规划] astar_planner PID: ${PIDS[-1]}"
            wait_for_service /astar_planner/plan 15
        fi

        if [ "$TRAINING_MODE" != "true" ]; then
            echo "[感知] 启动 rl_planner_node..."
            roslaunch rl_obstacle_avoidance rl_planner_node.launch &
            PIDS+=($!)
            echo "[感知] rl_planner_node PID: ${PIDS[-1]}"
            sleep 2
        fi
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
    roslaunch drone_launch_controller drone_controller.launch quiet_flight_status:=true &
    PIDS+=($!)
    sleep 3

    echo "[起飞] 发送起飞指令到 ${ALTITUDE}m..."
    rosservice call /drone_launch_controller/takeoff "altitude: ${ALTITUDE}
timeout: ${TIMEOUT}"

    # 等待起飞完成(高度接近目标)再启动 LQR, 避免 LQR 抢跑导致起飞被中断
    echo "[等待] 等待起飞完成至 ${ALTITUDE}m..."
    local t=0
    while [ $t -lt 100 ]; do
        local z
        z=$(timeout 5 rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | awk '/z:/{print $2; exit}')
        if [ -n "$z" ] && python3 -c "import sys; sys.exit(0 if abs($z-$ALTITUDE)<0.5 else 1)" 2>/dev/null; then
            echo "[等待] 起飞完成 (z=${z})"
            break
        fi
        sleep 2; t=$((t+2))
    done

    # 启动 LQR轨迹跟踪器
    echo "[启动] 启动 LQR轨迹跟踪器..."
    if [ "$GOAL_MODE" = "true" ]; then
        LQR_ARGS=("trajectory_type:=waypoints")
    elif [ ${#SCENARIO_LQR_ARGS[@]} -gt 0 ]; then
        LQR_ARGS=("${SCENARIO_LQR_ARGS[@]}")
    else
        LQR_ARGS=("trajectory_type:=${TRAJECTORY}")
        if [ "$TRAJECTORY" = "step" ]; then
            LQR_ARGS+=("step_z:=${TRAJECTORY_ALTITUDE:-$ALTITUDE}")
        elif [ "$TRAJECTORY" = "linear" ]; then
            LQR_ARGS+=("start_z:=${TRAJECTORY_ALTITUDE:-$ALTITUDE}")
        elif [ "$TRAJECTORY" = "fixed_horizontal" ]; then
            LQR_ARGS+=("fixed_horizontal_height:=${TRAJECTORY_ALTITUDE:-$ALTITUDE}")
        elif [ "$TRAJECTORY" = "figure_eight" ]; then
            LQR_ARGS+=("figure_eight_height:=${TRAJECTORY_ALTITUDE:-$ALTITUDE}")
        fi
        [ -n "$TRAJECTORY_DURATION" ] && LQR_ARGS+=("trajectory_duration:=${TRAJECTORY_DURATION}")
        [ -n "$ANGULAR_VELOCITY" ] && LQR_ARGS+=("angular_velocity:=${ANGULAR_VELOCITY}")
        [ "$LOOP_TRAJECTORY" = "false" ] && LQR_ARGS+=("loop_trajectory:=false")
    fi
    [ -n "$POSITION_LEAD_TIME" ] && LQR_ARGS+=("position_lead_time:=${POSITION_LEAD_TIME}")
    [ "$GUI_MODE" = "false" ] && LQR_ARGS+=("rviz:=false")
    roslaunch lqr_trajectory_tracker lqr_tracker_sim.launch "${LQR_ARGS[@]}" &
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
    wait_for_node /perception_node 10
    wait_for_topic_message /obstacle_features 10

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
    local color="${3:-1 0 0 1}"
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
            <material><ambient>${color}</ambient><diffuse>${color}</diffuse></material>
          </visual>
        </link>
      </model>
    </sdf>" | rosrun gazebo_ros spawn_model -sdf -stdin -model "$name" -x "$x" -y "$y" -z "$z" && \
    echo "[障碍物] 已生成，可使用 LiDAR 检测" || \
    echo "[警告] 障碍物生成失败，请检查 Gazebo 是否已启动"
}

# Spawn a typed obstacle from scenario_generator.py:
# type|x,y,z|sx,sy,sz|yaw
spawn_obstacle_spec() {
    local spec="$1"; local idx="${2:-$(date +%s)}"; local color="${3:-1 0 0 1}"
    local type coords size yaw x y z sx sy sz
    IFS='|' read -r type coords size yaw <<< "$spec"
    IFS=',' read -r x y z <<< "$coords"
    IFS=',' read -r sx sy sz <<< "$size"
    local name="obstacle_${idx}_$(date +%s%N | cut -c1-6)"
    local geometry
    if [ "$type" = "cylinder" ]; then
        geometry="<cylinder><radius>$(awk -v x="$sx" -v y="$sy" 'BEGIN{print (x>y?x:y)/2}')</radius><length>$sz</length></cylinder>"
    else
        geometry="<box><size>$sx $sy $sz</size></box>"
    fi
    echo "[障碍物] spawn $type 在 ($x, $y, $z), size=($sx,$sy,$sz), yaw=$yaw"
    echo "<sdf version='1.4'><model name='$name'><static>true</static><pose>0 0 0 0 0 $yaw</pose><link name='link'><collision name='collision'><geometry>$geometry</geometry></collision><visual name='visual'><geometry>$geometry</geometry><material><ambient>${color}</ambient><diffuse>${color}</diffuse></material></visual></link></model></sdf>" |
        rosrun gazebo_ros spawn_model -sdf -stdin -model "$name" -x "$x" -y "$y" -z "$z" ||
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
        --trajectory-altitude)
            TRAJECTORY_ALTITUDE="$2"
            shift 2
            ;;
        --trajectory-duration)
            TRAJECTORY_DURATION="$2"
            shift 2
            ;;
        --angular-velocity)
            ANGULAR_VELOCITY="$2"
            shift 2
            ;;
        --no-trajectory-loop)
            LOOP_TRAJECTORY="false"
            shift
            ;;
        --position-lead-time)
            POSITION_LEAD_TIME="$2"
            shift 2
            ;;
        --safety-filter-mode)
            SAFETY_FILTER_MODE="$2"
            shift 2
            ;;
        --safety-filter-base-margin)
            SAFETY_FILTER_BASE_MARGIN="$2"
            shift 2
            ;;
        --safety-filter-kappa)
            SAFETY_FILTER_KAPPA="$2"
            shift 2
            ;;
        --safety-filter-prediction-error)
            SAFETY_FILTER_PREDICTION_ERROR="$2"
            shift 2
            ;;
        --safety-filter-horizon)
            SAFETY_FILTER_HORIZON="$2"
            shift 2
            ;;
        --safety-filter-dt)
            SAFETY_FILTER_DT="$2"
            shift 2
            ;;
        --safety-filter-alpha)
            SAFETY_FILTER_ALPHA="$2"
            shift 2
            ;;
        --safety-filter-epsilon-pred)
            SAFETY_FILTER_EPSILON_PRED="$2"
            shift 2
            ;;
        --safety-filter-empty-set-terminal)
            SAFETY_FILTER_EMPTY_SET_TERMINAL="$2"
            shift 2
            ;;
        --action-interface)
            ACTION_INTERFACE="$2"
            shift 2
            ;;
        --with-perception)
            WITH_PERCEPTION="true"
            shift
            ;;
        --training)
            WITH_PERCEPTION="true"
            TRAINING_MODE="true"
            shift
            ;;
        --obstacle)
            OBSTACLES+=("$2")
            shift 2
            ;;
        --obstacle-b)
            OBSTACLES_B+=("$2")
            shift 2
            ;;
        --scenario)
            SCENARIO_FILE="$2"
            shift 2
            ;;
        --scenario-manifest)
            SCENARIO_MANIFEST="$2"
            shift 2
            ;;
        --goal-mode)
            GOAL_MODE="true"
            shift
            ;;
        --with-experiment-logger)
            WITH_EXPERIMENT_LOGGER="true"
            shift
            ;;
        --headless)
            GUI_MODE="false"
            shift
            ;;
        --gui)
            GUI_MODE="true"
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

if [ -n "$SCENARIO_FILE" ]; then
    if [ ! -f "$SCENARIO_FILE" ]; then
        echo "[错误] 场景文件不存在: $SCENARIO_FILE"
        exit 1
    fi
    # The helper emits shell-quoted assignments only; this preserves the
    # existing coordinate-array interface while adding typed geometry.
    source <(python3 "$WORKSPACE_DIR/scripts/scenario_shell.py" "$SCENARIO_FILE")
fi

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
