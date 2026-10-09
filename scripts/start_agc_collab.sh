#!/bin/bash
# 空地协同 run 一键启动（无头）+ 干净退出 + 连续运行批跑（M0 接口契约 v1.1 / M1）
#
# 用法: ./scripts/start_agc_collab.sh [选项]
#
# 编排流程（唯一共同世界 + 空中控制权交接）：
#   1) 清理本任务相关旧进程
#   2) 后台启动 air_ground_core/agc_collab.launch（单 gzserver + agc_world + 协同节点）
#   3) 等 /mavros/state connected 与 /clock
#   4) 调 /drone_launch_controller/takeoff 起飞，等 /mavros/local_position/pose 到位
#   5) 后台启动 LQR（trajectory_type=step）；LQR 启动时自行 release_control，即交接点
#   6) 等 /agc/air/handover_ok 为 true（air_control_bridge 核查实际交接与无空窗）
#      —— M1 起本项为 start 的硬前置：未通过则不进入 attempt，按分类记录 startup 异常
#   7) 连续批跑（--cycles 次「启动→退出」全循环，每循环内 --repeat 次 attempt）
#      每 attempt: reset -> 等 SUCCESS/odom 恢复 -> start -> 等终态 -> 记录
#   8) 记录退出前进程清单 -> 干净停止 -> 退出后核查关键进程残留
#
# 运行期常用操作（另开终端）：
#   rosservice call /agc/task/start          # 门禁通过后进入 AIR_SEARCH（std_srvs/Trigger）
#   rosservice call /agc/task/abort          # 人工急停 -> ABORTED（std_srvs/Trigger）
#   rosservice call /agc/scenario/reset      # 场景复位（std_srvs/Trigger）
#   rosparam get /agc/air/handover_ok        # 空中交接核查结果
#   rosparam get /agc/start/last_gate_json   # 最近一次 start 门禁明细
#
# 选项:
#   --takeoff-altitude <米>  起飞/悬停高度（默认 2.0）
#   --duration <秒>          非批跑模式下的运行时长后自动退出（默认 180）
#   --scenario <文件>        锁定场景文件（默认 config/scenario_flat.yaml）
#   --sensors-file <文件>    感知通道配置文件（默认 包内 config/perception_sensors.yaml）
#   --scene-seed <整数>      运行种子（默认 不覆盖=unseeded）
#   --world-file <文件>      世界文件（默认 包内 worlds/agc_world.world）
#   --repeat <N>             单次启动内连续 attempt 次数（默认 1；指定即进入批跑模式）
#   --cycles <M>             「启动→退出」全循环次数（默认 1；指定即进入批跑模式）
#   --serve                  常驻服务模式：一次启动栈后空闲等待，供上层逐回合热切换（不重启栈）
#   --dynamic-obstacles      透传 dynamic_obstacles:=true（障碍由 scenario_reset 动态增删）
#   --run-timeout <秒>       单 attempt 等待终态的最长秒数（默认 150）
#   --sample <名称>          预留：给上层注入 run 名称（默认不注入）
#   --auto-run <样例>        兼容旧用法：起飞交接后自动跑一个样例（success|mission_failed|
#                            infra_failure|aborted）并干净退出
#   --gui                    启动 Gazebo GUI（默认无头）
#   -h, --help               显示本帮助
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/anti_drone_agc_workspace"
RUNS_ROOT="$WORKSPACE_DIR/experiments/agc_runs"

# 默认参数
TAKEOFF_ALTITUDE="2.0"
DURATION="180"
SAMPLE=""
GUI_MODE="false"
SERVE_MODE="false"   # 常驻服务模式：一次启动栈后空闲等待停止信号，不进入 attempt 循环
DYNAMIC_OBSTACLES="false"  # 透传 dynamic_obstacles:=true（障碍由 scenario_reset 动态增删）
FOLLOW_MODE="false"  # N2：是否启用无人机跟随移动目标（安全距离带）
AUTO_RUN=""          # 自动跑一个样例: success | mission_failed | infra_failure | aborted
RUN_TIMEOUT="150"    # 单个 attempt 等待终态的最长秒数
SCENARIO_FILE=""     # 空 = 用包内 config/scenario_flat.yaml
PERCEPTION_SENSORS_FILE=""  # D1：感知通道配置文件（空=用包内默认）
SCENE_SEED=""               # D1：运行种子（空=不覆盖，日志回落 unseeded）
WORLD_FILE=""               # D1：世界文件（空=用包内默认 agc_world.world）
TARGET_SOURCE="gt_adapter"   # 目标源: gt_adapter | air_fusion（run 前固定）
OBSTACLE_SOURCE="scenario"   # 障碍源: scenario | air_fusion（run 前固定）
REPEAT="1"           # 单次启动内 attempt 数
CYCLES="1"           # 启动→退出全循环数
BATCH_MODE="false"   # 显式给了 --repeat/--cycles 才进入批跑
HANDOVER_TIMEOUT="60"
# M4 感知故障注入（离线 rosparam 注入；实时链路节点为 C++）
INJECT_MODE=""       # "" = 不注入；latency | loss | conflict
TIMEOUT_INJECT=""    # N4：run 前冻结的任务超时(s)，空=不覆盖（构造 MISSION_TIMEOUT 场景）
HANDOVER_INJECT=""   # N4：交接后按秒终止 lqr_tracker_node，制造控制空窗
FAULT_PRESET=""      # N4：启动前预置感知故障通道（空=不预置；故障在首次观测前生效，用于冷启动冲突）
FAULT_PRESET_BIAS="3.0"  # N4：预置离群偏置(m)
INJECT_SENSOR="all"  # 目标通道 id、逗号分隔或 all
INJECT_VALUE=""      # 覆盖注入强度（latency_s / drop_prob / outlier_bias_m）
INJECT_DELAY="0"     # start 后延迟多少秒再注入（让目标先被获得/跟踪，再注入故障）
# M5 地面写入权冲突注入（外部 set_model_state 写入者；实时链路节点为 C++）
CONFLICT_INJECT="false"  # true = 在 start 后注入外部写车位姿
CONFLICT_TX="-4.0"       # 固定遥测点 x（默认场景车起点，净空充裕）
CONFLICT_TY="-3.0"       # 固定遥测点 y
CONFLICT_DELAY="6"       # start 后延迟多少秒再注入
CONFLICT_DURATION="8"    # 注入持续秒数
BATCH_PREFIX="_m1_batch_"  # 批跑目录前缀（M4 证据用 _m4_ 前缀）

# 进程生命周期
LAUNCH_PID=""
LQR_PID=""
BATCH_DIR=""
RECORDS=""

# 需要核查/清理的进程名（与 agc_run_logger 的 procKeywords 一致）
PROC_NAMES=(px4 gzserver gzclient mavros air_odom_adapter ground_odom_adapter \
    gt_adapter air_perception air_fusion air_follow map_adapter task_manager ground_planner ground_commander \
    ground_drive_adapter intruder_driver scenario_reset agc_run_logger \
    air_control_bridge lqr_tracker_node drone_launch_controller rosmaster roscore rosout)

# 自身及祖先 PID：本脚本命令行可能含 air_fusion 等关键字（如 --target-source air_fusion），
# 清理/核查时必须排除自身与祖先，否则 pkill -f 会误杀本脚本、proc_json 会误报残留。
SELF_PIDS=""
collect_self_pids() {
    local p=$$ pp
    while [ -n "$p" ] && [ "$p" -gt 1 ]; do
        case " $SELF_PIDS " in *" $p "*) break ;; esac
        SELF_PIDS="$SELF_PIDS $p"
        pp=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
        [ -z "$pp" ] && break
        p="$pp"
    done
}
is_self_pid() { case " $SELF_PIDS " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }

# 按关键字杀掉进程，排除自身/祖先
kill_matching() {
    local pid
    for pid in $(pgrep -f "$1" 2>/dev/null || true); do
        is_self_pid "$pid" && continue
        kill -9 "$pid" 2>/dev/null || true
    done
}

# ==========================================
# 帮助信息
# ==========================================
show_help() {
    echo "用法: ./scripts/start_agc_collab.sh [选项]"
    echo ""
    echo "选项:"
    echo "  --takeoff-altitude <米>  起飞/悬停高度 (默认: 2.0)"
    echo "  --duration <秒>          非批跑模式运行时长后自动退出 (默认: 180)"
    echo "  --scenario <文件>        锁定场景文件 (默认: config/scenario_flat.yaml)"
    echo "  --sensors-file <文件>    感知通道配置文件 (默认: 包内 config/perception_sensors.yaml)"
    echo "  --scene-seed <整数>      运行种子，写入 /agc/log/seed (默认: 不覆盖=unseeded)"
    echo "  --world-file <文件>      世界文件 (默认: 包内 worlds/agc_world.world)"
    echo "  --target-source <源>     目标源 gt_adapter|air_fusion (默认: gt_adapter)"
    echo "  --obstacle-source <源>   障碍源 scenario|air_fusion (默认: scenario)"
    echo "  --inject <模式>          感知故障注入 latency|loss|conflict (默认: 不注入)"
    echo "  --inject-sensor <通道>   注入通道 id、逗号分隔或 all (默认: all)"
    echo "  --inject-value <值>      注入强度 latency_s/drop_prob/outlier_bias_m"
    echo "  --inject-delay <秒>      start 后延迟多少秒再注入 (默认: 0)"
    echo "  --timeout-inject <秒>    N4：覆盖任务超时(mission_timeout_s)以构造超时安全场景 (默认: 不覆盖)"
    echo "  --handover-inject <秒>   N4：交接后按秒终止 lqr_tracker_node，制造控制空窗 (默认: 不注入)"
    echo "  --fault-preset <通道>    N4：启动前预置感知故障通道（冷启动冲突；默认: 不预置）"
    echo "  --fault-preset-value <米> N4：预置离群偏置强度 (默认: 3.0)"
    echo "  --conflict-inject        M5 启用外部写车位姿注入（验证写入权冲突检出）"
    echo "  --conflict-target-x <米> 外部写车位姿固定遥测点 x (默认: -4.0)"
    echo "  --conflict-target-y <米> 外部写车位姿固定遥测点 y (默认: -3.0)"
    echo "  --conflict-delay <秒>    外部写车位姿注入延迟 (默认: 6)"
    echo "  --conflict-duration <秒> 外部写车位姿注入持续 (默认: 8)"
    echo "  --batch-prefix <前缀>    批跑目录前缀 (默认: _m1_batch_)"
    echo "  --repeat <N>             单次启动内连续 attempt 次数 (默认: 1)"
    echo "  --cycles <M>             启动→退出全循环次数 (默认: 1)"
    echo "  --run-timeout <秒>       单 attempt 等待终态最长秒数 (默认: 150)"
    echo "  --sample <名称>          预留：给上层注入 run 名称 (默认不注入)"
    echo "  --auto-run <样例>        起飞交接后自动跑一个样例：success | mission_failed |"
    echo "                           infra_failure | aborted（跑完自动干净退出）"
    echo "  --serve                  常驻服务模式：启动栈+起飞+交接后空闲等待，不进入 attempt 循环；"
    echo "                           就绪时打印 batch_dir=... 与 serve_ready=true；"
    echo "                           出现 \$batch_dir/serve_stop 或收到 SIGTERM/SIGINT 时干净退出"
    echo "  --dynamic-obstacles      透传 dynamic_obstacles:=true（障碍由 scenario_reset 动态增删）"
    echo "  --gui                    启动 Gazebo GUI (默认无头)"
    echo "  --follow                 N2：启用无人机跟随移动目标（安全距离带）"
    echo "  -h, --help               显示此帮助"
    echo ""
    echo "M1 连续运行示例:"
    echo "  ./scripts/start_agc_collab.sh --repeat 10 --cycles 3"
}

# ==========================================
# 环境与清理
# ==========================================
# 设置 ROS/Gazebo 环境变量（与 start_sim.sh 保持一致）
setup_env() {
    source /opt/ros/noetic/setup.bash
    source "$WORKSPACE_DIR/devel/setup.bash"
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/PX4_Firmware:/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo
    export GAZEBO_MODEL_PATH="$WORKSPACE_DIR/src/air_ground_core/models:$WORKSPACE_DIR/src/rl_ground_avoidance/models:/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH"
    export ROS_LOG_DIR=/tmp/ros_logs
    mkdir -p "$ROS_LOG_DIR"
    collect_self_pids  # 记录自身/祖先 PID，供清理与残留核查排除
}

# 清理旧进程：只清理本任务会用到的进程
cleanup_procs() {
    echo "[清理] 清理旧进程..."
    for n in "${PROC_NAMES[@]}"; do
        kill_matching "$n"
    done
    # 每个协同 run 用全新的 ROS master：避免上一次 run 的残留注册污染本 run
    kill_matching rosmaster
    kill_matching roscore
    sleep 3
}

# ==========================================
# 进程清单 / 残留核查（M1 交付口径）
# ==========================================
# 输出关键进程清单 JSON 数组（按 pid 去重）
proc_json() {
    declare -A seen
    local out="[" first=1 comm pid
    for n in "${PROC_NAMES[@]}"; do
        for pid in $(pgrep -f "$n" 2>/dev/null || true); do
            if [ -n "${seen[$pid]:-}" ]; then continue; fi
            is_self_pid "$pid" && continue
            seen[$pid]=1
            comm=""
            if [ -r "/proc/$pid/comm" ]; then comm=$(tr -d '\n' < "/proc/$pid/comm" 2>/dev/null || true); fi
            # 排除编排脚本自身及其命令替换子 shell：其命令行含 --target-source air_fusion 等关键字，
            # 会被 PROC_NAMES 匹配成"残留"，但它不是任务节点进程。
            case "$comm" in *start_agc_colla*) continue ;; esac
            [ -z "$comm" ] && continue
            if [ $first -eq 1 ]; then first=0; else out+=","; fi
            out+="{\"pid\": $pid, \"comm\": \"$comm\", \"match\": \"$n\"}"
        done
    done
    out+="]"
    printf '%s' "$out"
}

record_json() { printf '%s\n' "$1" >> "$RECORDS"; }

# bool 转 JSON：True/true -> true
jb() { if [ "$1" = "True" ] || [ "$1" = "true" ]; then echo true; else echo false; fi; }

# ==========================================
# 等待工具
# ==========================================
wait_for_mavros() {
    local max_wait=${1:-90}
    echo "[等待] 等待 MAVROS 连接 (最多 ${max_wait}s)..."
    for i in $(seq 1 "$max_wait"); do
        CONN=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "connected:" | awk '{print $2}')
        if [ "$CONN" = "True" ]; then
            echo "[等待] MAVROS 已连接 (${i}s)"
            return 0
        fi
        sleep 1
    done
    echo "[警告] MAVROS 未在 ${max_wait}s 内连接"
    return 1
}

wait_for_clock() {
    local max_wait=${1:-30}
    echo "[等待] 等待仿真时钟 /clock 开始发布 (最多 ${max_wait}s)..."
    for i in $(seq 1 "$max_wait"); do
        MSG=$(rostopic echo /clock -n 1 2>/dev/null)
        if [ -n "$MSG" ]; then
            echo "[等待] 仿真时钟已启动 (${i}s)"
            return 0
        fi
        sleep 1
    done
    echo "[警告] 仿真时钟未在 ${max_wait}s 内启动"
    return 1
}

wait_for_service() {
    local svc="$1"; local max_wait=${2:-30}
    echo "[等待] 等待服务就绪: ${svc} (最多 ${max_wait}s)..."
    for i in $(seq 1 "$max_wait"); do
        if rosservice list 2>/dev/null | grep -qx "$svc"; then
            echo "[等待] 服务已就绪: ${svc} (${i}s)"
            return 0
        fi
        sleep 1
    done
    echo "[错误] 未发现服务: ${svc}"
    return 1
}

# 等待终态；返回 0 到终态，1 超时
wait_terminal() {
    local timeout="$1"; local t=0; local S=""
    while [ $t -lt "$timeout" ]; do
        S=$(timeout 6 rostopic echo /agc/task/state -n 1 2>/dev/null | awk '/^state:/{print $2; exit}')
        case "$S" in
            4|5|6|8) echo "[样例] 到达终态 state=${S}"; return 0 ;;
        esac
        sleep 3; t=$((t+3))
    done
    echo "[样例] 未在 ${timeout}s 内到达终态（最后 state=${S}）"
    return 1
}

# ==========================================
# 栈生命周期
# ==========================================
launch_stack() {
    local scenario_arg=""
    if [ -n "$SCENARIO_FILE" ]; then
        scenario_arg="scenario_file:=${SCENARIO_FILE}"
    fi
    local sensors_arg=""
    if [ -n "$PERCEPTION_SENSORS_FILE" ]; then
        sensors_arg="perception_sensors_file:=${PERCEPTION_SENSORS_FILE}"
    fi
    local world_arg=""
    if [ -n "$WORLD_FILE" ]; then
        world_arg="world_file:=${WORLD_FILE}"
    fi
    local dynamic_arg=""
    if [ "$DYNAMIC_OBSTACLES" = "true" ]; then
        dynamic_arg="dynamic_obstacles:=true"
    fi
    echo "[启动] 启动 agc_collab.launch (gui=${GUI_MODE}, takeoff_altitude=${TAKEOFF_ALTITUDE}, scenario=${SCENARIO_FILE:-默认}, target_source=${TARGET_SOURCE}, obstacle_source=${OBSTACLE_SOURCE}, dynamic_obstacles=${DYNAMIC_OBSTACLES})..."
    # shellcheck disable=SC2086
    roslaunch air_ground_core agc_collab.launch gui:="${GUI_MODE}" \
        takeoff_altitude:="${TAKEOFF_ALTITUDE}" \
        target_source:="${TARGET_SOURCE}" obstacle_source:="${OBSTACLE_SOURCE}" \
        follow_enabled:="${FOLLOW_MODE}" \
        mission_timeout_s:="${TIMEOUT_INJECT:-120.0}" \
        perception_fault_sensor:="${FAULT_PRESET}" perception_fault_bias_m:="${FAULT_PRESET_BIAS}" \
        run_seed:="${SCENE_SEED}" \
        ${dynamic_arg} ${world_arg} ${sensors_arg} ${scenario_arg} &
    LAUNCH_PID=$!
    echo "[启动] agc_collab PID: ${LAUNCH_PID}"
}

# 起飞 + 记录起飞门禁；成功打印 "connected armed pose_wait"，失败返回 1
takeoff_and_record() {
    wait_for_mavros 90 || return 1
    wait_for_clock 30 || true
    wait_for_service /drone_launch_controller/takeoff 60 || return 1

    local conn armed
    conn=$(rostopic echo /mavros/state -n 1 2>/dev/null | awk '/connected:/{print $2; exit}')
    armed=$(rostopic echo /mavros/state -n 1 2>/dev/null | awk '/armed:/{print $2; exit}')

    # 起飞前必须确认本地位姿可用（EKF 未收敛时起飞控制器会失败）
    echo "[等待] 等待 /mavros/local_position/pose 开始发布..."
    local pose_wait=0 pose_ok=0
    for i in $(seq 1 60); do
        if [ -n "$(timeout 5 rostopic echo /mavros/local_position/pose -n 1 2>/dev/null)" ]; then
            pose_wait=$i; pose_ok=1; break
        fi
        sleep 1
    done
    echo "[起飞] 发送起飞指令到 ${TAKEOFF_ALTITUDE}m (connected=${conn}, armed=${armed}, pose_ok=${pose_ok}, pose_wait=${pose_wait}s)..."
    rosservice call /drone_launch_controller/takeoff "altitude: ${TAKEOFF_ALTITUDE}
timeout: 30.0" || true

    echo "[等待] 等待起飞完成至 ${TAKEOFF_ALTITUDE}m..."
    local z="" t=0
    while [ $t -lt 100 ]; do
        z=$(timeout 5 rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | awk '/z:/{print $2; exit}')
        if [ -n "$z" ] && python3 -c "import sys; sys.exit(0 if abs($z-$TAKEOFF_ALTITUDE)<0.5 else 1)" 2>/dev/null; then
            echo "[等待] 起飞完成 (z=${z})"; break
        fi
        sleep 2; t=$((t+2))
    done
    echo "[起飞记录] connected=${conn} armed=${armed} pose_ok=${pose_ok} pose_wait_s=${pose_wait} target_alt=${TAKEOFF_ALTITUDE} final_z=${z}"
}

# 启动 LQR（交接点：LQR 自行调用 release_control）
start_lqr() {
    echo "[启动] 启动 LQR 轨迹跟踪器 (trajectory_type=step, step_z=${TAKEOFF_ALTITUDE})..."
    roslaunch lqr_trajectory_tracker lqr_tracker_sim.launch \
        rviz:=false trajectory_type:=step step_z:="${TAKEOFF_ALTITUDE}" \
        follow_enabled:="${FOLLOW_MODE}" &
    LQR_PID=$!
    echo "[启动] LQR Tracker PID: ${LQR_PID}"
}

# 等交接核查通过（M1 起为硬前置）；成功时 stdout 只输出等待秒数（进度走 stderr）
wait_handover() {
    local max_wait=${1:-60}
    echo "[等待] 等待空中控制权交接核查 /agc/air/handover_ok ..." >&2
    for i in $(seq 1 "$max_wait"); do
        local H
        H=$(rosparam get /agc/air/handover_ok 2>/dev/null || echo "")
        if [ "$H" = "True" ] || [ "$H" = "true" ]; then
            echo "[等待] 交接核查通过 handover_ok=true (${i}s)" >&2
            echo "$i"; return 0
        fi
        sleep 1
    done
    return 1
}

# 停止本脚本启动的栈
stop_stack() {
    echo "[清理] 正在停止本脚本启动的进程..."
    [ -n "$LQR_PID" ] && kill "$LQR_PID" 2>/dev/null || true
    [ -n "$LAUNCH_PID" ] && kill "$LAUNCH_PID" 2>/dev/null || true
    sleep 2
    for n in "${PROC_NAMES[@]}"; do
        kill_matching "$n"
    done
    sleep 3
    LQR_PID=""; LAUNCH_PID=""
}

# 信号处理：一组循环中被打断也尽量落盘并清理
on_signal() {
    echo ""
    echo "[中断] 收到终止信号，清理并落盘..."
    stop_stack
    if [ -n "$RECORDS" ] && [ -n "$BATCH_DIR" ]; then
        record_json "{\"type\": \"procs_after_exit\", \"procs\": $(proc_json)}"
        record_json "{\"type\": \"interrupted\", \"clean\": false}"
        finalize_batch || true
    fi
    exit 130
}
trap on_signal SIGINT SIGTERM

# ==========================================
# 批跑核心
# ==========================================
# 单 attempt：reset -> 等恢复 -> start -> 等终态；返回一行 JSON 记录
run_attempt() {
    local cycle="$1" attempt="$2"
    # M4：本 attempt 起点先清故障，避免上一 attempt 注入残留
    if [ -n "$INJECT_MODE" ]; then
        python3 "$WORKSPACE_DIR/src/air_ground_core/scripts/m4_inject.py" apply \
            --mode clear --sensor all --record "$BATCH_DIR" \
            --tag "pre_cycle${cycle}_attempt${attempt}" >/dev/null 2>&1 || true
    fi
    echo "[cycle ${cycle} attempt ${attempt}] 复位场景..."
    rosservice call /agc/scenario/reset >/dev/null 2>&1 || true
    local r="" reset_ok=0
    for _ in $(seq 1 20); do
        r=$(rosparam get /agc/reset/last_result 2>/dev/null || echo "")
        [ "$r" = "SUCCESS" ] && { reset_ok=1; break; }
        sleep 1
    done

    # 等空地链路恢复
    for _ in $(seq 1 25); do
        local av gv
        av=$(rosparam get /agc/air/odom_valid 2>/dev/null || echo "")
        gv=$(rosparam get /agc/ground/odom_valid 2>/dev/null || echo "")
        [ "$av" = "True" ] && [ "$gv" = "True" ] && break
        sleep 1
    done

    echo "[cycle ${cycle} attempt ${attempt}] 调用 /agc/task/start ..."
    local out start_ok=0 run_dir=""
    # run_dir 必须在 start 之前取：若 start 后立刻进终态，logger 会马上新开目录，
    # 那时再取就指到后继目录去了（本 attempt 的事件落在当前这个目录）
    run_dir=$(rosparam get /agc/log/run_dir 2>/dev/null || echo "")
    out=$(rosservice call /agc/task/start 2>&1 || true)
    case "$out" in *AIR_SEARCH*) start_ok=1 ;; esac

    # M4：start 成功后按模式注入感知故障（离线 rosparam 注入）
    local inject_note="none"
    if [ -n "$INJECT_MODE" ] && [ $start_ok -eq 1 ]; then
        # 延迟注入：先让目标被获得/跟踪，再施加故障（丢失/冲突场景更有意义）
        if [ "${INJECT_DELAY:-0}" != "0" ]; then
            echo "[inject] cycle ${cycle} attempt ${attempt}: 延迟 ${INJECT_DELAY}s 后注入 ${INJECT_MODE}"
            sleep "$INJECT_DELAY"
        fi
        local iv="${INJECT_VALUE}"
        if [ -z "$iv" ]; then
            case "$INJECT_MODE" in
                latency) iv="0.6" ;;
                loss)    iv="1.0" ;;
                conflict) iv="3.0" ;;
                *)       iv="0.0" ;;
            esac
        fi
        python3 "$WORKSPACE_DIR/src/air_ground_core/scripts/m4_inject.py" apply \
            --mode "$INJECT_MODE" --sensor "$INJECT_SENSOR" --value "$iv" \
            --record "$BATCH_DIR" --tag "cycle${cycle}_attempt${attempt}" >/dev/null 2>&1 || true
        inject_note="${INJECT_MODE}/${INJECT_SENSOR}/${iv}"
        echo "[inject] cycle ${cycle} attempt ${attempt}: mode=${INJECT_MODE} sensor=${INJECT_SENSOR} value=${iv}"
    fi

    # M5：外部写车位姿注入（验证 ground_drive_adapter 的写入权核查与 CONTROL_CONFLICT）
    if [ "$CONFLICT_INJECT" = "true" ] && [ $start_ok -eq 1 ]; then
        echo "[inject] cycle ${cycle} attempt ${attempt}: 延迟 ${CONFLICT_DELAY}s 后注入外部写车位姿 target=(${CONFLICT_TX},${CONFLICT_TY})"
        sleep "$CONFLICT_DELAY"
        python3 "$WORKSPACE_DIR/src/air_ground_core/scripts/m5_conflict_inject.py" \
            --model agc_car --target-x "$CONFLICT_TX" --target-y "$CONFLICT_TY" --rate 10 \
            --duration "$CONFLICT_DURATION" --record "$BATCH_DIR" \
            --tag "cycle${cycle}_attempt${attempt}" > "$BATCH_DIR/conflict_inject_cycle${cycle}_attempt${attempt}.log" 2>&1 || true
        inject_note="${inject_note}+writer(${CONFLICT_TX},${CONFLICT_TY})"
        echo "[inject] cycle ${cycle} attempt ${attempt}: 外部写车位姿注入结束"
    fi

    # N4：超时注入仅影响 run 前冻结参数，此处仅标注；交接异常注入在交接后终止 LQR
    if [ -n "$TIMEOUT_INJECT" ]; then
        if [ "$inject_note" = "none" ]; then inject_note="timeout(${TIMEOUT_INJECT})"; else inject_note="${inject_note}+timeout(${TIMEOUT_INJECT})"; fi
    fi
    if [ -n "$FAULT_PRESET" ]; then
        if [ "$inject_note" = "none" ]; then inject_note="preset_fault(${FAULT_PRESET}/${FAULT_PRESET_BIAS})"; else inject_note="${inject_note}+preset_fault(${FAULT_PRESET}/${FAULT_PRESET_BIAS})"; fi
    fi
    if [ -n "$HANDOVER_INJECT" ] && [ $start_ok -eq 1 ]; then
        echo "[inject] cycle ${cycle} attempt ${attempt}: 延迟 ${HANDOVER_INJECT}s 后终止 lqr_tracker_node"
        sleep "$HANDOVER_INJECT"
        rosnode kill /lqr_tracker_node >/dev/null 2>&1 || true
        if [ "$inject_note" = "none" ]; then inject_note="handover(kill_lqr)"; else inject_note="${inject_note}+handover(kill_lqr)"; fi
        echo "[inject] cycle ${cycle} attempt ${attempt}: 已终止 lqr_tracker_node"
    fi

    wait_terminal "${RUN_TIMEOUT}" || true

    local st rc reset_b="false" start_b="false"
    if [ $reset_ok -eq 1 ]; then reset_b="true"; fi
    if [ $start_ok -eq 1 ]; then start_b="true"; fi
    st=$(timeout 6 rostopic echo /agc/task/state -n 1 2>/dev/null | awk '/^state:/{print $2; exit}')
    # reason_code 是 string，rostopic 输出带引号；去掉引号后再拼 JSON
    rc=$(timeout 6 rostopic echo /agc/task/state -n 1 2>/dev/null | awk '/^reason_code:/{gsub(/"/,"",$2); print $2; exit}')
    echo "[cycle ${cycle} attempt ${attempt}] final_state=${st} reason=${rc} run_dir=${run_dir}"
    record_json "{\"type\": \"attempt\", \"cycle\": ${cycle}, \"attempt\": ${attempt}, \"reset_ok\": ${reset_b}, \"start_ok\": ${start_b}, \"final_state\": ${st:-null}, \"reason\": \"${rc}\", \"inject\": \"${inject_note}\", \"run_dir\": \"${run_dir}\"}"
}

# 一次「启动→退出」全循环
run_cycle() {
    local cycle="$1"
    echo "========== 全循环 ${cycle}/${CYCLES} =========="
    cleanup_procs
    launch_stack

    if ! takeoff_and_record; then
        record_json "{\"type\": \"cycle\", \"cycle\": ${cycle}, \"startup_ok\": false, \"reason\": \"TAKEOFF_FAILURE\"}"
        record_json "{\"type\": \"procs_before_exit\", \"cycle\": ${cycle}, \"procs\": $(proc_json)}"
        stop_stack
        record_json "{\"type\": \"procs_after_exit\", \"cycle\": ${cycle}, \"procs\": $(proc_json)}"
        return 1
    fi

    start_lqr

    # 编排层门禁修复：handover_ok 为 start 的硬前置，未通过不进入 attempt
    local hw=""
    if ! hw=$(wait_handover "$HANDOVER_TIMEOUT"); then
        echo "[警告] 未在 ${HANDOVER_TIMEOUT}s 内等到 handover_ok=true，按 startup 异常记录，不进入 attempt"
        record_json "{\"type\": \"cycle\", \"cycle\": ${cycle}, \"startup_ok\": false, \"reason\": \"HANDOVER_TIMEOUT\"}"
        record_json "{\"type\": \"procs_before_exit\", \"cycle\": ${cycle}, \"procs\": $(proc_json)}"
        stop_stack
        record_json "{\"type\": \"procs_after_exit\", \"cycle\": ${cycle}, \"procs\": $(proc_json)}"
        return 1
    fi

    # 接口现场核查（栈存活期间一次）
    local audit_ok="null"
    if python3 "$WORKSPACE_DIR/src/air_ground_core/scripts/audit_interfaces.py" --seconds 5 \
            > "$BATCH_DIR/audit_cycle${cycle}.log" 2>&1; then
        audit_ok="true"
    else
        audit_ok="false"
    fi
    cp -f "$RUNS_ROOT/audit_interfaces.json" "$BATCH_DIR/audit_interfaces_cycle${cycle}.json" 2>/dev/null || true

    record_json "{\"type\": \"cycle\", \"cycle\": ${cycle}, \"startup_ok\": true, \"handover_wait_s\": ${hw}, \"audit_ok\": ${audit_ok}}"

    local k
    for k in $(seq 1 "$REPEAT"); do
        run_attempt "$cycle" "$k" || true
    done

    record_json "{\"type\": \"procs_before_exit\", \"cycle\": ${cycle}, \"procs\": $(proc_json)}"
    stop_stack
    record_json "{\"type\": \"procs_after_exit\", \"cycle\": ${cycle}, \"procs\": $(proc_json)}"
    return 0
}

# 常驻服务模式（--serve）：一次「启动→起飞→LQR→交接」后空闲等待停止信号，不进入 attempt 循环。
# 每回合的 set 参数/复位/start/等终态由上层批跑器（d1_batch_runner.py --hot-swap）驱动，
# 从而避免每回合重启整套栈。退出条件：$BATCH_DIR/serve_stop 出现 或 收到 SIGTERM/SIGINT（trap 处理）。
run_serve() {
    echo "========== 常驻服务模式 (--serve) =========="
    cleanup_procs
    launch_stack

    if ! takeoff_and_record; then
        record_json "{\"type\": \"cycle\", \"cycle\": 1, \"startup_ok\": false, \"reason\": \"TAKEOFF_FAILURE\", \"serve\": true}"
        record_json "{\"type\": \"procs_before_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
        stop_stack
        record_json "{\"type\": \"procs_after_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
        echo "serve_ready=false"
        return 1
    fi

    start_lqr

    # 交接核查为硬前置：未通过则视为未就绪，干净退出
    local hw=""
    if ! hw=$(wait_handover "$HANDOVER_TIMEOUT"); then
        echo "[警告] 未在 ${HANDOVER_TIMEOUT}s 内等到 handover_ok=true，常驻模式不就绪"
        record_json "{\"type\": \"cycle\", \"cycle\": 1, \"startup_ok\": false, \"reason\": \"HANDOVER_TIMEOUT\", \"serve\": true}"
        record_json "{\"type\": \"procs_before_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
        stop_stack
        record_json "{\"type\": \"procs_after_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
        echo "serve_ready=false"
        return 1
    fi

    # 栈就绪 + 交接通过：打印 batch_dir/serve_ready 供批跑器解析
    record_json "{\"type\": \"cycle\", \"cycle\": 1, \"startup_ok\": true, \"handover_wait_s\": ${hw}, \"serve\": true}"
    echo "batch_dir=${BATCH_DIR}"
    echo "serve_ready=true"

    # 空闲等待停止信号文件（signal 由 on_signal trap 处理）
    local STOP_FILE="$BATCH_DIR/serve_stop"
    while [ ! -f "$STOP_FILE" ]; do
        sleep 2
    done
    echo "[serve] 检测到停止信号文件 ${STOP_FILE}，开始干净收尾"

    record_json "{\"type\": \"procs_before_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
    stop_stack
    record_json "{\"type\": \"procs_after_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
    return 0
}

# 把 records.jsonl 归并成 batch.json（inline python 保证合法 JSON）
finalize_batch() {
    [ -n "$RECORDS" ] && [ -f "$RECORDS" ] || { echo "[警告] 无 records，跳过 batch.json"; return 0; }
    python3 - "$BATCH_DIR" "$RECORDS" <<'PY'
import json, os, sys
batch_dir, records_path = sys.argv[1], sys.argv[2]
recs = []
with open(records_path) as f:
    for line in f:
        line = line.strip()
        if line:
            recs.append(json.loads(line))
batch = next((r for r in recs if r.get("type") == "batch"), {"type": "batch"})
cycles = {}
order = []
for r in recs:
    if r.get("type") == "cycle":
        c = r["cycle"]
        if c not in cycles:
            cycles[c] = {"cycle": c, "attempts": []}
            order.append(c)
        cycles[c].update({k: v for k, v in r.items() if k != "type"})
    elif r.get("type") == "attempt":
        c = r["cycle"]
        cycles.setdefault(c, {"cycle": c, "attempts": []})
        if c not in order:
            order.append(c)
        cycles[c]["attempts"].append({k: v for k, v in r.items() if k != "type"})
    elif r.get("type") in ("procs_before_exit", "procs_after_exit"):
        c = r["cycle"]
        cycles.setdefault(c, {"cycle": c, "attempts": []})
        if c not in order:
            order.append(c)
        key = "procs_before_exit" if r["type"] == "procs_before_exit" else "procs_after_exit"
        cycles[c][key] = r["procs"]
        if key == "procs_after_exit":
            cycles[c]["leftovers"] = r["procs"]
            cycles[c]["clean_exit_no_leftover"] = (len(r["procs"]) == 0)
out = dict(batch)
out["cycles"] = [cycles[c] for c in order]
out["any_leftover"] = any(not c.get("clean_exit_no_leftover", True) for c in out["cycles"])
with open(os.path.join(batch_dir, "batch.json"), "w") as f:
    json.dump(out, f, ensure_ascii=False, indent=2)
print("[批跑] batch.json 已写入: %s  (any_leftover=%s)" % (os.path.join(batch_dir, "batch.json"), out["any_leftover"]))
PY
}

# ==========================================
# 兼容旧用法：--auto-run 单样例
# ==========================================
run_sample() {
    local sample="$1"
    echo "[样例] 样例名: ${sample}"
    wait_for_service /agc/scenario/reset 30 || true

    local ok=1
    for k in $(seq 1 3); do
        local r=""
        rosservice call /agc/scenario/reset >/dev/null 2>&1 || true
        for _ in $(seq 1 20); do
            r=$(rosparam get /agc/reset/last_result 2>/dev/null || echo "")
            [ "$r" = "SUCCESS" ] && break
            sleep 1
        done
        echo "[样例] 第 ${k} 次复位 last_result=${r}；等待空地 odom 恢复..."
        for _ in $(seq 1 25); do
            local av gv
            av=$(rosparam get /agc/air/odom_valid 2>/dev/null || echo "")
            gv=$(rosparam get /agc/ground/odom_valid 2>/dev/null || echo "")
            [ "$av" = "True" ] && [ "$gv" = "True" ] && break
            sleep 1
        done
        echo "[样例] 调用 /agc/task/start ..."
        local out
        out=$(rosservice call /agc/task/start 2>&1)
        echo "$out"
        case "$out" in
            *AIR_SEARCH*) ok=0; break ;;
        esac
        echo "[样例] 门禁未通过，5s 后重试（需先重新复位）"
        sleep 5
    done
    if [ "$ok" != "0" ]; then
        echo "[样例] 连续 3 次未通过 start 门禁，结束本样例"
        return 1
    fi

    if [ "$sample" != "success" ]; then
        for _ in $(seq 1 30); do
            local S
            S=$(timeout 6 rostopic echo /agc/task/state -n 1 2>/dev/null | awk '/^state:/{print $2; exit}')
            case "$S" in 2|3) break ;; esac
            sleep 2
        done
    fi

    case "$sample" in
        success)        echo "[样例] 不做注入，观察抵近并持续跟踪" ;;
        mission_failed) echo "[样例] 注入观测丢失：/agc/target/enabled=false"
                        rosparam set /agc/target/enabled false ;;
        infra_failure)  echo "[样例] 注入空中链路失效：停止 air_odom_adapter"
                        pkill -9 -f "air_odom_adapter" 2>/dev/null || true ;;
        aborted)        echo "[样例] 注入人工急停：/agc/task/abort"
                        rosservice call /agc/task/abort ;;
        *) echo "[错误] 未知样例: ${sample}"; return 1 ;;
    esac

    wait_terminal "${RUN_TIMEOUT}" || true
    echo "[样例] 最终任务状态："
    timeout 6 rostopic echo /agc/task/state -n 1 2>/dev/null | head -20 || true
    echo "[样例] run 目录: $(rosparam get /agc/log/run_dir 2>/dev/null || echo unknown)"
}

# ==========================================
# 主流程
# ==========================================
if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    show_help
    exit 0
fi
while [ $# -gt 0 ]; do
    case "$1" in
        --takeoff-altitude) TAKEOFF_ALTITUDE="$2"; shift 2 ;;
        --duration)         DURATION="$2"; shift 2 ;;
        --scenario)         SCENARIO_FILE="$2"; shift 2 ;;
        --sensors-file)     PERCEPTION_SENSORS_FILE="$2"; shift 2 ;;
        --scene-seed)       SCENE_SEED="$2"; shift 2 ;;
        --world-file)       WORLD_FILE="$2"; shift 2 ;;
        --target-source)    TARGET_SOURCE="$2"; shift 2 ;;
        --obstacle-source)  OBSTACLE_SOURCE="$2"; shift 2 ;;
        --inject)           INJECT_MODE="$2"; shift 2 ;;
        --inject-sensor)    INJECT_SENSOR="$2"; shift 2 ;;
        --inject-value)     INJECT_VALUE="$2"; shift 2 ;;
        --inject-delay)     INJECT_DELAY="$2"; shift 2 ;;
        --timeout-inject)   TIMEOUT_INJECT="$2"; shift 2 ;;
        --handover-inject)  HANDOVER_INJECT="$2"; shift 2 ;;
        --fault-preset)     FAULT_PRESET="$2"; shift 2 ;;
        --fault-preset-value) FAULT_PRESET_BIAS="$2"; shift 2 ;;
        --conflict-inject)  CONFLICT_INJECT="true"; shift ;;
        --conflict-target-x) CONFLICT_TX="$2"; shift 2 ;;
        --conflict-target-y) CONFLICT_TY="$2"; shift 2 ;;
        --conflict-delay)   CONFLICT_DELAY="$2"; shift 2 ;;
        --conflict-duration) CONFLICT_DURATION="$2"; shift 2 ;;
        --batch-prefix)     BATCH_PREFIX="$2"; shift 2 ;;
        --repeat)           REPEAT="$2"; BATCH_MODE="true"; shift 2 ;;
        --cycles)           CYCLES="$2"; BATCH_MODE="true"; shift 2 ;;
        --sample)           SAMPLE="$2"; shift 2 ;;
        --auto-run)         AUTO_RUN="$2"; shift 2 ;;
        --run-timeout)      RUN_TIMEOUT="$2"; shift 2 ;;
        --gui)              GUI_MODE="true"; shift ;;
        --follow)           FOLLOW_MODE="true"; shift ;;
        --serve)            SERVE_MODE="true"; shift ;;
        --dynamic-obstacles) DYNAMIC_OBSTACLES="true"; shift ;;
        -h|--help)          show_help; exit 0 ;;
        *) echo "[错误] 未知选项: $1"; show_help; exit 1 ;;
    esac
done

setup_env

# 批跑目录（下划线开头，避免被 summarize_runs.py 当成 run 目录）
BATCH_ID="$(date -u +%Y%m%dT%H%M%S)"
BATCH_DIR="$RUNS_ROOT/${BATCH_PREFIX}${BATCH_ID}"
mkdir -p "$BATCH_DIR"
RECORDS="$BATCH_DIR/records.jsonl"
: > "$RECORDS"
record_json "{\"type\": \"batch\", \"batch_id\": \"${BATCH_ID}\", \"cycles_requested\": ${CYCLES}, \"attempts_per_cycle\": ${REPEAT}, \"scenario_file\": \"${SCENARIO_FILE:-<package default>}\", \"sensors_file\": \"${PERCEPTION_SENSORS_FILE}\", \"scene_seed\": \"${SCENE_SEED}\", \"world_file\": \"${WORLD_FILE}\", \"target_source\": \"${TARGET_SOURCE}\", \"obstacle_source\": \"${OBSTACLE_SOURCE}\", \"inject_mode\": \"${INJECT_MODE}\", \"inject_sensor\": \"${INJECT_SENSOR}\", \"inject_value\": \"${INJECT_VALUE}\", \"conflict_inject\": \"${CONFLICT_INJECT}\", \"conflict_target\": \"${CONFLICT_TX},${CONFLICT_TY}\", \"takeoff_altitude\": ${TAKEOFF_ALTITUDE}, \"gui\": ${GUI_MODE}, \"start_wall\": $(date +%s)}"

if [ "$SERVE_MODE" = "true" ]; then
    # 常驻服务模式：一次启动，空闲等待上层逐回合驱动
    echo "[常驻] serve 模式，batch_dir=${BATCH_DIR}"
    run_serve || true
elif [ "$BATCH_MODE" = "true" ]; then
    echo "[批跑] cycles=${CYCLES} attempts_per_cycle=${REPEAT}，batch_dir=${BATCH_DIR}"
    c=0
    while [ $c -lt "$CYCLES" ]; do
        c=$((c+1))
        run_cycle "$c" || true
    done
elif [ -n "$AUTO_RUN" ]; then
    echo "[auto-run] 单样例模式：${AUTO_RUN}，batch_dir=${BATCH_DIR}"
    cleanup_procs
    launch_stack
    if takeoff_and_record; then
        start_lqr
        hw=$(wait_handover "$HANDOVER_TIMEOUT") || hw="timeout"
        echo "[auto-run] handover_wait_s=${hw}"
        run_sample "$AUTO_RUN" || true
    else
        echo "[auto-run] 起飞失败，跳过样例"
    fi
    record_json "{\"type\": \"procs_before_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
    stop_stack
    record_json "{\"type\": \"procs_after_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
else
    # 原始用法：启动就绪后运行 DURATION 秒再干净退出（同时做残留核查）
    cleanup_procs
    launch_stack
    if takeoff_and_record; then
        start_lqr
        wait_handover "$HANDOVER_TIMEOUT" >/dev/null || echo "[警告] 交接核查未通过"
    fi
    echo "[运行] 协同 run 已就绪，${DURATION}s 后自动退出 (Ctrl+C 可提前结束)..."
    sleep "$DURATION"
    record_json "{\"type\": \"procs_before_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
    stop_stack
    record_json "{\"type\": \"procs_after_exit\", \"cycle\": 1, \"procs\": $(proc_json)}"
fi

# 收尾：残留核查 + batch.json
LEFTOVERS="$(proc_json)"
echo "[退出核查] 退出后关键进程残留: ${LEFTOVERS}"
finalize_batch

# 聚合口径
echo "[汇总] 运行 summarize_runs.py ..."
python3 "$WORKSPACE_DIR/src/air_ground_core/scripts/summarize_runs.py" || true