# Checklist

> 逐项核查 [spec.md](file:///home/ubuntu/anti_drone_agc_workspace/.trae/specs/implement-agc-m0-contract-v11/spec.md) 与 [tasks.md](file:///home/ubuntu/anti_drone_agc_workspace/.trae/specs/implement-agc-m0-contract-v11/tasks.md)，按契约「可核查证据」口径判定。
> 核查方式：现场无头实跑（`scripts/start_agc_collab.sh`）+ `audit_interfaces.py` 现场核查 + `summarize_runs.py` 汇总 + 代码路径核对。证据索引见 `tasks.md` 末节。

## 消息与包

- [x] `TargetTrack.msg` / `TargetTrackArray.msg` / `AgcTaskState.msg` 字段与契约逐字一致（含 9 个状态常量、NaN 语义、`source` 取值注释）
- [x] `catkin build air_ground_core` 通过，`rosmsg show` 三条消息可查
- [x] `car_commander` 不再被任何 launch 启动，可执行目标已从构建移除且源码标注废弃（全仓 launch/脚本中仅剩注释提及；devel 目录无对应可执行文件）

## 世界、时钟与 TF

- [x] 协同 run 只有一个 gzserver / 一个 Gazebo 命名空间，不出现 `ground_gazebo`（audit：仅 `/gazebo`）
- [x] 世界内同时存在无人机、地面车、目标（默认 z=2.0）与场景障碍（`/gazebo/model_states` = iris / agc_car / intruder_uav / pole_1 / pole_2）
- [x] 全局 `/use_sim_time=true`，`/clock` 只有一个发布者
- [x] `ground_sim.launch` 的 `use_sim_time` 已参数化（默认 false，仅地面单体用），协同 run 不 include 它
- [x] TF 存在且闭合：`world`、`air_local`、`drone/base_link`、`car/base_link`；无裸 `base_link`（采样 551 帧 `/tf`，child 含裸 `base_link` = False）
- [x] `air_odom_adapter` 每次启动/复位记录 `T_world_air_local`、时间与残差（`air_calibration.json`）

## 接口与发布者

- [x] `/agc/air/odom`、`/agc/ground/odom` 频率 ≥20 Hz、年龄 ≤0.5 s，`frame_id=world`、`child_frame_id` 分别为 `drone/base_link`/`car/base_link`，位置在 world、twist 在 child frame（实测 30.2 / 30.2 Hz）
- [x] `/agc/ground/odom` 来自 Gazebo 实际车体状态，未从 `CarStatus` 位置或无 Header 字段冒充（audit：与 `model_states` 距离差 0.0000 m；代码仅订阅 `/gazebo/model_states`）
- [x] `/agc/targets` 10 Hz、唯一源 `gt_adapter`、`confidence=1`，无目标发空数组且重发不刷新 `stamp`（实测 10.11 Hz；空数组仅出现在观测关闭后）
- [x] `/agc/map` ≥1 Hz 且变更即发，含原点/分辨率/尺寸与内容哈希版本；空图判为无效（实测 2.12 Hz，`/agc/map/version_hash` 非空）
- [x] `/agc/ground/goal` ≥5 Hz 且变化即发、年龄 ≤1 s、world 且 z=0（实测 6.99 Hz；goal=(6.50,2.50,0)）
- [x] `/agc/ground/path` ≥1 Hz 且重规划即发、年龄 ≤2 s、非空（实测 2.00 Hz；67 条路径全部非空）
- [x] `/agc/ground/cmd_vel` 20 Hz 含零速、年龄 ≤0.2 s、`header.frame_id=car/base_link`（实测 20.09 Hz）
- [x] `/agc/task/state` 2 Hz 且变化即发；无目标时 `target_xy_distance_m`/`target_obs_age_s` 为 NaN（实测 2.00 Hz，frame=world）
- [x] `/agc/task/start` 仅在 IDLE 且门禁通过时成功；`/agc/task/abort` 对任意非终态有效（门禁未过 → `STARTUP_FAILURE`；abort → `ABORTED`）
- [x] 每个 `/agc/` 话题的实际消息发布者唯一且与契约表一致（audit 逐话题核对发布者集合，全部 PASS）

## 执行与控制权

- [x] 正向、转向、零速 cmd_vel 可使车按 `kinematic_sim` 运动与停车（车从 (-4,-3) 行驶 11.9 m 到目标投影附近并在 goal 处零速；max_v=1.0 m/s）
- [x] 任务运行期只有 `ground_drive_adapter` 写本方车位姿；`ground_env`、`car_launch_controller` 写位姿循环未运行（协同 launch 不启动它们；odom 与 Gazebo 位姿零差）
- [x] 命令断开 ≤0.2 s 后 `ground_drive_adapter` 置零速并记录 watchdog 事件（杀掉 `ground_commander` 后 `/agc/drive/watchdog_count` 0→117，车 3 s 内位移 0.0000 m，`drive_log.jsonl` 记录 `watchdog=true`）
- [x] `scenario_reset` 只在任务非运行期写位姿，复位失败记 `RESET_FAILURE` 且不进任务失败分子（运行中调用返回 `success=False, "task running: GROUND_APPROACH"`）
- [x] 目标/地图/路径过期或进入终态时 `ground_commander` 立即发零速（观测丢失后 reason→`goal_stale` 并零速；终态置 `force_stop`）
- [x] 起飞→`release_control`→LQR 交接无 setpoint 空窗（最大发布间隔 <0.2 s），发布者实际切换（交接空窗实测 0.036–0.052 s；`publishers_before=[drone_launch_controller]` → `after=[...,lqr_tracker_node]`）
- [x] 终止/失联时空中控制者发布经验证安全 hold，未归零偏移后沿旧参考飞行（`step` 参考为固定悬停点，bridge 复核 `hold_ok=true`，最大偏移在容差内）

## 状态机与判据

- [x] 门禁未通过不进入 `AIR_SEARCH`，记 `INFRA_FAILURE`（实测 `IDLE → INFRA_FAILURE / STARTUP_FAILURE`，并列出 7 项门禁逐项结果）
- [x] `TARGET_ACQUIRED` 规划连续失败有界（`bound = min(plan_retry_max × replan_period, approach_timeout)`，到点结算 `PLAN_UNREACHABLE`）
- [x] 抵近跟踪 5.0 s 连续窗口四条件同时成立才判 `TRACKING_SUCCESS`；中断即清零（3 次 run 判 `TRACKING_SUCCESS`，`tracking_milestone_met=true`，`tracking_frames.jsonl` 逐帧记录距离/净空/车速/目标速/相对速度）
- [x] 目标丢失 >2 s 先停车再结算 `TARGET_LOST`（注入观测丢失的 run：`goal_stale` 零速 → `MISSION_FAILED / TARGET_LOST`）
- [x] 同拍结算优先级为 急停>已验证碰撞>基础设施失败>成功>超时，其余事件完整落盘（按 rank 结算，其余事件照常写 `state_events.jsonl`）
- [x] 保护距离违例记 `CLEARANCE_VIOLATION`（不叫碰撞）；紧急距离触发单记 `EMERGENCY_TRIGGER`（代码路径核对；四类样例中未触发，`aggregate.json` 事件计数为 0）
- [x] 终态只能由下一次 run 的复位退出（`terminal_epoch` 与 `/agc/reset/epoch` 比较；实测每次重跑都必须先复位才允许 start）

## 规划与日志

- [x] 重规划触发条件（位移>1 m / 地图版本变化 / 路径不可行 / ≥2 s）均实现，且保存触发原因、输入与地图版本、耗时、路径与失败原因（`planner_log.jsonl` 含 `first`/`period` 等 trigger 与 `map_version_hash`）
- [x] 每次 run 保存 `run_id`、契约/代码/SDF/场景哈希、目标源与后端、参数与种子、仿真与单调墙钟时间（`meta.json`：`contract_doc/agc_params_hash/scenario_hash/world_hash/model_hashes/scripts_hash/git_commit/target_source=gt_adapter/drive_backend=kinematic_sim/seed=unseeded`）
- [x] run 日志含 TF 校准、空地 odom、目标、地图、goal、path、cmd、实际轨迹、状态事件、控制交接、启动/复位/清理结果（`air_calibration.json`、`air_handover.json`、`topics/*.jsonl`、`drive_log.jsonl`、`target_trajectory.jsonl`、`state_events.jsonl`、`reset_log.jsonl`、`summary.json`）
- [x] 汇总分母自洽：`valid_task_runs = tracking_success_runs + mission_failure_runs`，`attempted_runs = valid_task_runs + infra_failure_runs + aborted_runs`（`aggregate.json` 两条等式 PASS：4=3+1，6=4+1+1）
- [x] 报告含任务成功率与基础设施失败/全部尝试、人工终止/全部尝试；真实接触、几何相交、保护距离违例、过滤器触发分列（0.75 (3/4)、0.1667 (1/6)、0.1667 (1/6)；四类事件计数分列）

## 端到端与口径

- [x] 至少各有一个成功、任务失败、基础设施失败、人工 abort 的可回放样例（`20260926T081303_cd34a5` / `...081432_1ec90d` / `...081548_6be25d` / `...081700_fc960d`）
- [x] 因果闭环：固定起点与地图、只改目标/障碍输入，goal/path/cmd/实际轨迹可解释变化；无信息时安全停车（目标可用：goal=(6.5,2.5)、路径 11.21 m、cmd max 1.0 m/s、车行驶 11.9 m 到 0.448 m；观测关闭：目标数组转空 → `goal_stale` → 零速停车，全程最近距离 10.83 m）
- [x] 重复性：同场景连续多次 run 均完整退出、每次尝试有日志、退出后无本次遗留进程（6 次 attempted run 全部正常退出；`summarize_runs.py` 无 `invalid_logs`；实跑结束后 `pgrep` 确认无 gzserver/mavros/rosmaster/协同节点残留）
- [x] `TRACKING_SUCCESS` 措辞为“抵近并持续跟踪”，未表述为拦截完成；全程未出现 `INTERCEPT_CONFIRMED`
- [x] 无人机单体链路（`start_sim.sh`、LQR、感知/DQN）行为未被改变
  - 说明：为满足契约「交接期不得留 0.2 s setpoint 空窗」，对无人机侧做了两处**时序**微调：① 切 OFFBOARD 期间与重试等待期间持续发布 setpoint（此前空窗会触发 PX4 掉出 OFFBOARD → AUTO.LAND）；② `release_control` 后的等待 0.5 s → 0.05 s。控制律、轨迹、参数与实验口径未改动。
  - 复核：`bash scripts/start_sim.sh lqr --altitude 2.0 --trajectory step --trajectory-duration 30 --headless` 仍正常起飞（z=1.97）并进入 LQR 跟踪。

## 未在本轮验证的项（如实登记，不作为已验收）

- [ ] 阶段 B 拦截机构、指定接触判据与 `INTERCEPT_CONFIRMED`（按契约为非目标，需另行冻结方案）
- [ ] `air_fusion` 真实融合与多源切换（本轮只启用 `gt_adapter` 真值源；接口与 `source` 字段已就位）
- [ ] `dynamic_sim` / `hardware` 后端与实物、动力学表述（非目标）
- [ ] M7 单体性能量化、M8 硬件联调（非目标）
