# 按 M0 接口契约 v1.1 改造项目 Spec

> 依据：[M0_接口契约_v1.1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E4%BB%BB%E5%8A%A1%E6%97%A5%E5%BF%97/M0/M0_%E6%8E%A5%E5%8F%A3%E5%A5%91%E7%BA%A6_v1.1.md)（版本 `agc-m0-v1.1`）。
> 本文只覆盖该契约的**接口层 + 第一条端到端闭环**，不覆盖阶段 B 拦截、M4 真实融合、M7/M8。

## Why

工作区已有无人机（PX4/MAVROS/Gazebo + LQR）与地面车（独立 `ground_gazebo` + RL 环境）两条**互相独立**的单体链路。`air_ground_core` 目前仅有骨架节点 `car_commander.cpp`（持续发布固定画圆速度），不存在任何 `/agc/` 话题、协同消息、适配器或状态机，也没有共同世界与统一时空基准。

契约 v1.1 已冻结协同层的唯一任务定义、坐标系与时间、话题/消息、控制所有权、状态机与结果口径。本次改造把项目从「两套单体仿真」推进到「同一世界、同一时钟、唯一执行口、可核查结局」的第一条端到端闭环，并消除已知硬冲突。

## What Changes

- 在 `src/air_ground_core/` 内落地协同层：3 个新消息、8 个 `/agc/` 话题、2 个 Trigger 服务、适配器（空中 odom / 地面 odom / 真值目标 / 地图）、`task_manager` 状态机、`ground_planner`、`ground_commander`、`ground_drive_adapter`、`intruder_driver`、`scenario_reset`、run 日志。
- 新建唯一共同世界 `agc_world`：同一 gzserver 内含本方无人机、地面车、目标 `intruder_uav`（默认 z=2.0）与场景障碍；协同 run 不再启动独立 `ground_gazebo`。
- 新建协同启动入口 `agc_collab.launch`（无头），并新增 `scripts/start_agc_collab.sh`。
- **BREAKING**（仅协同 run）：`car_commander` 不再是地面执行口；`car_launch_controller` 与 `ground_env` 的 `set_model_state` 写位姿循环在任务运行期停用，地面唯一执行口为 `/agc/ground/cmd_vel`，唯一运动写入者为 `ground_drive_adapter`。
- **BREAKING**：`rl_ground_avoidance/launch/ground_sim.launch` 中强制 `use_sim_time=false` 的写法改为可配置参数（默认保持地面单体实验行为），协同 run 不 include 它。
- 修复 TF 裸 `base_link` 重名：机体帧统一为 `drone/base_link`、`car/base_link`。
- 地面任务终态统一为 `TRACKING_SUCCESS`（“抵近并持续跟踪”），**不得**表述为拦截完成；阶段 A 禁用 `INTERCEPT_CONFIRMED`。

## Impact

- Affected specs：`/agc/` 协同能力（新增）；地面执行后端与 TF/时钟基准（修改）；`car_commander` 执行口（移除）。
- Affected code：
  - `src/air_ground_core/`（msg / src / scripts / launch / config / worlds / models，主要新增）
  - `src/rl_ground_avoidance/launch/ground_sim.launch`、`src/rl_ground_avoidance/scripts/ground_env.py`（时钟与写位姿权）
  - `src/car_launch_controller/`（任务运行期停用写位姿循环）
  - `scripts/`（新增协同启动脚本；`start_sim.sh` 保持无人机单体入口不变）
  - `docs/项目指导书/下一阶段任务清单.md`、`docs/项目任务日志/M0/`（状态登记）
- 不改动：无人机单体实验链路（`start_sim.sh`、LQR、感知/DQN）与既有论文实验资产的行为；`ground_sim.launch` 的地面单体语义保留。

## ADDED Requirements

### Requirement: 唯一共同世界与全局仿真时钟
协同 run SHALL 只使用一个 gzserver 与一个世界 `agc_world`，其中同时存在本方无人机、地面车、目标与场景障碍；全局 `/use_sim_time=true`，任务消息 stamp、观测年龄与任务时限 SHALL 取自 `/clock`。

#### Scenario: 单 world 启动核查
- **WHEN** 协同 run 启动完成
- **THEN** 全系统只有一个 gzserver 进程与一个 Gazebo 命名空间，不出现 `ground_gazebo` 节点
- **AND** `/clock` 只有一个发布者且 `/use_sim_time` 为 true

#### Scenario: 停钟不累积任务时长
- **WHEN** `/clock` 停滞
- **THEN** 成功窗口计时不增长，且由单调墙钟判定为 `CLOCK_STALL` 基础设施失败

### Requirement: 坐标、TF 与消息时间语义
系统 SHALL 以 Gazebo `world`（ENU：+x 东、+y 北、+z 上；位置 m、速度 m/s、角度 rad）为唯一基准；TF 名称 SHALL 为 `world`、`air_local`、`drone/base_link`、`car/base_link`，不得发布裸 `base_link`。含 Header 的协同位置/速度/地图/路径/状态消息 `frame_id=world`；`/agc/ground/cmd_vel` 用量纲为车体系的 `TwistStamped`。

#### Scenario: odom 语义正确
- **WHEN** 读取 `/agc/air/odom` 与 `/agc/ground/odom`
- **THEN** 二者 `header.frame_id=world`、`child_frame_id` 分别为 `drone/base_link` / `car/base_link`，位置在 world 系，`twist` 表达在对应 child frame

#### Scenario: 空中本地位姿标定
- **WHEN** 空中链路每次启动或复位
- **THEN** `air_odom_adapter` 标定并记录 `T_world_air_local`、时间与残差，且不假定本地原点恒等于 spawn 点

### Requirement: 协同层话题与服务
系统 SHALL 提供下表接口，类型、唯一发布者、更新频率与年龄上限与契约一致；每个话题只能有一个实际发布者。

| 名称 | 类型 | 唯一发布者 | 频率 / 年龄 |
| --- | --- | --- | --- |
| `/agc/air/odom` | `nav_msgs/Odometry` | `air_odom_adapter` | ≥20 Hz，≤0.5 s |
| `/agc/ground/odom` | `nav_msgs/Odometry` | `ground_odom_adapter` | ≥20 Hz，≤0.5 s |
| `/agc/targets` | `air_ground_core/TargetTrackArray` | `gt_adapter` | 10 Hz，无目标发空数组 |
| `/agc/map` | `nav_msgs/OccupancyGrid` | `map_adapter` | ≥1 Hz 且变更即发 |
| `/agc/ground/goal` | `geometry_msgs/PoseStamped` | `task_manager` | ≥5 Hz 且变化即发，≤1 s，z=0 |
| `/agc/ground/path` | `nav_msgs/Path` | `ground_planner` | ≥1 Hz 且重规划即发，≤2 s，非空 |
| `/agc/ground/cmd_vel` | `geometry_msgs/TwistStamped` | `ground_commander` | 20 Hz（含零速），≤0.2 s |
| `/agc/task/state` | `air_ground_core/AgcTaskState` | `task_manager` | 2 Hz 且变化即发，≤1.5 s |
| `/agc/task/start`、`/agc/task/abort` | `std_srvs/Trigger` | `task_manager` | start 仅 IDLE 且门禁通过；abort 对任意非终态有效 |

#### Scenario: 发布者唯一
- **WHEN** 对每个 `/agc/` 话题核查实际消息发布者
- **THEN** 每个话题只有一个发布者，且与上表一致

### Requirement: 协同消息定义
系统 SHALL 新增 `air_ground_core/TargetTrack.msg`、`TargetTrackArray.msg`、`AgcTaskState.msg`，字段与契约完全一致。

```text
# TargetTrack.msg
std_msgs/Header header                 # 最后一次真实观测时刻；frame_id=world
string target_id                       # 阶段 A 为 intruder_uav
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
float32 confidence                     # 来源内已校准的观测质量
string source                          # gt_adapter / air_fusion

# TargetTrackArray.msg
std_msgs/Header header                 # 数组发布时刻；frame_id=world
TargetTrack[] tracks                   # 未观测到有效目标时为空

# AgcTaskState.msg：header(world,/clock)、state、state_name、reason_code、
# air_controller、ground_controller、target_id、target_xy_distance_m(无目标 NaN)、
# target_obs_age_s(无目标 NaN)、tracking_milestone_met
# 状态常量：IDLE=0 AIR_SEARCH=1 TARGET_ACQUIRED=2 GROUND_APPROACH=3
#           TRACKING_SUCCESS=4 MISSION_FAILED=5 ABORTED=6
#           INTERCEPT_CONFIRMED=7（阶段 B 保留，阶段 A 禁用）INFRA_FAILURE=8
```

#### Scenario: 重发不刷新观测时间
- **WHEN** 目标源重发同一 track
- **THEN** `Track.header.stamp` 保持最后一次真实观测时刻，不因重发而变新

#### Scenario: 阶段 A 禁用拦截态
- **WHEN** 协同 run 结束于抵近跟踪成功
- **THEN** `state=TRACKING_SUCCESS`，不出现 `INTERCEPT_CONFIRMED`

### Requirement: 目标观测来源固定为真值适配器
阶段 A 本次实现 SHALL 只启用 `gt_adapter` 作为唯一目标源（`source=gt_adapter`，无噪声基线 `confidence=1`，协方差取已注明精度），来源在 run 开始前固定，不可中途切换；`air_fusion` 仅保留接口不实现。

#### Scenario: 无目标发空数组
- **WHEN** 场景中目标不存在或已被移出
- **THEN** `/agc/targets` 发布空 `tracks` 数组，不复用旧姿态换新 stamp

### Requirement: 地面执行后端与唯一运动写入者
地面执行后端 SHALL 为 `kinematic_sim`：仅 `ground_drive_adapter` 可写本方车位姿，由它积分命令并调用 Gazebo `set_model_state`；`ground_drive_adapter` SHALL 独立检测命令年龄 >0.2 s 并置零速，且上报服务调用失败。`scenario_reset` SHALL 只在任务非运行期写本方车位姿。结果表述 SHALL 为“运动学仿真”，不得称动力学或实物验证。

#### Scenario: 断开命令停车
- **WHEN** `ground_commander` 停止发布超过 0.2 s
- **THEN** `ground_drive_adapter` 置零速并记录 watchdog 事件

#### Scenario: 写入权唯一
- **WHEN** 任务运行期核查车体写入者
- **THEN** 只有 `ground_drive_adapter` 在写本方车位姿；`car_commander`、`car_launch_controller` 写位姿循环、`ground_env` 均未运行

### Requirement: 状态机、门禁与失败分类
`task_manager` SHALL 实现契约状态机与门禁，并按契约原因码分类失败；终态只能由下一次 run 的复位退出。

```text
IDLE --start且门禁通过--> AIR_SEARCH --有效观测连续1 s--> TARGET_ACQUIRED
TARGET_ACQUIRED --目标/地图/车位姿有效且路径可用--> GROUND_APPROACH
GROUND_APPROACH --连续抵近跟踪窗口达成--> TRACKING_SUCCESS
任一非终态 --任务条件失败--> MISSION_FAILED
任一非终态 --人工急停--> ABORTED
任一非终态 --复位/时钟/TF/执行设施失败--> INFRA_FAILURE
```

#### Scenario: 门禁未通过
- **WHEN** start 时单 world / 时钟 / TF / 空地 odom / 目标与地图源 / 地面后端 / 空中起飞交接 / 场景复位任一项不满足
- **THEN** 不进入 `AIR_SEARCH`，记 `INFRA_FAILURE`（reason 属 `STARTUP_FAILURE` 等基础设施码）

#### Scenario: 目标丢失先停车
- **WHEN** 曾获得目标后观测连续丢失 >2 s
- **THEN** 先停车，再结算 `MISSION_FAILED` / `TARGET_LOST`

#### Scenario: 同拍结算优先级
- **WHEN** 同一拍出现多个事件
- **THEN** 结算优先级为 人工急停 > 已验证碰撞 > 基础设施失败 > 成功窗口完成 > 任务超时，且其余事件仍完整落盘

#### Scenario: 复位失败不计任务失败
- **WHEN** 场景复位失败
- **THEN** 只记 `INFRA_FAILURE`，不进入 `MISSION_FAILED` 分子

### Requirement: 抵近跟踪成功判据
进入 `GROUND_APPROACH` 后，以下条件 SHALL 在同一连续 **5.0 s 仿真时间**窗口内同时成立方判 `TRACKING_SUCCESS`；任一条件中断即窗口清零：

1. 车中心到**目标地面投影**平面距离 ≤3.0 m；
2. 目标真实观测年龄 ≤0.5 s、质量达该来源已标定门槛；车 odom 年龄 ≤0.5 s；
3. 对**非目标障碍物**几何净空不低于场景预设保护距离，且无非目标碰撞体接触；
4. 逐帧记录车与目标速度、相对速度与距离（阶段 A 不设相对速度上限，不得称“速度匹配”或“捕获”）。

#### Scenario: 窗口连续
- **WHEN** 5.0 s 仿真时间内条件始终成立
- **THEN** 记 `TRACKING_SUCCESS` 并置 `tracking_milestone_met=true`

#### Scenario: 中断清零
- **WHEN** 窗口内出现条件中断
- **THEN** 计时清零并从恢复时刻重新累计

### Requirement: 规划与过期停车
`ground_planner` 输入 SHALL 包含目标地面接近点、有效地图、车当前姿态、车体膨胀半径及时间戳；目标位移 >1 m、地图版本变化、路径不可行或距上次规划 ≥2 s 时 SHALL 重规划，路径至少 1 Hz 重发；每次 SHALL 保存触发原因、输入与地图版本、规划耗时、路径与失败原因。目标/地图/路径过期时地面 SHALL 先停，`ground_commander` 对过期、终态与异常立即发零速。

#### Scenario: 因果闭环
- **WHEN** 固定起点与地图，只改变空中目标/障碍输入
- **THEN** `goal`、`path`、`cmd_vel` 与实际轨迹出现可解释变化；无信息时安全停车

#### Scenario: 规划不可达有界重试
- **WHEN** 输入有效但规划按场景预设次数重试仍不可达
- **THEN** 结算 `MISSION_FAILED` / `PLAN_UNREACHABLE`，不无限等待

### Requirement: 空中控制权交接与安全 hold
空中 setpoint 所有权 SHALL 按阶段分配：起飞时 `drone_launch_controller`，确认起飞并调用 `release_control` 后由 `lqr_tracker_node` 接管，任务期仅 LQR 实际发布；SHALL 核查**实际消息时间序列**（双方最后/首次发布时间、最大间隔、MAVROS 模式）而非仅看 advertiser 列表；交接期 SHALL 保持 OFFBOARD 连续 setpoint，不留 0.2 s 空窗。终止、目标失联或异常时由指定空中控制者持续发布经验证的安全 hold，不得将偏移归零后沿旧参考继续飞行。

#### Scenario: 交接无空窗
- **WHEN** 完成 release_control 交接
- **THEN** `/mavros/setpoint_position/local` 的实际发布间隔最大值小于 0.2 s，且发布者从起飞控制器切换为 LQR

### Requirement: run 日志与统计口径
每次 run SHALL 保存：唯一 `run_id`、契约/代码/SDF/场景哈希、目标源与驱动后端、参数与种子（无则写 `unseeded`）、仿真与单调墙钟时间、TF 校准、空地 odom、目标、地图、goal、path、cmd、实际轨迹、状态事件、控制交接、启动/复位/清理结果。统计 SHALL 分列 `attempted_runs`、`infra_failure_runs`、`valid_task_runs`、`tracking_success_runs`、`mission_failure_runs`、`aborted_runs`，并满足：

- `valid_task_runs = tracking_success_runs + mission_failure_runs`
- `attempted_runs = valid_task_runs + infra_failure_runs + aborted_runs`
- 任务成功率 = `tracking_success_runs / valid_task_runs`，同时报告基础设施失败/全部尝试与人工终止/全部尝试
- 真实接触、几何相交、保护距离违例、过滤器触发分别报（保护距离违例记 `CLEARANCE_VIOLATION`，不叫碰撞）

#### Scenario: 分母自洽
- **WHEN** 汇总一批 run
- **THEN** 两个等式成立，且基础设施失败不混入任务失败分子

### Requirement: 冲突修复与非回归
系统 SHALL 消除已知硬冲突，且不改变既有无人机单体实验链路行为。

#### Scenario: 时钟冲突解除
- **WHEN** 协同 run 与地面单体 run 分别启动
- **THEN** 协同 run 全局 `use_sim_time=true` 且不含 `ground_sim.launch`；地面单体 run 保持既有墙钟行为

#### Scenario: 旧执行口停用
- **WHEN** 检查 `car_commander`
- **THEN** 它不再作为地面执行口被任何协同 launch 启动，其源码/构建保留但标注废弃

## MODIFIED Requirements

### Requirement: 地面单体仿真入口
`ground_sim.launch` 的 `/use_sim_time` SHALL 由 launch 参数控制（默认保持地面单体实验所需的墙钟行为），不再无条件强制 `false`；该 launch SHALL 仅用于地面单体实验，不参与协同 run。

## REMOVED Requirements

### Requirement: `car_commander` 地面执行口
**Reason**：契约规定地面唯一执行口为 `/agc/ground/cmd_vel`，`car_commander` 持续发布固定画圆速度，与唯一控制权冲突。
**Migration**：协同 run 改由 `ground_commander` + `ground_drive_adapter` 驱动；`car_commander` 可执行目标从构建中移除（源码保留并标注废弃），任何 launch 不再启动它。

### Requirement: `INTERCEPT_CONFIRMED` 与阶段 B 判据
**Reason**：阶段 A 无 2 m 高捕获装置与指定接触判据，不能由车体或目标任意接触推定拦截。
**Migration**：状态常量保留但阶段 A 禁用；拦截方案单独冻结后另起契约版本。

## 非目标（Non-goals）

- 不实现阶段 B 拦截机构、接触判定与 `INTERCEPT_CONFIRMED`。
- 不实现 `air_fusion` 真实融合（秦可豪分工），只保留 `/agc/targets` 接口与 `source` 字段。
- 不实现 `dynamic_sim` / `hardware` 后端，不做实物与动力学验证表述。
- 不做 M7 单体性能量化与 M8 硬件联调；不改动无人机单体实验与论文实验资产。