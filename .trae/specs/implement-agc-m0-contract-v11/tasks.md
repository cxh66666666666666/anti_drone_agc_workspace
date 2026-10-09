# Tasks

> 依据 [M0_接口契约_v1.1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E4%BB%BB%E5%8A%A1%E6%97%A5%E5%BF%97/M0/M0_%E6%8E%A5%E5%8F%A3%E5%A5%91%E7%BA%A6_v1.1.md)。新代码一律落在 `src/air_ground_core/`（消息/适配器/规划/执行/日志/launch/worlds/models）；无人机单体实验链路只读不改；全程无头运行，进程按本会话启动差集回收。

- [x] Task 1：协同层消息与包骨架
  - [x] SubTask 1.1：新增 `msg/TargetTrack.msg`、`msg/TargetTrackArray.msg`、`msg/AgcTaskState.msg`，字段与契约完全一致（含全部状态常量与 NaN 语义注释）
  - [x] SubTask 1.2：改 `CMakeLists.txt`/`package.xml`：启用 `message_generation`/`message_runtime`、加入 `nav_msgs`/`geometry_msgs`/`std_msgs` 依赖，生成三个消息
  - [x] SubTask 1.3：从构建中移除 `car_commander` 可执行目标（源码保留并在文件头标注“废弃：地面执行口已改为 `/agc/ground/cmd_vel`”）
  - [x] SubTask 1.4：`catkin_make air_ground_core` 通过，`rosmsg show air_ground_core/TargetTrack` 等三条可查

- [x] Task 2：唯一共同世界 `agc_world` 与协同启动入口
  - [x] SubTask 2.1：新增 `worlds/agc_world.world`：由 PX4 `empty.world` 派生（保留 sun/ground_plane 与物理参数），include 地面车、目标 `intruder_uav`、场景障碍
  - [x] SubTask 2.2：新增 `models/agc_car/`（link 名 `car/base_link`）与 `models/intruder_uav/`（默认 z=2.0，非被控目标）
  - [x] SubTask 2.3：新增 `launch/agc_collab.launch`：`world:=agc_world`、`use_sim_time=true`、无头、只起一个 PX4/Gazebo + 车 + `intruder_driver` + 全部 `/agc/` 节点（**不** include `ground_sim.launch`，**不** 起 `ground_gazebo`）
  - [x] SubTask 2.4：新增 `scripts/start_agc_collab.sh`（无头一键启动 + 干净退出），并在 `scripts/` 内不改 `start_sim.sh`
  - [x] SubTask 2.5：修 `rl_ground_avoidance/launch/ground_sim.launch`：`use_sim_time` 改为 launch arg（默认保持地面单体墙钟行为）
  - [x] SubTask 2.6：核查单 world 与单 `/clock`：无 `ground_gazebo` 节点、世界内可见车/目标/障碍

- [x] Task 3：时空基准与四个适配器
  - [x] SubTask 3.1：`air_odom_adapter`：MAVROS 本地位姿 → `world`，标定并记录 `T_world_air_local`/时间/残差，twist 旋转到机体系，发布 `/agc/air/odom`（≥20 Hz）与 TF
  - [x] SubTask 3.2：`ground_odom_adapter`：读 Gazebo 实际车体位姿 → `/agc/ground/odom`（≥20 Hz），不读 `CarStatus` 位置或驱动器积分状态
  - [x] SubTask 3.3：`gt_adapter`：真值 `intruder_uav` → `/agc/targets`（10 Hz），`source=gt_adapter`、`confidence=1`、无目标发空数组、重发不刷新 stamp
  - [x] SubTask 3.4：`map_adapter`：场景静态障碍 → `/agc/map`（≥1 Hz 且变更即发），含原点/分辨率/尺寸与内容哈希版本，空图无效
  - [x] SubTask 3.5：TF 闭合核查：`world`/`air_local`/`drone/base_link`/`car/base_link`，无裸 `base_link`

- [x] Task 4：任务管理（状态机、门禁、goal）
  - [x] SubTask 4.1：`task_manager`：`/agc/task/start`（仅 IDLE 且门禁通过）、`/agc/task/abort`（任意非终态），门禁含单 world/时钟/TF/空地 odom/目标与地图源/地面后端/空中起飞交接/场景复位
  - [x] SubTask 4.2：状态机 `IDLE→AIR_SEARCH→TARGET_ACQUIRED→GROUND_APPROACH→TRACKING_SUCCESS` 与 `MISSION_FAILED`/`ABORTED`/`INFRA_FAILURE`，含路径可用性前置与有界等待
  - [x] SubTask 4.3：抵近跟踪 5.0 s 连续窗口四条件判定（3.0 m / 年龄 0.5 s / 非目标净空 / 逐帧记录）
  - [x] SubTask 4.4：发布 `/agc/ground/goal`（≥5 Hz 且变化即发，world、z=0，取目标地面投影或规划出的安全接近点）
  - [x] SubTask 4.5：原因码与同拍结算优先级（急停>碰撞>基础设施失败>成功>超时；保护距离违例记 `CLEARANCE_VIOLATION`，不叫碰撞）
  - [x] SubTask 4.6：发布 `/agc/task/state`（2 Hz 且变化即发），无目标时距离/年龄为 NaN

- [x] Task 5：规划与地面执行链
  - [x] SubTask 5.1：`ground_planner`：输入含目标地面接近点/地图/车位姿/膨胀半径/时间戳；重规划条件（位移>1 m、地图版本变化、路径不可行、≥2 s）；`/agc/ground/path` ≥1 Hz 且重规划即发，非空
  - [x] SubTask 5.2：`ground_commander`：`/agc/ground/cmd_vel` 唯一发布者，20 Hz 含零速；目标/路径过期、终态、异常立即零速
  - [x] SubTask 5.3：`ground_drive_adapter`：运动学积分 + 写本方车 `set_model_state`（`kinematic_sim` 唯一写入者），独立 0.2 s 命令 watchdog 零速，上报服务调用失败
  - [x] SubTask 5.4：`intruder_driver`：目标按场景轨迹运动（可写非被控目标位姿），记录轨迹与运动约束
  - [x] SubTask 5.5：`scenario_reset`：仅在任务非运行期复位本方车（与目标/场景），失败重试并上报 `RESET_FAILURE`
  - [x] SubTask 5.6：规划日志：每次保存触发原因、输入与地图版本、规划耗时、路径、失败原因

- [x] Task 6：空中控制权交接与安全 hold
  - [x] SubTask 6.1：起飞 → 确认起飞 → `release_control` → LQR 接管，核查实际消息时间序列（双方最后/首次发布时间、最大间隔、MAVROS 模式）
  - [x] SubTask 6.2：交接期保持连续 setpoint（无固定静默 0.2 s 空窗）
  - [x] SubTask 6.3：终止/目标失联/异常时由指定空中控制者持续发布经验证安全 hold，禁止归零偏移后沿旧参考飞行

- [x] Task 7：run 日志与统计口径
  - [x] SubTask 7.1：run 目录：唯一 `run_id`、契约/代码/SDF/场景哈希、目标源与驱动后端、参数与种子（无则 `unseeded`）、仿真与单调墙钟时间
  - [x] SubTask 7.2：落盘 TF 校准、空地 odom、目标、地图、goal、path、cmd、实际轨迹、状态事件、控制交接、启动/复位/清理结果
  - [x] SubTask 7.3：汇总脚本：分列 `attempted/infra_failure/valid_task/tracking_success/mission_failure/aborted`，校验两条分母等式并输出三类比率
  - [x] SubTask 7.4：核查脚本：逐话题实际发布者/频率/年龄、TF 闭合、车 odom 与 Gazebo 车体一致性

- [x] Task 8：无头端到端实跑与四类结局证据
  - [x] SubTask 8.1：固定简单场景无头实跑一个成功 run（到达 `TRACKING_SUCCESS`），产出可回放日志与轨迹
  - [x] SubTask 8.2：产出 `MISSION_FAILED` 样例（如移除目标 → `SEARCH_TIMEOUT` / `TARGET_LOST`）
  - [x] SubTask 8.3：产出 `INFRA_FAILURE` 样例（如时钟停滞或 TF/odom 失效注入）与 `ABORTED` 样例（调用 `/agc/task/abort`）
  - [x] SubTask 8.4：因果闭环核查：固定起点与地图、只改目标/障碍输入，goal/path/cmd/实际轨迹出现可解释变化；无信息时安全停车
  - [x] SubTask 8.5：重复性核查：同场景连续多次 run 均完整退出、每次尝试都有日志、无本次遗留进程
  - [x] SubTask 8.6：结果口径与措辞核对：`TRACKING_SUCCESS` 不等于拦截完成；`INTERCEPT_CONFIRMED` 未出现

# Task Dependencies

- Task 2 依赖 Task 1（消息与包骨架）
- Task 3 依赖 Task 2（世界与启动入口）
- Task 4 依赖 Task 3（odom/目标/地图源）
- Task 5 依赖 Task 3、Task 4（goal 与状态机输入）
- Task 6 依赖 Task 2（PX4 启动链）；可与 Task 4/5 并行开发
- Task 7 依赖 Task 3–Task 5（需要各话题数据）
- Task 8 依赖 Task 1–Task 7
- 各 Task 产物互不覆盖；`start_sim.sh`、无人机单体实验与论文实验资产只读保留

# 证据索引（本次实跑）

- 官方 run 根目录：`experiments/agc_runs/`（`_` 开头目录为调试/辅助 run，不计入分母）
  - 任务成功：`20260926T081303_cd34a5`（另有 2 次重复成功 run）
  - 任务失败：`20260926T081432_1ec90d`（`TARGET_LOST`）
  - 基础设施失败：`20260926T081548_6be25d`（`ODOM_INVALID`）
  - 人工终止：`20260926T081700_fc960d`（`OPERATOR_ABORT`）
- 汇总报表：`experiments/agc_runs/aggregate.json`（两条分母等式均 PASS；成功率 0.75 (3/4)）
- 接口现场核查：`experiments/agc_runs/audit_interfaces.json`（51 PASS / 0 FAIL）
- 单命令运行脚本：`scripts/start_agc_collab.sh`（`--auto-run success|mission_failed|infra_failure|aborted`）

