# M2 共同仿真世界与时空基准验收 Spec

> 依据 [下一阶段任务清单.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E6%8C%87%E5%AF%BC%E4%B9%A6/%E4%B8%8B%E4%B8%80%E9%98%B6%E6%AE%B5%E4%BB%BB%E5%8A%A1%E6%B8%85%E5%8D%95.md) 的 M2（P0）与 [M0_接口契约_v1.1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E4%BB%BB%E5%8A%A1%E6%97%A5%E5%BF%97/M0/M0_%E6%8E%A5%E5%8F%A3%E5%A5%91%E7%BA%A6_v1.1.md) 第二、五节。
> 本 Spec 只覆盖 M2：**共同仿真世界与时空基准的专用验收证据** + **契约 §5.2 指派给 M2 的地面车保护距离标定**。不推进 M3–M9，不做阶段 B 拦截、真实融合与实物。

## Why

M2 的实现（单一 `agc_world`、全局 `/use_sim_time=true`、`world`/`air_local`/`drone/base_link`/`car/base_link` TF、模型命名、适配器日志）已在 M0 改造中落地并被 M0/M1 验收间接覆盖，但**尚无 M2 专用验收证据**：现有 `audit_interfaces.py` 只核对了车 odom 与 Gazebo 一致性，**未**核对空中 odom 与 Gazebo 真值一致性、同拍空地/目标位姿新鲜度、消息时间戳是否取自 `/clock`、地图内容是否与场景障碍一致。

同时契约 §5.2 明确写着「地面车保护距离**待 M2 按车体几何与传感器标定**」，当前 `clearance` 阈值仍是 M0 的工程初值：`collision_m=0.50` 被直接当作 `GROUND_COLLISION` 触发，会把「接近」误记成「接触/几何相交」，与契约 §5.2 的碰撞口径不一致；且 `scenario_flat.yaml` 的 `thresholds.clearance_*` 与 `agc_params.yaml` 的 `clearance` 重复但**从不被消费**，存在分歧风险。

## What Changes

- 扩展 `audit_interfaces.py`，新增 M2 专用核验项：空中 odom 与 Gazebo `iris` 真值一致性、同拍模型齐全（`iris`/`agc_car`/`intruder_uav`/`pole_1`/`pole_2`）、同拍空地 odom 与目标观测的新鲜度、消息时间戳取自 `/clock`、`/agc/map` 原点/分辨率/尺寸与占用格与场景障碍一致。
- 地面车保护距离标定：定义净空度量、由车体几何与感知基础推导 `car_radius_m`/`emergency_m`/`protection_m`，写入推导依据与配置哈希。
- **BREAKING**（协同层阈值语义）：`GROUND_COLLISION` 改为**仅由接触/几何相交**触发（`clearance ≤ car_radius_m`）；`CLEARANCE_VIOLATION`/`EMERGENCY_TRIGGER` 保持为不改结局的保护带事件。
- **BREAKING**（配置）：移除 `clearance.collision_m`（契约无此概念）；`clearance` 阈值以 `agc_params.yaml` 为唯一来源，删除 `scenario_flat.yaml` 中重复且从未生效的 `thresholds.clearance_*` 死配置。
- 产出 M2 无头批跑证据与 `docs/项目任务日志/M2/M2_共同世界验收_v1.md`。

## Impact

- Affected specs：共同世界/时空基准（M2 验收）；地面净空阈值语义与碰撞口径（修改）。
- Affected code：
  - `src/air_ground_core/src/task_manager.cpp`（碰撞判据与阈值读取）
  - `src/air_ground_core/src/agc_common.cpp`（移除未使用的场景净空字段解析）
  - `src/air_ground_core/config/agc_params.yaml`、`src/air_ground_core/config/scenario_flat.yaml`
  - `src/air_ground_core/scripts/audit_interfaces.py`（M2 核验扩展）
  - `docs/项目任务日志/M2/`（新增验收文档）
- 不改动：无人机单体实验链路、LQR、感知/DQN、`start_sim.sh`、既有论文实验资产；M0 已冻结的 `/agc/` 话题与消息定义。

## ADDED Requirements

### Requirement: M2 专用共同世界与时空基准证据
系统 SHALL 提供一次无头批跑证据，证明同一 run 内：全系统只有一个 `gzserver`/一个世界 `agc_world`、只有一个 `/clock` 发布者且 `/use_sim_time=true`；世界内同拍存在本方无人机、地面车、目标与障碍模型；空地与地面 odom、目标观测可在同一 run 内同时读取且各自年龄不超过契约上限；TF `world`/`air_local`/`drone/base_link`/`car/base_link` 闭合且无裸 `base_link`。

#### Scenario: 同拍位姿可读
- **WHEN** 在一次协同 run 的采样窗口内同时订阅 `/agc/air/odom`、`/agc/ground/odom`、`/agc/targets` 与 `/gazebo/model_states`
- **THEN** 三路位姿/观测均收到有效样本，且空中与地面 odom 的 `header.stamp` 年龄、目标观测年龄不超过各自阈值
- **AND** 同一条 `/gazebo/model_states` 内 `name` 同时含 `iris`、`agc_car`、`intruder_uav`、`pole_1`、`pole_2`

#### Scenario: 单 world 与单时钟
- **WHEN** 协同 run 启动完成
- **THEN** gazebo 相关节点数为 1、不存在 `ground_gazebo`、`/use_sim_time=true`，不启动独立 `ground_gazebo`

### Requirement: 坐标变换核验
系统 SHALL 核验 `/agc/air/odom`（world 系）与 Gazebo `iris` 真值姿态一致，且核验值由 `air_odom_adapter` 的标定残差 `residual_m` 支撑；SHALL 核验 `/agc/ground/odom` 与 Gazebo `agc_car` 真值一致（M0 已实现，M2 沿用）。

#### Scenario: 空中 odom 与真值一致
- **WHEN** 在一次 run 的稳定采样点比较 `/agc/air/odom` 位置与 `/gazebo/model_states` 中 `iris` 位置
- **THEN** 两位置差不超过 0.20 m，且同 run `air_calibration.json` 的 `residual_m` 非空且 ≤ 0.05 m

#### Scenario: 地面 odom 与真值一致
- **WHEN** 比较 `/agc/ground/odom` 位置与 `/gazebo/model_states` 中 `agc_car` 位置
- **THEN** 两位置差不超过 0.05 m

### Requirement: 时间戳取自仿真时钟
所有含 Header 的协同消息 SHALL 使用仿真时钟；M2 SHALL 提供核验证明消息时间戳取自 `/clock` 而非墙钟。

#### Scenario: 时间戳非墙钟
- **WHEN** 对 `/agc/air/odom`、`/agc/ground/odom`、`/agc/targets`、`/agc/map` 的采样样本比较 `header.stamp` 与 `/clock`
- **THEN** 每个样本的 `header.stamp` 与 `/clock` 当前值的差不超过 1.0 s（排除以墙钟 epoch 秒计的取值）

### Requirement: 地图接口与场景一致
`/agc/map` SHALL 与 `config/scenario_flat.yaml` 的 `map` 段和 `obstacles` 一致：原点、分辨率、尺寸与场景一致；场景障碍所在世界坐标的栅格被占用（100）；占用格数与障碍几何相符；地图版本哈希非空且写入 run 日志。

#### Scenario: 地图内容正确
- **WHEN** 解析 `/agc/map` 与场景文件
- **THEN** `info.resolution`/`info.width`/`info.height`/`info.origin` 与场景匹配，`pole_1`/`pole_2` 中心所在栅格为占用，占用格数 > 0
- **AND** `/agc/map/version_hash` 非空

### Requirement: 单一启动/退出入口
系统 SHALL 提供唯一协同启动入口与干净退出入口，批跑结束后无本次遗留进程。

#### Scenario: 干净退出
- **WHEN** 批跑完成后核查进程
- **THEN** `gzserver`/`gzclient`/`rosmaster`/`mavros_node`/协同节点均无残留

### Requirement: 地面车保护距离标定
系统 SHALL 按车体几何与感知基础标定并锁定地面净空阈值，给出推导依据、配置哈希与场景锁定口径。净空度量 SHALL 定义为**车中心到障碍 OBB 表面的有符号距离**（`pointObbClearance`）。

- `car_radius_m`：由车体 OBB `0.44 × 0.34 × 0.16 m`（`models/agc_car/agc_car.sdf`）得二维外接圆半径 `sqrt(0.22²+0.17²) = 0.2784 m`，取 `0.30 m`（含 0.02 m 裕度）。**接触/几何相交判据**：`clearance ≤ car_radius_m`。
- `emergency_m = car_radius_m + 单控制步位移`，其中单控制步位移 `= max_linear_speed ÷ drive_hz = 1.0 ÷ 20 = 0.05 m`，故 `0.35 m`。触发 `EMERGENCY_TRIGGER`（事件，不改结局）。
- `protection_m = car_radius_m + 地图量化误差(resolution/√2 = 0.5/√2 = 0.354 m) + 单控制步位移(0.05 m) + 裕度(0.10 m) ≈ 0.80 m`。触发 `CLEARANCE_VIOLATION`（事件，不改结局），同时是抵近跟踪成功条件 3 与规划膨胀下界。
- 阈值顺序：`接触(car_radius 0.30) ≤ emergency(0.35) < protection(0.80)`。

#### Scenario: 阈值来源可核查
- **WHEN** 读取 run `meta.json`
- **THEN** `params.clearance` 含 `car_radius_m`/`emergency_m`/`protection_m` 且与推导一致，不含 `collision_m`
- **AND** 该 run 记录的 `agc_params_hash` 与 `scenario_hash` 可复算

#### Scenario: 标定不影响安全判据结果
- **WHEN** 在固定简单场景运行
- **THEN** 执行轨迹的最小净空不低于 `protection_m`（规划膨胀保证），且不出现 `GROUND_COLLISION`

## MODIFIED Requirements

### Requirement: 抵近跟踪成功条件 3
进入 `GROUND_APPROACH` 后，成功窗口条件 3 SHALL 为：对非目标障碍物的几何净空不低于**标定后的** `protection_m`，且无 `GROUND_COLLISION`。保护距离的来源由 M0 的工程初值改为本 Spec 的标定值。

#### Scenario: 保护带事件不判失败
- **WHEN** 净空落入 `[car_radius_m, protection_m)` 区间
- **THEN** 只记 `CLEARANCE_VIOLATION`（或 `EMERGENCY_TRIGGER`）事件，不结算 `MISSION_FAILED`；仅当 `clearance ≤ car_radius_m` 才结算 `GROUND_COLLISION`

## REMOVED Requirements

### Requirement: `clearance.collision_m`
**Reason**：契约 §5.2 只定义「碰撞/几何相交」与两类保护带事件（`CLEARANCE_VIOLATION`、`EMERGENCY_TRIGGER`），无 `collision_m` 概念；把 `collision_m=0.50` 当作 `GROUND_COLLISION` 触发会提前 0.20 m 把「接近」记成「碰撞」。
**Migration**：接触判据改用 `car_radius_m`；从 `agc_params.yaml`、`scenario_flat.yaml`、`agc_common.cpp`（`Scenario` 净空字段解析）与 `task_manager.cpp` 移除 `collision_m`。

### Requirement: `scenario_flat.yaml` 的重复净空阈值
**Reason**：`thresholds.clearance_protection_m`/`clearance_collision_m`/`clearance_emergency_m` 被解析进 `Scenario` 却从未被任何节点消费，与 `agc_params.yaml` 的 `clearance` 重复，存在静默分歧风险。
**Migration**：删除这些死配置；净空阈值以 `agc_params.yaml` 的 `clearance` 段为唯一权威来源（该段哈希已写入 run `meta.json` 的 `agc_params_hash`）。「阈值随场景文件锁定」留待 M6 多场景时另行设计。

## 非目标（Non-goals）

- 不推进 M3–M9；不做阶段 B 拦截机构、接触判据与 `INTERCEPT_CONFIRMED`。
- 不做 M1 遗留项收口（终态→同码终态 run 边界、复位后 odom 瞬态根因）——本轮按用户决定排除。
- 不做 `dynamic_sim`/`hardware` 后端、`air_fusion` 真实融合、M7 单体性能与 M8 硬件联调。
- 不强制触发保护带事件（`CLEARANCE_VIOLATION`/`EMERGENCY_TRIGGER`）：规划膨胀使执行轨迹净空 ≥ `protection_m`，故该两类事件在本轮仍以代码路径核查为准，不构造接触场景。