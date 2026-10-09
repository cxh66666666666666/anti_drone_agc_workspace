# M4 目标识别与多源融合（感知→融合→规划）验收 Spec

> 依据 [下一阶段任务清单.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E6%8C%87%E5%AF%BC%E4%B9%A6/%E4%B8%8B%E4%B8%80%E9%98%B6%E6%AE%B5%E4%BB%BB%E5%8A%A1%E6%B8%85%E5%8D%95.md) 的 M4（P1）与升级后的 [M0_接口契约_v1.2.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E4%BB%BB%E5%8A%A1%E6%97%A5%E5%BF%97/M0/M0_%E6%8E%A5%E5%8F%A3%E5%A5%91%E7%BA%A6_v1.2.md)。
> 只覆盖 M4：**多源空中观测→融合输出，被 M3 规划链路消费；延迟/丢失/冲突注入时行为可解释**。不推进 M5 控制权、M6 多场景、M8 实物。
> **诚实口径**：本阶段感知是「由 Gazebo 真值派生的感知仿真」，不是真实视觉/雷达识别，不得宣称完成真实目标探测。

## Why

M0 契约 v1.1 冻结了 `/agc/targets` 单源 `gt_adapter`，链路 `gt_adapter → task_manager → ground_planner → ground_commander → ground_drive_adapter` 已在 M3 验收因果闭环。但全仓**没有感知前端、没有多源融合、没有障碍观测消息**：目标源直接取 Gazebo 真值，障碍避障直接取场景静态配置。M4 需补齐"目标识别与多源融合"，且必须**不破坏** M1–M3 既有证据。

## What Changes

- 升 M0 契约为 v1.2，登记感知/融合话题与消息、`air_fusion` 目标源、`/agc/obstacles/source`、confidence 标定定义、故障语义。
- 新增实时链路 C++ 节点 `air_perception`（真值派生多通道观测，含噪声/延迟/丢包/离群，在线 rosparam 可调）与 `air_fusion`（新鲜度/置信度/空间一致性门控 + 保守融合）。
- 新增 6 个消息：`TargetObservation`、`ObstacleObservation`、`SensorObservation`、`AirObservationArray`、`ObstacleTrack`、`ObstacleTrackArray`；新增话题 `/agc/air/obs`、`/agc/obstacles`。
- `/agc/targets` 发布者随 `/agc/target/source` 在 run 前固定（`gt_adapter` | `air_fusion`）。
- 障碍融合接入 `map_adapter`/`ground_planner`（source=air_fusion 时走融合障碍）；`task_manager` 安全判据坚持真值。source=scenario 时路径与现状构造性等价。
- 新增离线工具 `scripts/m4_inject.py`（按时序注入）与 `scripts/verify_fusion.py`（核验），扩展 `audit_interfaces.py`。
- 产出证据批次与验收文档。

## Impact

- Affected specs：M4 感知与多源融合；M0 契约 v1.1 → v1.2（仅新增，不改动 v1.1 已冻结的 8 个话题语义）。
- Affected code（新增）：`msg/` 6 个消息、`src/air_perception.cpp`、`src/air_fusion.cpp`、`scripts/m4_inject.py`、`scripts/verify_fusion.py`、`config` 感知/融合参数。
- Affected code（修改）：`CMakeLists.txt`、`launch/agc_collab.launch`、`scripts/start_agc_collab.sh`、`src/gt_adapter.cpp`（源门控）、`src/map_adapter.cpp`、`src/ground_planner.cpp`、`src/agc_run_logger.cpp`、`scripts/audit_interfaces.py`。
- 只读保留：`task_manager.cpp` 安全判据、`ground_commander.cpp`、`ground_drive_adapter.cpp`、`intruder_driver.cpp`、`scenario_flat.yaml`、`scenario_linear_target.yaml`、M1–M3 验收文档与既有证据目录。

### 实现语言理由（Python 例外登记）
`m4_inject.py`（按时间线调 rosparam）与 `verify_fusion.py`（离线日志后处理）不订阅/发布实时控制话题、不参与控制时序，属工程约定允许的"离线工具"；实时链路节点一律 C++。

## ADDED Requirements

### Requirement: 空中观测接口（目标与障碍）
系统 SHALL 提供 `/agc/air/obs`（`AirObservationArray`，唯一发布者 `air_perception`，≥10 Hz，age ≤0.3 s），其中每个 `SensorObservation` 含 `sensor_id`，`TargetObservation`/`ObstacleObservation` 均含 `header`（观测时刻，`frame_id=world`）、`confidence`、`obs_seq`。

#### Scenario: 观测消息字段齐全
- **WHEN** 读取融合 run 的 `/agc/air/obs` 采样
- **THEN** 每条目标/障碍观测含非空 `sensor_id`、`header.frame_id=="world"`、`header.stamp>0`、`0≤confidence≤1`、单调 `obs_seq`

### Requirement: 多源融合输出
系统 SHALL 由 `air_fusion` 融合多通道观测：新鲜度门控（age ≤ `/agc/fusion/target_max_age_s`，默认 0.35 s）、置信度门控（分源阈值）、空间一致性门控（与当前估计距离超 `gate_m` 判离群）；输出 `/agc/targets`（`source=air_fusion`，`header.stamp=最新真实观测时刻`，无有效观测时为空数组）与 `/agc/obstacles`（按 `obstacle_id` 关联 inlier，并集+保守外扩）。

#### Scenario: 融合结果可被 M3 消费
- **WHEN** 以 `--target-source air_fusion` 跑 nominal 场景
- **THEN** `/agc/targets` 发布者为 `air_fusion` 且 `source=="air_fusion"`；`/agc/ground/goal` 跟随融合目标；任务状态正常结算；`/agc/obstacles` 非空且被规划消费

#### Scenario: 不刷新旧观测时间
- **WHEN** 某拍无新有效观测
- **THEN** 输出为空数组，融合 track 的 `header.stamp` 不被刷新为发布时刻

### Requirement: 超时与失联处理
系统 SHALL 在观测过期/丢失时输出空数组而非复制旧姿态；目标丢失超过契约容忍窗时由 `task_manager` 结算 `TARGET_LOST` 并先停车。

#### Scenario: 全通道丢失
- **WHEN** 注入全部传感器 `drop_prob=1.0`
- **THEN** ≤ `target_lost_tolerance_s` 内 `/agc/targets` 转空，车 `cmd_vel` 归零，终态 `MISSION_FAILED/TARGET_LOST`

### Requirement: 注入延迟/丢失/冲突的可解释行为
系统 SHALL 支持在线 rosparam 注入 `/agc/perception/fault/<sensor>/{enabled,drop_prob,latency_s,outlier_bias_m}`，并在融合日志中给出可解释记录。

#### Scenario: 延迟
- **WHEN** 注入某通道 `latency_s` 使观测年龄超过融合门限
- **THEN** 该通道观测被判旧（`stale`），不计入融合；若其余通道有效则输出由其余 inlier 支撑，输出 `header.stamp` 不早于最近有效观测

#### Scenario: 冲突
- **WHEN** 注入某通道 `outlier_bias_m` 使其与其余通道显著不一致
- **THEN** 该通道观测被空间门控判 `outlier` 并记录原因；融合结果跟随 inlier；**不做跨源取最大**

### Requirement: confidence 标定
系统 SHALL 定义 `confidence` 为「位置误差 ≤ `confidence_tolerance_m` 的经验概率」，提供离线经验覆盖核验。

#### Scenario: 经验覆盖与声明一致
- **WHEN** 用 run 内观测与真值计算经验覆盖 P(|err|≤tol)
- **THEN** 经验覆盖与通道声明 confidence 的偏差在预设容差内

## MODIFIED Requirements

### Requirement: 契约版本
M0 契约 SHALL 升为 `agc-m0-v1.2`：新增上述话题/消息与 source 语义；`agc_params.yaml.contract_version` 与 `agc_run_logger` 的 `CONTRACT_DOC` 同步。旧 run 的 `contract_doc` 哈希保持指向其当时版本，不改写历史。

### Requirement: 障碍源选择与规划接入
系统 SHALL 由 `/agc/obstacles/source` ∈ {`scenario`,`air_fusion`} 在 run 前固定障碍源；`air_fusion` 时 `map_adapter`/`ground_planner` 使用融合障碍；`task_manager` 净空/碰撞判据 SHALL 始终使用真值场景几何。

#### Scenario: scenario 等价
- **WHEN** `--obstacle-source scenario`（默认）
- **THEN** `map_adapter`/`ground_planner`/`task_manager` 行为与 M1–M3 一致，`/agc/map` 内容哈希与 M2 记录一致
