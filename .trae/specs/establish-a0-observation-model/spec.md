# 阶段3 A0：观测模型与真值集 Spec

## Why
阶段3 D（评估科学化）已验收，主线转入 A（非视觉感知与目标跟踪）。A0 是 A 的起点：当前 `air_perception` 只模拟**独立**通道噪声、固定延迟/丢包和几何门控（N1），缺少 A1 状态估计、A2 关联与 A3 敏感性扫描所必需的可控观测模型——没有可配置的相关噪声、偏置、短时/成簇丢失、遮挡、虚警与目标机动，也没有可审计的「生成/送达时刻 + 真值关联 ID + 拒收原因」真值集。

A0 只做**观测生成侧**（真值派生的多源观测仿真 + 评估日志），不做状态估计、关联或任务闭环；这些属 A1/A2/A3。

## What Changes
- `perception_sensors.yaml` 通道字段扩展：相关噪声（通道内 `corr_xy`、通道间公共模态 `corr_group`/`corr_rho`、时间相关 `noise_tau_s`）、确定性偏置 `bias_m`、成簇丢失（`drop_burst_*` 与跨通道 `drop_group`）、虚警率 `false_alarm_rate_hz`；新增顶层 `perception_model` 段承载遮挡与全局开关。
- `air_perception` 实现上述观测模型：每通道每子流**独立 RNG 命名空间**（噪声/丢包/虚警/成簇），噪声协方差写入消息 `pose.covariance`（含 x/y 非对角项以反映相关噪声）。
- 新增**遮挡门控**：在原始观测时刻对无人机→目标/障碍的视线与真值障碍 OBB 求交，被遮挡则不产生该通道观测，`gate_reason=occluded`。
- 新增**虚警生成**：按 `false_alarm_rate_hz` 生成幻影观测，仅落评估日志（`is_false_alarm=true`、`truth_id=null`）；**A0 不上线多目标 ID**。
- **真值集日志扩展**：`perception_log.jsonl` 每条记录生成时刻、送达时刻、源、真值关联 ID、拒收原因与故障参数；真值关联 ID **只写评估日志**，不进入任何协同话题/消息。
- 新增**目标机动**：`scenario` 的 `target_motion` 新增可配置机动模式（在现有 `static`/`linear` 之外），由 `intruder_driver` 消费，用于估计器在机动下的评估。
- 新增 A0 冻结脚本与核验脚本，产出 `experiments/a0/` 证据与 `docs/项目任务日志/阶段3/A/A0.md` 验收记录。
- **BREAKING**：无。沿用 `agc-m0-v1.2`；A0 改动为**配置字段级 + 评估日志级**，不新增/修改协同话题与消息语义。

## Impact
- Affected specs: 阶段3 A0（本 spec 新建）；承接 N1 几何门控、D0 `eval_protocol_v1`（不修改该冻结协议）。
- Affected code:
  - `src/air_ground_core/config/perception_sensors.yaml`（通道字段 + 顶层 `perception_model`）
  - `src/air_ground_core/config/agc_params.yaml`（`perception` 段默认项）
  - `src/air_ground_core/src/air_perception.cpp`（观测模型、遮挡、虚警、日志）
  - `src/air_ground_core/src/intruder_driver.cpp`、`src/air_ground_core/src/agc_common.cpp`（`target_motion` 机动模式解析与驱动）
  - `src/air_ground_core/scripts/d1_scenario_gen.py`（感知随机化扩展新字段；机动场景）
  - `src/air_ground_core/scripts/`（新增 `a0_freeze.py`、`verify_a0_perception.py`）
  - `experiments/a0/`（冻结哈希、重放与核验证据）
  - `docs/项目任务日志/阶段3/A/A0.md`（新增）、`docs/项目任务日志/阶段3/任务清单.md`（A0 状态更新）
- 契约影响：不新增/修改八个协同话题，**不构成 M0 语义变更**。虚警/多目标 ID 的在线接线属语义变更，**延后到 A2 契约检查点**（见下「契约检查点」）。

## 契约检查点（本 spec 的明确边界）
阶段3任务清单第二节第 6 条：多目标 ID 规则改变契约须先形成 v1.3 变更表。因此：

- A0 **不**在 `/agc/air/obs`、`/agc/targets` 上引入第二个 `target_id`，虚警与多目标只在评估日志中体现（`is_false_alarm`、`truth_id`）。
- 真值关联 ID 仅用于**离线评估**，不得出现在任何发布消息字段中；在线关联器仍只见 `target_id=intruder_uav`。
- 若后续需要把虚警/多目标观测送入在线消费，须先完成 A2 的 M0 v1.3 契约变更表、迁移/回归清单与旧消费者兼容测试，再行实现。

## ADDED Requirements

### Requirement: 可配置相关噪声、偏置与成簇丢失
`air_perception` SHALL 按通道配置生成带**相关噪声**、**确定性偏置**与**短时/成簇丢失**的真值派生目标观测，且每通道每随机源使用独立命名空间种子。

#### Scenario: 相关噪声与配置相符
- **WHEN** 通道配置了通道内相关 `corr_xy`、通道间公共模态 `corr_group`/`corr_rho` 或时间相关 `noise_tau_s`
- **THEN** 观测位置协方差（含 x/y 非对角项）与配置一致；离线统计的通道间误差相关系数、时间自相关与配置相符；不同随机源（噪声/丢包/虚警）互不影响

#### Scenario: 成簇丢失与配置相符
- **WHEN** 通道配置了 `drop_burst_*` 与 `drop_group`
- **THEN** 丢失呈成簇（突发）分布而非逐拍独立；同 `drop_group` 的通道在同一突发窗内同向丢失；实测突发长度均值、丢包率与配置相符

#### Scenario: 确定性偏置
- **WHEN** 通道配置了 `bias_m`
- **THEN** 观测误差均值等于该偏置（与随机噪声分离），并在评估日志中可区分偏置与噪声

### Requirement: 遮挡门控
`air_perception` SHALL 在**原始观测时刻**按真值障碍几何对目标/障碍观测做视线遮挡判定，被遮挡通道不产生该观测并记录拒收原因。

#### Scenario: 视线被遮挡
- **WHEN** 无人机到目标的视线与某真值障碍 OBB 相交（含配置外扩）
- **THEN** 该通道不产生目标观测，`perception_log.jsonl` 记录 `gate_reason=occluded`；遮挡解除后按门控恢复观测，且不刷新旧观测时间戳

#### Scenario: 与 N1 门控次序一致
- **WHEN** 目标同时出视场/量程且被遮挡
- **THEN** 拒收原因取值稳定、可判定（沿用 N1 原因集并新增 `occluded`），不产生观测

### Requirement: 虚警生成（评估日志级）
`air_perception` SHALL 按 `false_alarm_rate_hz` 生成幻影目标观测，并**仅在评估日志**中标注，不改变在线消息语义。

#### Scenario: 虚警率与配置相符
- **WHEN** 通道配置了 `false_alarm_rate_hz`
- **THEN** 实测虚警发生率与配置相符；虚警记录 `is_false_alarm=true`、`truth_id=null`、位置与真值不相邻

#### Scenario: 在线链路不受虚警影响
- **WHEN** 查看 `/agc/air/obs`、`/agc/targets` 的消息字段
- **THEN** 不出现真值关联 ID 或虚警专用 `target_id`；在线消费仍只见 `target_id=intruder_uav`

### Requirement: 真值集日志与真值 ID 隔离
`perception_log.jsonl` SHALL 逐样本记录观测**生成时刻**、**送达时刻**、**源（通道）**、**真值关联 ID**、**拒收原因**与故障参数；真值关联 ID MUST 只写评估日志，不泄漏给在线关联器。

#### Scenario: 日志字段完整
- **WHEN** 任一 run 结束
- **THEN** 每条样本含生成时刻、送达时刻、通道/源、真值关联 ID、`gate_reason`（含 `occluded`）、`is_false_alarm`；门控/遮挡关闭时不产生观测且原因可查

#### Scenario: 真值 ID 不泄漏
- **WHEN** 核验消息定义与在线话题记录
- **THEN** `TargetObservation`/`TargetTrack` 等消息结构与其发布内容不含真值关联 ID 字段；仅评估日志含之

### Requirement: 目标机动
`scenario` 的 `target_motion` SHALL 支持在 `static`/`linear` 之外的可配置机动模式，由 `intruder_driver` 按仿真时钟确定性驱动。

#### Scenario: 机动轨迹确定性
- **WHEN** 同一场景/种子重复运行
- **THEN** `target_trajectory.jsonl` 逐条一致；机动模式的位置/速度连续、速度不超过场景约束（仍低于车上限）

### Requirement: A0 冻结、重放可复现与核验
系统 SHALL 提供 A0 参数/种子冻结记录与核验脚本，证明重放可复现且各项观测统计与配置相符。

#### Scenario: 参数/种子冻结
- **WHEN** 运行冻结脚本
- **THEN** 输出 `perception_sensors.yaml`、`agc_params.yaml`（perception 段）、场景/种子 manifest 与核验脚本的哈希（含 `combined_hash`）；改动被冻结对象后复核 FAIL

#### Scenario: 重放可复现
- **WHEN** 以相同种子与配置重复运行
- **THEN** `perception_log.jsonl` 的观测序列（时刻、通道、序号、值、原因）逐条一致，差异仅为墙钟字段

#### Scenario: 核验通过
- **WHEN** 运行 `verify_a0_perception.py`
- **THEN** FOV/量程门控一致性、延迟分布、漏检率/虚警率、相关性与配置相符、`truth_id` 不泄漏、重放一致性逐项给出 PASS/FAIL

## MODIFIED Requirements

### Requirement: 几何门控（N1）
保留 N1 的视场/量程门控语义与拒收原因集（`no_drone_pose`/`out_of_range_min`/`out_of_range_max`/`behind`/`out_of_fov`/`ok`/`no_target`），在其基础上**新增遮挡原因** `occluded`，作为门控关闭的一种。门控判定仍严格按**原始观测时刻**的无人机位姿执行，消息时间戳语义不变。

## REMOVED Requirements
无。