# N4 融合与控制故障边界 规格（change-id: complete-n4-fault-boundaries）

## Why

`下一阶段任务清单.md` 的 N4（P1，未开始）要求补齐两类故障边界：

1. **融合冷启动冲突**：`air_fusion` 在无历史估计时以「最高置信通道」作空间一致性参考（[air_fusion.cpp](file:///home/ubuntu/anti_drone_agc_workspace/src/air_ground_core/src/air_fusion.cpp#L178-L198)）。若恰好是最高置信通道被注入离群（`air_eo_front` conf≈0.956 > `air_ir_top` conf≈0.542），该错误观测会被当作参考，反而把正常通道判为离群并输出错误目标。M4 §八.3 已记录此风险，用低置信通道注入规避。
2. **控制侧故障边界**：M5 §八.1 记录 `drone_launch_controller` 释放控制权后仍保留 `/mavros/setpoint_position/local` advertiser，无法仅凭发布者列表区分「advertiser」与「实际发布者」；M5 §八.4 记录 `*_TIMEOUT` 安全路径仅由代码核查、未构造超时场景。

目标：建立**冷启动冲突规则**（不把无法判定的冲突当作高置信输出），收紧 `release_control` 后的 advertiser 生命周期，并注入**高置信离群、单通道失效、目标超时、交接异常**四类故障，逐组留「观测门控 / 实际控制源 / 停车时延 / 结局」证据，正常组无新增误报。

## What Changes

- **冷启动冲突规则（`air_fusion`）**：无历史参考时，不再直接取最高置信通道，改为按 `gate_m` 对候选做单链聚类、取最大簇作参考；当候选 ≥2 且最大簇成员数 < `fusion/cold_start_min_agree`（默认 2）时判**冷启动歧义**，发布空 `/agc/targets`（`result=cold_start_ambiguous`），不输出无法判定的冲突。有历史（`ref_hold_s` 内）时行为不变。
- **advertiser 生命周期收紧（`drone_launch_controller`）**：`release_control` 后 `shutdown()` 该 setpoint advertiser，使发布者列表与实际发布者一致；`takeoff`/`arm` 入口重新 advertise，保证 `--repeat/--cycles` 批跑后续 attempt 仍能起飞。
- **故障注入能力**：编排脚本新增 `--timeout-inject <sec>`（经 launch 覆盖 `limits/mission_timeout_s`，run 前冻结）与 `--handover-inject <sec>`（交接后按秒 `rosnode kill` LQR，制造控制空窗）。融合侧沿用既有 `m4_inject.py`（`conflict`/`loss`）。
- **离线核验扩展**：`verify_fusion.py` 增 `outlier`（冷启动高置信离群）与 `single_channel` 模式；`verify_control_authority.py` 增 `timeout` 与 `handover` 模式。
- **不改变**既有 8 个协同话题语义、状态机判据、坐标/时间基准与终态分类；`contract_version` 保持 `agc-m0-v1.2`。**BREAKING**：无。

## Impact

- Affected specs：M0 接口契约 v1.2（第八节融合门控、第四节控制权）；N3 目标格可行化（沿用）。
- Affected code：
  - `src/air_ground_core/src/air_fusion.cpp`、`src/air_ground_core/config/agc_params.yaml`
  - `src/drone_launch_controller/src/drone_launch_controller_node.cpp`（+ 对应头文件如需）
  - `src/air_ground_core/launch/agc_collab.launch`、`scripts/start_agc_collab.sh`
  - `src/air_ground_core/scripts/verify_fusion.py`、`verify_control_authority.py`
  - 新增验收文档 `docs/项目任务日志/阶段2/N4.md`

## ADDED Requirements

### Requirement: 冷启动冲突规则

`air_fusion` SHALL 在无有效历史参考时，用鲁棒关联（按 `gate_m` 单链聚类取最大簇）确定空间一致性参考；当存在 ≥2 个候选且最大簇成员数小于 `fusion/cold_start_min_agree` 时，SHALL **不发布被判定为高置信的目标**（发布空数组并记录 `result=cold_start_ambiguous`）。

#### Scenario: 冷启动高置信通道离群
- **WHEN** 目标首次被观测（冷启动、无历史）时，最高置信通道被注入离群偏置（偏置 > `gate_m`），另一通道正常
- **THEN** 两候选互不落入同一簇、最大簇成员数 1 < 2，判定冷启动歧义，`/agc/targets` 持续为空
- **AND** 地面不进入向偏置点的抵近（无 `tracking_frames` / 无跟随），迟滞终止为超时类结局
- **AND** `fusion_log.jsonl` 记录 `cold_start=true`、`ref_source="ambiguous"`、`cluster_sizes`

#### Scenario: 冷启动正常（无注入）
- **WHEN** 冷启动时两通道观测一致（簇内距离 ≤ `gate_m`）
- **THEN** 最大簇成员数 ≥ 2，正常输出融合目标，行为与既有 M4 nominal 一致（无新增误报）

### Requirement: 单通道失效降级

`air_fusion` SHALL 在单通道失效（该通道 drop/无观测）时，用剩余通道继续输出目标，不因缺少冗余而误判冲突或输出空。

#### Scenario: 最高置信通道全部丢失
- **WHEN** 注入 `loss@air_eo_front drop_prob=1.0`（另一通道正常）
- **THEN** 融合仅由 `air_ir_top` 支撑，`/agc/targets` 仍非空、任务可继续；`fusion_log` 记录该通道 dropped/无候选
- **AND** 正常通道不被误拒（无 `gate_rejected_all` 误报）

### Requirement: 超时安全路径场景

系统 SHALL 通过 run 前冻结的超时参数构造真实 `*_TIMEOUT` 场景，验证超时终态下的安全状态，而非仅代码核查。

#### Scenario: 超时触发
- **WHEN** 以 `--timeout-inject` 覆盖 `limits/mission_timeout_s` 运行固定场景
- **THEN** `summary.json` 结算 `MISSION_FAILED/MISSION_TIMEOUT`；终态后地面零速停稳、空中 hold 通过（`hold_triggered && hold_ok`）；控制权仍唯一（地面 `ground_commander`、空中 `lqr_tracker_node`）

### Requirement: 交接异常与 advertiser / 发布者可区分

系统 SHALL 在 `release_control` 后关闭起飞控制器的 setpoint advertiser，使 `air_control_authority.jsonl` 的 `setpoint_publishers` 与实际发布者一致；并 SHALL 能检出交接后的控制空窗。

#### Scenario: 交接后 advertiser 不残留
- **WHEN** 正常交接完成
- **THEN** 交接后发布者列表仅含 `lqr_tracker_node`（不再含 `drone_launch_controller`），`active_controller=lqr_tracker_node`

#### Scenario: 交接后控制空窗注入
- **WHEN** 交接完成后按 `--handover-inject` 终止 `lqr_tracker_node`（setpoint 空窗 > `setpoint_gap_max_s`）
- **THEN** `air_control_authority.jsonl` 记录 `control_conflict=true` 事件，`/agc/air/control_conflict=true`；台账记录当时实际控制源；地面侧结局与停车时延照常记录

## MODIFIED Requirements

### Requirement: 融合空间一致性门控（M4 §8.2）

参考点选择扩展为：有历史 → 最近融合估计（`ref_hold_s` 内，保持不变）；无历史 → 最大簇（鲁棒）参考；最大簇不足以判定 → 不输出。其余（新鲜度、置信度、`gate_m` 一致性、逆方差加权、相关噪声协方差下界）不变。

## REMOVED Requirements

无。