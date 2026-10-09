# M3 空地最小闭环（空中信息→地面规划→执行）验收 Spec

> 依据 [下一阶段任务清单.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E6%8C%87%E5%AF%BC%E4%B9%A6/%E4%B8%8B%E4%B8%80%E9%98%B6%E6%AE%B5%E4%BB%BB%E5%8A%A1%E6%B8%85%E5%8D%95.md) 的 M3（P1）与 [M0_接口契约_v1.1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E4%BB%BB%E5%8A%A1%E6%97%A5%E5%BF%97/M0/M0_%E6%8E%A5%E5%8F%A3%E5%A5%91%E7%BA%A6_v1.1.md) 第三、六、七节。
> 本 Spec 只覆盖 M3：**用可控仿真真值适配器验证「空中信息确实改变地面规划与动作」的因果闭环**，并补齐 M3 专用证据。不推进 M4 真实感知融合、M5 控制权、M6 多场景、M8 实物。

## Why

M0 改造已落地完整链路 `gt_adapter → /agc/targets → task_manager(/agc/ground/goal) → ground_planner(/agc/ground/path) → ground_commander(/agc/ground/cmd_vel) → ground_drive_adapter(写车体位姿)`，并在**静态目标**下确实驱动车辆运动（例如 `20260927T051413_ac0c19` 车位移约 11.5 m）。

但 M3 所要求的**因果闭环**——「改变空中观测/目标后，地面规划路径随之更新并驱动车运动」——**从未被真正触发或验证**：全仓 157 个 `planner_log.jsonl` 中 `trigger="displacement"` 的出现次数为 **0**，说明现有全部 run 的目标输入都是静止的，`ground_planner` 的「目标位移 >1 m 即重规划」这条随空中输入变化的路径从未被激活。同时缺少 M3 专用的可核查证据与验收文档。本轮即为补齐这一验证。

## What Changes

- 新增受控对比场景 `src/air_ground_core/config/scenario_linear_target.yaml`：与 `scenario_flat.yaml`（对照组）**除 `target_motion` 外逐字段一致**（`car_start`/`target_start`/`obstacles`/`map`/`thresholds` 完全相同），仅把目标运动改为 `mode: linear`、新增可达的目标速度，用于「只改变空中目标输入」的处理组。
- 新增离线核验工具 `src/air_ground_core/scripts/verify_causal_closure.py`（**离线工具，非实时链路，故用 Python**，理由见下），读取 run 日志与两个场景文件，对因果链与「无信息安全停车」逐项输出 PASS/FAIL 与 JSON。
- 产出 M3 无头批跑证据：静态基线 run（对照组）、目标运动 run（处理组）、无信息安全停车 run。
- 产出验收文档 `docs/项目任务日志/M3/M3_空地最小闭环验收_v1.md`。
- **不改动实时链路节点**：M0 已实现的链路已满足 M3；本轮只做受控验证与证据。不修改 `scenario_flat.yaml`（保留为对照组）、M0 已冻结的 `/agc/` 话题与消息、launch 与 `start_agc_collab.sh`。

## Impact

- Affected specs：空地最小闭环因果验证（M3 验收）；使用 `intruder_driver` 的 `linear` 运动分支（首次集成验证）。
- Affected code（仅新增，不改实时节点）：
  - `src/air_ground_core/config/scenario_linear_target.yaml`（新增场景）
  - `src/air_ground_core/scripts/verify_causal_closure.py`（新增离线核验）
  - `docs/项目任务日志/M3/M3_空地最小闭环验收_v1.md`（新增验收文档）
  - `experiments/agc_runs/_m3_batch_*`（新增证据目录）
- 只读保留：`scenario_flat.yaml`、`start_agc_collab.sh`、`audit_interfaces.py`、`summarize_runs.py`、全部实时链路 C++ 节点、无人机单体与论文实验资产。

### 实现语言理由（Python 例外登记）

新增核验为**离线日志后处理**，不订阅/发布任何实时话题、不参与控制时序，属于工程约定中「明确不在实时链路（离线工具）」的允许情形；实时链路节点一律保持 C++，本 Spec 不新增任何实时节点。

## ADDED Requirements

### Requirement: 受控对比的空中输入（仅改变目标）

系统 SHALL 提供一组「只改变空中目标输入」的受控对比场景：处理组场景与对照组场景在 `car_start`、`target_start`、`obstacles`、`map`、`thresholds` 上逐字段一致，仅在 `target_motion` 上不同；处理组 `target_motion.mode=linear`，目标速度 SHALL 低于 `commander.max_linear_speed_mps`（1.0 m/s）并完成可达性说明，避免用物理上无法追上的轨迹评估控制器。

#### Scenario: 场景仅目标运动不同
- **WHEN** 解析 `scenario_flat.yaml`（对照）与 `scenario_linear_target.yaml`（处理）
- **THEN** 两者 `car_start`/`target_start`/`obstacles`/`map`/`thresholds` 完全一致，仅 `target_motion` 不同
- **AND** 处理组 `target_motion.mode=linear` 且 `speed_mps ≤ 1.0`

#### Scenario: 处理组目标确实在运动
- **WHEN** 读取处理组 run 任务活动窗口内的 `target_trajectory.jsonl` 或 `topics/_agc_targets.jsonl`
- **THEN** 目标地面位置 `(x, y)` 的变化范围 ≥ 1.0 m

### Requirement: 空中目标改变驱动地面规划更新

系统 SHALL 在目标输入改变时，使地面 `goal` 与 `path` 随之更新，并将每次重规划的触发原因、输入与规划结果落盘。目标/`goal` 的跟随关系按**任务活动窗口**评估，并排除 `/agc/scenario/reset` 后 2.0 s 的目标瞬移稳定期。

#### Scenario: goal 跟随目标（处理组）
- **WHEN** 在同一处理组 run 的**任务活动窗口**（任务状态非 `IDLE` 的首/末时刻）内对齐 `/agc/targets` 与 `/agc/ground/goal` 样本，并排除每次 `/agc/scenario/reset` 后 2.0 s（复位按设计把目标写回 `target_start`，其瞬移与目标运动无关）
- **THEN** `goal` 与目标地面投影的中位距离 ≤ 0.5 m、最大距离 ≤ 1.5 m（差额来自 `safeApproachPoint` 在障碍保护距离内的回退）
- **AND** 该 run 内 `goal` 位置变化范围 ≥ 1.0 m

#### Scenario: goal 恒定（对照组）
- **WHEN** 读取对照组 run 任务活动窗口内的 `/agc/ground/goal`
- **THEN** `goal` 位置变化范围 ≤ 0.2 m，且目标位置变化范围 ≤ 0.1 m

#### Scenario: 重规划原因落盘
- **WHEN** 读取 `planner_log.jsonl`
- **THEN** 每条规划记录（`trigger≠invalid`）含 `trigger`、`goal`、`car_pose`、`map_version_hash`、`plan_time_ms`、`n_waypoints`、`success`、`failure_reason`
- **AND** 处理组 run 至少出现 1 次 `trigger="displacement"`（目标位移 >1 m 触发），对照组 run 不出现 `displacement`，其 `trigger` 仅取 `first`/`period`/`infeasible`/`invalid`（`invalid` 表示该拍输入无效、未发布路径）

### Requirement: 规划更新驱动车运动（执行闭环）

系统 SHALL 证明地面动作确由更新后的规划驱动：命令与实际车体运动随 `goal`/`path` 变化而变化。

#### Scenario: 车确实运动
- **WHEN** 读取处理组 run 的 `/agc/ground/odom` 与 `/agc/ground/cmd_vel`
- **THEN** 车辆实际位置相对起点的最大位移 ≥ 3.0 m
- **AND** 在 `GROUND_APPROACH` 期间存在非零 `/agc/ground/cmd_vel.linear.x`

#### Scenario: 运动方向与 goal 方向一致
- **WHEN** 比较车辆实际位移向量与「车→goal」方向
- **THEN** 位移方向与 goal 方向的平均同向性为正（方向一致性），即车辆朝 `goal` 运动而非背离

### Requirement: 无信息时安全停车

系统 SHALL 在失去空中目标输入后使地面立即停车并保持不动。

#### Scenario: 目标消失后停车
- **WHEN** 目标输入停止（`/agc/targets` 变为空数组）后
- **THEN** 在 ≤1.0 s（`goal_max_age_s`）内 `/agc/ground/cmd_vel` 归零
- **AND** 自最后一次有效目标样本起，车辆实际位置位移 ≤ 0.10 m
- **AND** run 结束时任务状态为 `MISSION_FAILED` 且 `reason_code` 为 `TARGET_LOST`（或等价的失联终态）

### Requirement: 离线因果闭环核验与证据

系统 SHALL 提供离线核验工具，对上述因果链逐项判定 PASS/FAIL 并落盘，作为 M3 可核查证据。

#### Scenario: 核验输出与退出码
- **WHEN** 以对照 run、处理 run、无信息 run 与两个场景文件为输入运行核验
- **THEN** 打印 PASS/FAIL 表，写出结果 JSON，且任一 FAIL 时退出码为 1

#### Scenario: 证据可复算
- **WHEN** 读取三个 run 的 `meta.json`
- **THEN** 可复算 `agc_params_hash` 与 `scenario_hash`，且对照组与处理组的 `agc_params_hash` 相同

## 非目标（Non-goals）

- 不推进 M4 真实感知/多源融合：本轮 M3 只用可控仿真真值适配器 `gt_adapter` 验证接口，不接入真实空中感知输出。
- 不做 M5 控制权交接、M6 多场景（静态障碍/目标移动/观测丢失/不可达四类完整覆盖）、M7 单体性能、M8 实物联调。
- 不做阶段 B 拦截机构、接触判据与 `INTERCEPT_CONFIRMED`；`TRACKING_SUCCESS` 不等于拦截完成。
- 不以「改变障碍/地图输入」验证因果（阶段 A 地图来自场景、静态；该路径属 M2/M4），本轮只改变**目标**输入。
- 不修改任何实时链路节点、`scenario_flat.yaml` 与已冻结接口。
