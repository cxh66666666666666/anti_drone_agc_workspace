# 项目任务日志完整核验与缺失修补 Spec

## Why
对 `docs/项目任务日志`（M0–M7）做完整核验后确认：证据目录、核验计数与哈希绝大多数真实一致，但存在断裂引用、陈旧表述、状态口径不一致，以及 M6 验收文档已披露但未修的复位独立性代码缺陷。需一次性补齐这些缺失，使任务日志自洽、可追溯。

## What Changes
- 修正 [M1_单体验收_v1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目任务日志/M1/M1_单体验收_v1.md) 两处 `agc_params.yaml` 断裂链接（错误路径 `src/air_ground_core/src/config/` → 正确 `src/air_ground_core/config/`）。
- 修正对不存在文件 `M0_接口契约_v1.0.md` 的引用（M0 v1.1/v1.2 与《项目总结》共 4 处）：改为标注 v1.0 已被 v1.1 取代、未随仓库保留，**不重建** v1.0。
- 更新 [项目总结_系统与进展.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目指导书/项目总结_系统与进展.md)：契约版本 v1.0→v1.2、`car_commander` 骨架表述、第五节“尚缺条件”按 M3–M6 已验收事实改写、文档索引路径修正。
- 更新 [下一阶段任务清单.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目指导书/下一阶段任务清单.md)：M5 行补【已验收】状态（其余 M0–M4/M6/M7 已标注）。
- **代码修复**：`ground_drive_adapter` 增加“任务非运行期不写本方车位姿”门控（与契约 §4「运行期唯一写入者」一致），消除同一栈内多次 attempt 复位不独立的问题（M6 §八.1 披露）。
- **重新验收**：编译通过 + 非无头多 attempt 复位独立性验证 + M5 控制权/终态停稳回归；据此更新 M1/M5/M6 文档证据与未决事项。

不涉及：阶段 B 拦截、真实感知、M8 实物、M9 选题；不改动 M0 已冻结话题/消息语义与坐标系/时间基准。

## Impact
- Affected specs: `agc-m0-v1.2` 契约文本引用；M1/M5/M6 验收证据记录；项目总结与任务清单状态口径。
- Affected code: [ground_drive_adapter.cpp](file:///home/ubuntu/anti_drone_agc_workspace/src/air_ground_core/src/ground_drive_adapter.cpp)；离线核验/编排脚本（复用现有 `verify_control_authority.py`、`run_m6_scenarios.py`）。
- Affected docs: M1 验收、M0 v1.1/v1.2、项目总结、下一阶段任务清单、M5/M6 验收（证据补记）。
- 不改动：`task_manager`/`ground_planner`/`ground_commander`/`air_control_bridge`/`scenario_reset`/`air_fusion` 等实时节点逻辑。

## ADDED Requirements

### Requirement: 任务日志引用完整性
任务日志与项目指导书中的全部相对链接与文件引用 SHALL 指向真实存在的文件。

#### Scenario: 链接可达
- **WHEN** 对 `docs/项目任务日志/**/*.md` 与 `docs/项目指导书/*.md` 做链接可达性检查
- **THEN** 无断裂相对链接（含 `#L` 行号锚点）

#### Scenario: v1.0 引用不再指向不存在文件
- **WHEN** 阅读 M0 v1.1/v1.2 与项目总结中关于 v1.0 的表述
- **THEN** 明确说明 v1.0 已被 v1.1 取代且未随仓库保留，不再存在指向缺失文件的链接

### Requirement: 任务状态口径一致
各任务里程碑状态 SHALL 与已交付验收文档一致。

#### Scenario: M5 状态标注
- **WHEN** 查看 `下一阶段任务清单.md` 的 M5 行
- **THEN** 标注【已验收】并附证据路径与权威批次

### Requirement: 复位独立性（同一栈内多 attempt）
`ground_drive_adapter` SHALL 仅在任务运行期写入本方车位姿；任务非运行期不写位姿，以保证 `scenario_reset` 写入的 `car_start` 不被覆盖，同一栈内后续 attempt 从场景初值开始。

#### Scenario: 非运行期不写位姿
- **WHEN** 任务状态为 `IDLE` 或任一终态
- **THEN** `ground_drive_adapter` 不调用 `set_model_state` 写 `agc_car`；积分器重同步到 Gazebo 实际位姿以备下次运行

#### Scenario: 运行期语义不变
- **WHEN** 任务状态 ∈ {`AIR_SEARCH`,`TARGET_ACQUIRED`,`GROUND_APPROACH`}
- **THEN** 20 Hz 运动学积分并写位姿；命令 watchdog（>0.2 s 零速）与写入权外部冲突检测（`INFRA_FAILURE/CONTROL_CONFLICT`）行为与修复前一致

#### Scenario: 同栈多 attempt 复位独立
- **WHEN** 单次启动内连续执行 ≥3 个 attempt
- **THEN** 每个 attempt 车实际起点均为场景 `car_start`（`(-4.000,-3.000)±1e-2`），不再出现“后续 attempt 起点=前一 attempt 终点”

### Requirement: 修复后回归验收
代码修复后 SHALL 重新取得可核查证据，证明无回归。

#### Scenario: 编译与多 attempt 独立性
- **WHEN** 重新 `catkin build` 并以非无头模式运行一次多 attempt 批次
- **THEN** 编译成功；每 attempt 起点独立；终态与 M0 口径恒等式成立；退出后无残留进程

#### Scenario: M5 控制权与停稳回归
- **WHEN** 以非无头模式复跑 M5 名义（及冲突注入）组
- **THEN** `/agc/ground/cmd_vel` 唯一发布者为 `ground_commander`、无 `CONTROL_CONFLICT` 误报、终态后零速停稳；冲突注入仍能被检出并结算 `CONTROL_CONFLICT`

## MODIFIED Requirements

### Requirement: M6 未决事项 1（复位独立性）
原文以“每回合一个全新栈”规避同栈复位不独立，并建议在 `ground_drive_adapter` 增加非运行期门控。修改为：该门控已落地并取得同栈多 attempt 起点独立的证据；M6 正文改为引用修复后的验证结果，历史现象保留为缺陷记录。

## REMOVED Requirements
无。