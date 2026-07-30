# 固化规则避障 Baseline + 评价指标 Spec

## Why
规则避障系统（`rl_planner_node.py`）已完成三组最小实验验证（无障碍、单障碍、三障碍），实验日志系统（`experiment_logger_node.py`）也已就绪。但当前存在两个缺口：

1. 成功参数散落在 `rl_planner_node.launch` 和代码默认值中，没有集中的配置文件和文档说明，后续若要调整或对比 RL 策略时没有可靠的 baseline 参照。
2. 实验数据已有 CSV 记录，但缺少量化的评价指标体系和分析脚本，无法自动提取避障成功率、跟踪误差、路径长度等关键指标。

## What Changes
- 创建 `src/rl_obstacle_avoidance/config/rule_avoidance.yaml`：集中固化当前成功参数
- 修改 `rl_planner_node.launch`：引用 YAML 配置文件替代硬编码参数
- 更新项目 `README.md`：补充避障系统的启动顺序和成功场景说明
- 新增 `scripts/evaluate_experiment.py`：实验数据分析脚本，自动计算六项评价指标

## Impact
- Affected specs: `experiment-logger-node`（实验日志节点），`fix-obstacle-strategy`（避障策略修复）
- Affected code:
  - `src/rl_obstacle_avoidance/config/rule_avoidance.yaml`（新增）
  - `src/rl_obstacle_avoidance/launch/rl_planner_node.launch`（修改：引用 YAML）
  - `README.md`（修改：补充避障系统说明）
  - `scripts/evaluate_experiment.py`（新增）

## ADDED Requirements

### Requirement: 规则避障参数固化
系统 SHALL 提供一个 YAML 配置文件 `src/rl_obstacle_avoidance/config/rule_avoidance.yaml`，集中存储当前经过实验验证的避障参数，供 `rl_planner_node.launch` 引用。

#### Scenario: 配置文件包含所有关键参数
- **WHEN** 查看 `rule_avoidance.yaml`
- **THEN** 文件 SHALL 包含以下参数及其实验验证值：
  - `safe_distance_norm: 0.35`（安全距离归一值）
  - `emergency_dist_norm: 0.15`（紧急避障距离）
  - `min_offset_xy: 0.3`（最小水平偏移）
  - `max_offset_xy: 1.2`（最大水平偏移）
  - `max_offset_z: 0.3`（最大垂直偏移）
  - `step_xy: 1.5`（水平规划步长）
  - `step_z: 0.3`（垂直规划步长）
  - `min_height: 1.0`（最低安全高度）
  - `avoid_hold_time: 1.5`（避障方向保持时间）
  - `side_avoid_gain: 0.5`（侧方避让增益）
  - `offset_smooth_alpha: 0.8`（偏移低通滤波系数）
  - 并注明 `rl_velocity_gain: 1.0`（来自 LQR Tracker 的速度增益）作为关联参数

#### Scenario: Launch 文件引用 YAML 配置
- **WHEN** 执行 `roslaunch rl_obstacle_avoidance rl_planner_node.launch`
- **THEN** 参数从 `rule_avoidance.yaml` 加载，节点行为与实验验证时一致

### Requirement: README 避障系统文档
项目 `README.md` SHALL 包含规则避障系统的启动顺序说明和已验证的成功场景。

#### Scenario: 启动顺序说明
- **WHEN** 阅读 README 的避障系统章节
- **THEN** SHALL 包含完整的启动命令序列（仿真 → 起飞 → 感知+规划 → 实验日志）
- **THEN** SHALL 说明使用 `scripts/start_sim.sh` 一键启动的方法

#### Scenario: 成功场景说明
- **WHEN** 阅读 README 的验证结果章节
- **THEN** SHALL 列出已在实验中验证的三个场景：无障碍、单障碍、三障碍连续绕行

### Requirement: 实验评价指标定义与计算
系统 SHALL 提供一个实验数据分析脚本 `scripts/evaluate_experiment.py`，读取 `experiment_logger_node.py` 生成的 CSV 文件，输出六项量化指标。

#### Scenario: 计算避障成功率
- **WHEN** 对包含单障碍实验的 CSV 运行分析脚本
- **THEN** 系统 SHALL 判断是否成功避开障碍（无碰撞、offset 在避障结束后归零），输出 success/failure 及成功率

#### Scenario: 计算最小障碍距离
- **WHEN** 运行分析脚本
- **THEN** 系统 SHALL 基于 offset 数据和 action 状态，估算避障期间无人机与障碍物的最小距离（通过 offset 幅值反推）

#### Scenario: 计算跟踪误差
- **WHEN** CSV 中包含 `position_error` 字段的有效值
- **THEN** 系统 SHALL 输出全程和避障期间的 最大/平均跟踪误差

#### Scenario: 计算路径长度
- **WHEN** 运行分析脚本
- **THEN** 系统 SHALL 基于 `pose_x, pose_y, pose_z` 序列计算无人机实际飞行路径总长度

#### Scenario: 计算绕障耗时
- **WHEN** 运行分析脚本
- **THEN** 系统 SHALL 基于 `is_avoiding` 字段统计避障行为的总持续时间

#### Scenario: 计算 offset 抖动程度
- **WHEN** 运行分析脚本
- **THEN** 系统 SHALL 计算 `offset_magnitude_xy` 序列在避障期间的标准差，作为 offset 抖动程度的量化指标

#### Scenario: 批量分析多组实验
- **WHEN** 运行 `python3 scripts/evaluate_experiment.py experiment_data/*.csv`
- **THEN** 系统 SHALL 依次分析所有 CSV 文件，并输出汇总对比表格
