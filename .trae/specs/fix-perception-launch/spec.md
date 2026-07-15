# 修复 start_sim.sh 感知模式启动不完整 & 移除障碍物功能 Spec

## Why
`./scripts/start_sim.sh lqr --with-perception` 只启动了 `perception_node`（发布 `/obstacle_features`），但没有启动 `rl_planner_node`（发布 `/rl_planner/action`），导致 RL 规划链路不完整。同时用户不再需要脚本中的障碍物生成功能。

## What Changes
- `start_sim.sh` 中 `lqr --with-perception` 流程增加启动 `rl_planner_node`
- 从 `start_sim.sh` 中移除 `--with-obstacles` 选项及所有相关代码（障碍物加载逻辑、参数解析、帮助信息）
- 删除 `scripts/spawn_obstacles.py`（不再使用）

## Impact
- Affected specs: 无
- Affected code: `scripts/start_sim.sh`, `scripts/spawn_obstacles.py`

## MODIFIED Requirements

### Requirement: lqr 模式感知启动链路
`start_sim.sh lqr --with-perception` 模式下，系统 SHALL 依次启动：
1. `perception_node`（通过 `perception.launch`）— 发布 `/obstacle_features`
2. `rl_planner_node`（通过 `rl_planner_node.launch`）— 订阅 `/obstacle_features`，发布 `/rl_planner/action` 和 `/rl_planner/local_goal`

#### Scenario: 完整 RL 规划链路启动
- **WHEN** 用户执行 `./scripts/start_sim.sh lqr --with-perception`
- **THEN** `rostopic echo /rl_planner/action` 应能看到 action 消息发布

## REMOVED Requirements

### Requirement: --with-obstacles 障碍物加载
**Reason**: 用户不再需要此功能
**Migration**: 如需障碍物可手动运行 `python3 scripts/spawn_obstacles.py`
