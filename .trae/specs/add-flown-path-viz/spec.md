# 添加 RViz 飞行轨迹可视化 Spec

## Why
`./scripts/start_sim.sh lqr --with-perception` 运行后无法在 RViz 中看到无人机实际飞过的路径（只有参考轨迹 `nav_msgs::Path`）。需要添加实际飞行轨迹的 Marker 可视化。

## What Changes
- `lqr_tracker_node` 新增 `visualization_msgs::Marker` (LINE_STRIP) 发布实际飞行轨迹
- 更新 `lqr_tracker.rviz` 配置，增加飞行轨迹 Marker 和 RL 局部目标显示

## Impact
- Affected specs: 无
- Affected code: `src/lqr_trajectory_tracker/src/lqr_tracker_node.cpp`, `src/lqr_trajectory_tracker/include/lqr_trajectory_tracker/lqr_tracker_node.h`, `src/lqr_trajectory_tracker/rviz/lqr_tracker.rviz`

## ADDED Requirements

### Requirement: 实际飞行轨迹 Marker 可视化
系统 SHALL 在 `lqr_tracker_node` 中发布实际飞行轨迹为 `visualization_msgs::Marker` (LINE_STRIP 类型)，发布到 `/lqr_tracker/flown_path`。

#### Scenario: 飞行轨迹实时更新
- **WHEN** `lqr_tracker_node` 运行并收到位姿数据
- **THEN** 每次控制迭代将当前位置追加到 LINE_STRIP Marker points 中
- **AND** Marker 以红色线条显示在 RViz 中

### Requirement: RViz 配置更新
RViz 配置文件 SHALL 包含以下显示：
- `/lqr_tracker/flown_path` — 红色 LINE_STRIP，实际飞行轨迹
- `/rl_planner/local_goal` — 黄色小球，RL 规划器局部目标

#### Scenario: RViz 一键查看完整飞行状态
- **WHEN** 用户运行 `rviz -d src/lqr_trajectory_tracker/rviz/lqr_tracker.rviz`
- **THEN** 同时看到绿色参考轨迹、红色实际轨迹、RL 局部目标、无人机模型
