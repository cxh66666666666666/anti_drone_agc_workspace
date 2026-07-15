# 修复无人机避障撞击问题 Spec

## Why
无人机已能通过激光雷达检测障碍物（perception_node 正常发布 ObstacleFeatures），但仍疯狂撞击障碍物。根因是 **rl_planner_node 的 `plan_once()` 调用了未定义的方法 `choose_action()` 和 `action_to_offset()`**，导致该节点在运行时抛出 AttributeError 崩溃，**从未发布 `/rl_planner/local_offset` 避障偏移量**。LQR 跟踪器虽然订阅了该话题，但永远收不到数据，避障完全失效。

## What Changes
- 修复 `rl_planner_node.py` 的 `plan_once()` 方法：将未定义的 `choose_action()` / `action_to_offset()` 调用替换为已实现的 `compute_escape_offset()`
- 添加 `features` 为 None 的防护检查（首次定时器触发时 features 可能未就绪）

## Impact
- Affected specs: 无现有 spec
- Affected code: `src/rl_obstacle_avoidance/scripts/rl_planner_node.py`（仅此一个文件）

## ADDED Requirements
无新增需求，属于 Bug 修复。

## MODIFIED Requirements
无修改需求。

## REMOVED Requirements
无移除需求。
