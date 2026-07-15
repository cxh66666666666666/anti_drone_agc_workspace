# 修复避障策略导致撞墙 Spec

## Why
上一轮修复了 `plan_once()` 的 AttributeError 崩溃，`/rl_planner/local_offset` 已正常发布。但无人机仍然撞墙，根因在策略层面有两个关键缺陷：

1. **控制指令冲突**：`drone_launch_controller` 起飞后进入 HOVER 状态，`hoverMaintainLoop()` 以 20Hz 持续向 `/mavros/setpoint_position/local` 发布当前 XY 悬停位置；同时 `lqr_tracker_node` 以 50Hz 向同一话题发布轨迹跟踪位置。PX4 收到两个节点交替发布的矛盾指令，实际效果被严重削弱。

2. **非紧急避障方向策略错误**：`compute_escape_offset()` 非紧急分支中，`sector.index(max(sector))` 总是返回"第一个"最空扇区（索引 0，即后方 ~-165°），导致无人机试图向**后方**偏移。而轨迹参考点将其向前拉，二者抵消，避障失效。正确做法是向障碍物两侧（±90°）中选择更空的一侧绕行。

## What Changes
- **rl_planner_node.py**：修复 `compute_escape_offset()` 非紧急分支方向选择逻辑，改为选择障碍物两侧中更空旷的一侧
- **rl_planner_node.py**：紧急避障时增大偏移量（`max_offset_xy` → 3.0m），并降低无人机接近速度（在偏移中加入减速分量）
- **lqr_tracker_node.cpp**：恢复安全状态时重置 `rl_offset_` 为零
- **drone_launch_controller**：起飞完成后向 LQR 交出控制权（发信号停止 hover 发布）— **BREAKING** 行为变更

## Impact
- Affected specs: `fix-obstacle-avoidance-crash`（已修复的 plan_once 崩溃）
- Affected code:
  - `src/rl_obstacle_avoidance/scripts/rl_planner_node.py`
  - `src/lqr_trajectory_tracker/src/lqr_tracker_node.cpp`
  - `src/drone_launch_controller/src/takeoff_controller.cpp`
  - `src/drone_launch_controller/include/drone_launch_controller/takeoff_controller.h`

## MODIFIED Requirements

### Requirement: 非紧急避障方向选择
`compute_escape_offset()` 在 min_dist 位于 [emergency_dist_norm, safe_distance_norm) 区间时，SHALL 选择障碍物两侧（垂直于最近障碍物方向）中更空旷的一侧作为逃逸方向，而非选择索引最小的空扇区。

#### Scenario: 前方有障碍物，左右均空
- **WHEN** 最近障碍物在正前方扇区（idx≈6），且左侧和右侧扇区均为空
- **THEN** 逃逸方向 SHALL 为左侧或右侧（±90°），偏移量正比于危险度

#### Scenario: 前方有障碍物，左侧有第二障碍物
- **WHEN** 最近障碍物在正前方，左侧扇区也有障碍物（距离更近），右侧空旷
- **THEN** 逃逸方向 SHALL 为右侧

### Requirement: 控制权交接
无人机完成起飞进入悬停后，`drone_launch_controller` SHALL 停止向 `/mavros/setpoint_position/local` 发布悬停 setpoint，将控制权完全交给 LQR 跟踪器。

#### Scenario: 起飞完成后 LQR 接管
- **WHEN** 起飞达到目标高度进入 HOVER 状态
- **THEN** `hoverMaintainLoop()` SHALL 停止发布 setpoint（或停止线程），避免与 LQR 轨迹位置产生控制冲突

### Requirement: 安全恢复时偏移归零
当 `compute_escape_offset()` 判定安全（min_dist ≥ safe_distance_norm）并发布零偏移量时，LQR 跟踪器 SHALL 将 `rl_offset_` 归零，不再叠加旧偏移。

#### Scenario: 障碍物消失后恢复跟踪
- **WHEN** 无人机已绕过障碍物，所有扇区 min_dist ≥ 0.4
- **THEN** `/rl_planner/local_offset` 发布 (0, 0, 0)，LQR 收到后 `has_rl_offset_` 设为 false，`rl_offset_` 归零

## REMOVED Requirements
无移除需求。
