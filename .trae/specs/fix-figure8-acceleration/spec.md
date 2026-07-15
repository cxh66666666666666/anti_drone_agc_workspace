# 修复8字形轨迹直线段加速问题

## Why
8字形轨迹在飞行中"直线部分"出现明显加速。经全面排查，根因在**轨迹生成层**，RL和LQR层无问题。

## 根因分析

### 根因1：位置与速度的角速率不一致（持续加速）
[trajectory_generator.cpp](file:///home/ubuntu/anti_drone_agc_workspace/src/lqr_trajectory_tracker/src/trajectory_generator.cpp#L308-L310) 中位置角度由 `duration` 驱动，速度却由 `angular_velocity` 驱动：

```cpp
double theta_total = 2.0 * M_PI * normalized_t;  // 角速率 = 2π/duration ≈ 0.209 rad/s
velocity.x() = -params.radius * omega * std::sin(theta_right);  // 角速率 = omega = 0.1 rad/s
```

位置以 0.209 rad/s 移动（线速度0.628 m/s），但速度参考值只给 0.3 m/s。**速度参考永远追不上位置变化**，LQR控制器持续输出加速度来弥补速度误差。

### 根因2：半圆衔接处位置跳变（瞬时急加速）
右半圆终点 `(3, 3)` 和左半圆起点 `(-3, 3)` 之间有 6m 间隙。在 0.02s 的采样间隔内位置跳变6m（等效300 m/s），导致LQR产生巨大位置误差，触发最大加速度指令。

## 修改方案

### MODIFIED: 轨迹生成 — 统一角速率
- 位置角度改由 `angular_velocity * t` 计算（不通过 duration 归一化）
- 位置、速度、加速度三者在同一角速率下保持运动学一致
- 线速度恒为 `radius * angular_velocity`

### ADDED: 轨迹生成 — 接入直线段
- 两个半圆之间添加入直线段（上连接线 `(3,3)→(-3,3)`、下连接线 `(-3,-3)→(3,-3)`）
- 直线段速度大小与圆弧相同（恒定线速度），方向与圆弧终点切线一致
- 全程4段：右半圆 → 上直线 → 左半圆 → 下直线

### ADDED: 轨迹周期自适应
- 根据总路程和恒定线速度自动计算完整周期
- duration 参数仅决定采样点数，不影响角速率

## Impact
- Affected specs: 无现有spec
- Affected code:
  - [trajectory_generator.cpp](file:///home/ubuntu/anti_drone_agc_workspace/src/lqr_trajectory_tracker/src/trajectory_generator.cpp#L293-L357) — `generateFixedHorizontalTrajectory()`
  - LQR和RL层无需修改

## ADDED Requirements
### Requirement: 运动学一致的速度生成
系统 SHALL 使用同一个角速率参数 `angular_velocity` 同时驱动位置角度和速度计算，确保线速度大小恒定为 `radius * angular_velocity`。

#### Scenario: 匀速圆周段
- **WHEN** 生成圆弧轨迹点
- **THEN** 位置切向速度大小 SHALL 等于 `radius * angular_velocity`，且位置增量与速度增量一致

### Requirement: 连续无跳变的8字形轨迹
系统 SHALL 生成由两个半圆弧和两条直线段组成的连续8字形轨迹，相邻段交界处位置连续、速度方向一致。

#### Scenario: 半圆到直线过渡
- **WHEN** 右半圆弧到达终点 `(right_center_x, right_center_y + radius)`
- **THEN** 下一点 SHALL 在直线段上，速度方向向左，大小不变

### Requirement: 轨迹周期与duration解耦
系统 SHALL 根据总路程和恒定线速度自动计算完整周期，duration参数仅控制采样点数。

#### Scenario: 用户设置duration
- **WHEN** 设置 `trajectory_duration: 30.0` 且 `angular_velocity: 0.1`
- **THEN** 系统 SHALL 生成30秒的轨迹点，无论是否包含完整周期
