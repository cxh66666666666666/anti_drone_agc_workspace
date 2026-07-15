# Tasks

- [x] Task 1: 重构 `generateFixedHorizontalTrajectory()` — 统一角速率 + 接入直线段
  - [x] 将 `theta_total` 的计算从 `2π*t/duration` 改为 `angular_velocity * t`
  - [x] 取消归一化时间，直接用绝对角度判断所处的轨迹段
  - [x] 计算完整周期长度：`T_total = (2*π*r + 2*straight_length) / (r*omega)`
  - [x] 将轨迹分为4段：右半圆(θ: -π/2→π/2)、上直线、左半圆(θ: π/2→3π/2)、下直线
  - [x] 每段按恒定线速度均匀分布采样点
  - [x] 直线段位置用线性插值，速度用方向向量×线速度，加速度为0
  - [x] 移除 `linear_vel` 等无用变量（若还存在）

# Task Dependencies
- 无依赖，单一文件修改
