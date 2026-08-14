# DQN 避障 Progress Reward 规范

## Why
当前 `compute_reward()` 每步仅产出 0.01-0.03 的微小奖励，终端由 `step()` 加上 ±30 的终端奖励。这导致 episode 总奖励几乎完全由终端结果决定（成功 ≈ +31，失败 ≈ -30），中间过程缺乏梯度信号，障碍物增多后模型难以学到有效避障策略。

## What Changes
- 在 `GazeboEnv.__init__` 中添加 `w_progress` 参数（默认 1.0）
- 在 `__init__` 和 `reset()` 中初始化 `prev_dist_to_target = None`
- 在 `compute_reward()` 中加入 progress reward：每步奖励加上 `w_progress * (prev_dist - curr_dist)`
- 在 `start_sim.sh` 训练模式中设置 `w_progress:=1.0` 参数

## Impact
- Affected specs: 无（新增功能）
- Affected code:
  - `src/rl_obstacle_avoidance/scripts/gazebo_env.py` — 主要修改
  - `scripts/start_sim.sh` — 参数传递

## ADDED Requirements

### Requirement: Progress Reward 参数
系统 SHALL 在 `GazeboEnv.__init__` 中通过 ROS 参数 `~w_progress` 读取 progress reward 权重，默认值为 1.0。

#### Scenario: 读取默认权重
- **WHEN** 未在 ROS 参数中设置 `~w_progress`
- **THEN** `self.w_progress` 应为 1.0

#### Scenario: 读取自定义权重
- **WHEN** ROS 参数 `~w_progress` 设置为 2.0
- **THEN** `self.w_progress` 应为 2.0

### Requirement: 上一帧距离追踪
系统 SHALL 在 `GazeboEnv.__init__` 中初始化 `self.prev_dist_to_target = None`，并在 `reset()` 方法中重置为 `None`，确保每个 episode 开始时重新累积 progress。

#### Scenario: Episode 开始时重置
- **WHEN** 调用 `reset()`
- **THEN** `self.prev_dist_to_target` 应为 `None`

#### Scenario: 首步无 progress
- **WHEN** episode 第一步调用 `compute_reward()` 且 `prev_dist_to_target` 为 `None`
- **THEN** progress reward 部分不生效，奖励不变

### Requirement: Progress Reward 计算
系统 SHALL 在 `compute_reward()` 末尾（`return` 之前）计算 progress reward：

```python
if self.prev_dist_to_target is not None and hasattr(self, 'w_progress'):
    curr_dist = self._tracking_error()
    progress = self.prev_dist_to_target - curr_dist
    reward += self.w_progress * progress
self.prev_dist_to_target = self._tracking_error()
```

#### Scenario: 靠近目标
- **WHEN** 当前 tracking error 比上一帧小（`prev_dist - curr_dist > 0`）
- **THEN** reward 增加 `w_progress * (prev_dist - curr_dist)`

#### Scenario: 远离目标
- **WHEN** 当前 tracking error 比上一帧大（`prev_dist - curr_dist < 0`）
- **THEN** reward 减少 `w_progress * |prev_dist - curr_dist|`

#### Scenario: 距离不变
- **WHEN** 当前 tracking error 与上一帧相同
- **THEN** reward 不变

### Requirement: 训练启动脚本参数
系统 SHALL 在 `start_sim.sh` 的训练模式中传递 `w_progress:=1.0` 参数给 `gazebo_env` 节点。

#### Scenario: 训练模式启动
- **WHEN** 以 `--training` 模式启动
- **THEN** `gazebo_env` 节点接收到 `w_progress:=1.0` 参数