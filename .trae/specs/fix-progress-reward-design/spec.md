# DQN Progress Reward 修复规范

## Why
当前 progress reward 有两个严重问题：

1. **终端奖励覆写**：`step()` 第 307 行使用 `reward = -self.terminal_failure_penalty`（赋值）而非 `reward -=`（累加），导致失败时之前所有 progress reward 被丢弃，直接变为 -30。成功避障第一个障碍物后在第二个障碍物失败时得不到任何部分奖励。

2. **Progress reward 惩罚避障行为**：`_tracking_error()` 测量的是无人机偏离 LQR 参考轨迹的垂距。当无人机为避障而偏离轨迹时，tracking error 增大，progress reward 变为负值，实际上在惩罚我们希望强化的避障行为。

训练日志证实：12 个 episode 中仅 1 次成功，11 次碰撞全得 -30，progress reward 未能提供有效梯度。

## What Changes
- 将终端失败惩罚从赋值 `=` 改为累加 `-=`，保留中间 progress reward
- Progress reward 仅在安全区域（无近距障碍物）生效，避免惩罚避障行为
- 在 `start_sim.sh` 中同步更新参数

## Impact
- Affected specs: `add-progress-reward`（修改）
- Affected code:
  - `src/rl_obstacle_avoidance/scripts/gazebo_env.py` — `step()` 和 `compute_reward()`

## MODIFIED Requirements

### Requirement: 终端失败惩罚不覆写累积奖励
系统 SHALL 在 `step()` 中失败时将终端惩罚以累加方式加入，而非赋值覆盖。

**原代码（第 304-307 行）：**
```python
elif self.last_done_reason in ("collision", "diverged",
                               "low_altitude", "timeout"):
    reward = -self.terminal_failure_penalty
```

**改为：**
```python
elif self.last_done_reason in ("collision", "diverged",
                               "low_altitude", "timeout"):
    reward -= self.terminal_failure_penalty
```

#### Scenario: 避障后失败保留中间奖励
- **WHEN** 无人机成功避开第一个障碍物（累积 progress reward），在第二个障碍物碰撞
- **THEN** 最终 reward = progress_reward_sum - 30，而非 -30

### Requirement: Progress reward 仅在安全区域生效
系统 SHALL 仅在 `obstacle_min >= safe_distance_norm` 时计算 progress reward，避免在避障机动期间因偏离轨迹而受到惩罚。

**修改 `compute_reward()` 中 progress reward 部分：**
```python
# progress reward: 仅在无近距障碍物时生效，避免惩罚正常避障偏离
if (self.prev_dist_to_target is not None
        and hasattr(self, 'w_progress')
        and obstacle_min >= self.safe_distance_norm):
    curr_dist = self._tracking_error()
    progress = self.prev_dist_to_target - curr_dist
    reward += self.w_progress * progress
self.prev_dist_to_target = self._tracking_error()
```

#### Scenario: 避障机动中不惩罚
- **WHEN** 无人机前方有障碍物（`obstacle_min < safe_distance_norm`），为避障偏离轨迹导致 tracking error 增大
- **THEN** progress reward 不生效，不会因偏离轨迹而产生负奖励

#### Scenario: 无障区域鼓励前进
- **WHEN** 无人机前方无障碍物（`obstacle_min >= safe_distance_norm`）
- **THEN** progress reward 正常生效，靠近目标为正，远离为负