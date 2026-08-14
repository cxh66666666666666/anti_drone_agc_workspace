# Tasks

- [x] Task 1: 修复终端失败惩罚覆写问题（`step()` 第 307 行）
  - [x] 将 `reward = -self.terminal_failure_penalty` 改为 `reward -= self.terminal_failure_penalty`

- [x] Task 2: 修复 progress reward 在避障机动中惩罚偏离问题（`compute_reward()` 第 381-388 行）
  - [x] 在 progress reward 条件中增加 `obstacle_min >= self.safe_distance_norm` 检查
  - [x] 更新注释说明仅在安全区域生效

# Task Dependencies
- Task 1 和 Task 2 可独立并行执行