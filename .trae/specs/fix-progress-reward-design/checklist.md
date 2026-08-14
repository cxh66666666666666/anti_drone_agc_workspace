# Checklist

- [x] 终端失败时 `reward -= self.terminal_failure_penalty`（累加而非赋值）
- [x] Progress reward 仅在 `obstacle_min >= self.safe_distance_norm` 时生效
- [x] `self.prev_dist_to_target` 始终更新（无论 progress reward 是否生效）
- [x] 终端成功奖励逻辑不变（`reward += self.avoidance_success_reward`）
- [x] `_clip_reward` 仍正常限制最终 reward