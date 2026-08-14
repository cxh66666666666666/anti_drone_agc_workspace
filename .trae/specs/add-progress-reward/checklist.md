# Checklist

- [x] `self.w_progress` 在 `__init__` 中通过 `rospy.get_param("~w_progress", 1.0)` 正确初始化
- [x] `self.prev_dist_to_target = None` 在 `__init__` 和 `reset()` 中正确初始化
- [x] `compute_reward()` 在 `return` 前正确计算 progress reward（靠近加、远离减）
- [x] 首步（`prev_dist_to_target` 为 `None`）时 progress reward 不生效
- [x] `start_sim.sh` 训练模式正确传递 `w_progress:=1.0` 参数
- [x] 不影响终端 ±30 奖励逻辑和 `_clip_reward` 行为