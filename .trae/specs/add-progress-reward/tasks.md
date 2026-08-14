# Tasks

- [x] Task 1: 在 `gazebo_env.py` 中添加 progress reward 参数和状态追踪
  - [x] 在 `__init__` 中 `self.reward_clip_abs` 附近（第 99 行）添加 `self.w_progress = rospy.get_param("~w_progress", 1.0)`
  - [x] 在 `__init__` 中 reset 相关变量附近（第 153 行）添加 `self.prev_dist_to_target = None`
  - [x] 在 `reset()` 中 `self.episode_step = 0` 之后（第 218 行）添加 `self.prev_dist_to_target = None`
  - [x] 在 `compute_reward()` 末尾 `return` 之前（第 380-385 行）添加 progress reward 计算逻辑

- [x] Task 2: 在 `start_sim.sh` 训练模式中设置 `w_progress` 参数
  - [x] 在训练模式 `rosparam set` 区域（第 225 行）添加 `rosparam set /gazebo_env/w_progress 1.0`

# Task Dependencies
- Task 2 可独立于 Task 1 执行（无依赖关系）