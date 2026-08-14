# Tasks

- [x] Task 1: 实现 Dueling Q-Network 架构
  - [x] 在 `dqn_agent.py` 中新增 `DuelingQNetwork` 类，共享特征层 (21→256→256→256)，分 value stream (256→1) 和 advantage stream (256→n_actions)
  - [x] 修改 `DQNAgent.__init__` 使用 `DuelingQNetwork` 替代 `QNetwork`
  - [x] 验证：运行 `python -c "from dqn_agent import DuelingQNetwork; m = DuelingQNetwork(21,5); print(m)"` 不报错

- [x] Task 2: 实现 Double DQN 更新逻辑
  - [x] 修改 `DQNAgent.update`，用在线网络选择最佳动作索引，再用目标网络取值
  - [x] 验证：检查 `update` 方法中 TD target 计算逻辑

- [x] Task 3: 实现 Prioritized Experience Replay
  - [x] 新增 `PrioritizedReplayBuffer` 类，基于 SumTree 实现优先采样
  - [x] 支持 `push` 带初始优先级、`sample` 返回 (batch, indices, weights)、`update_priorities` 更新
  - [x] 修改 `train_dqn.py` 使用 `PrioritizedReplayBuffer` 替代 `ReplayBuffer`
  - [x] 验证：运行 `python -c "from replay_buffer import PrioritizedReplayBuffer"` 不报错

- [x] Task 4: 优化 ε 衰减策略
  - [x] 在 `training_utils.py` 中新增 `epsilon_linear_at_step` 函数
  - [x] 修改 `train_dqn.py` 使用线性衰减
  - [x] 验证：`epsilon_linear_at_step(1.0, 0.05, 20000, 0)` → 1.0, `epsilon_linear_at_step(1.0, 0.05, 20000, 20000)` → 0.05

- [x] Task 5: 增强奖励塑形
  - [x] 修改 `gazebo_env.py` 中 `compute_reward`，增加 progress reward 权重，调整步级奖励
  - [x] 增加 `w_progress` 从 1.0 → 2.0，`safe_reward` 从 0.05 → 0.1
  - [x] 减小 `terminal_failure_penalty` 和 `avoidance_success_reward` 从 30.0 → 15.0
  - [x] 验证：检查 reward 计算逻辑

- [x] Task 6: 调整训练超参数
  - [x] 修改 `train_dqn.py` 默认值：`BATCH_SIZE=64`, `BUFFER_CAPACITY=100000`, `TARGET_UPDATE_STEPS=1000`, `LEARNING_RATE=5e-4`
  - [x] 默认 `epsilon_decay_steps` 改为总步数的 90%（而非 80%）
  - [x] 验证：确认参数已更新

- [x] Task 7: 添加学习率调度
  - [x] 在 `DQNAgent` 中添加余弦退火学习率调度 `schedule_lr`
  - [x] 每个 episode 后更新学习率
  - [x] 验证：学习率随训练逐步下降

# Task Dependencies
- Task 2 依赖 Task 1（Double DQN 需要 Dueling 网络）
- Task 3 独立（可并行）
- Task 4 独立（可并行）
- Task 5 独立（可并行）
- Task 6 独立（可并行）
- Task 7 依赖 Task 1（需要 DQNAgent 配合）