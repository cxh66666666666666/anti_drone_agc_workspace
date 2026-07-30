# Tasks

- [x] Task 1: 实现 replay_buffer.py
  - [x] 创建 `ReplayBuffer` 类，`__init__(capacity)`
  - [x] 实现 `push(state, action, reward, next_state, done)`（循环缓冲区）
  - [x] 实现 `sample(batch_size)` → 返回五元组或 None
  - [x] 实现 `__len__()` 返回当前经验数

- [x] Task 2: 实现 dqn_agent.py
  - [x] 实现 `QNetwork(nn.Module)`：Linear(16,128) → ReLU → Linear(128,128) → ReLU → Linear(128,6)
  - [x] 实现 `DQNAgent` 类构造函数：创建在线网络 + 目标网络 + 优化器
  - [x] 实现 `select_action(state, epsilon)`：epsilon-greedy，返回 int
  - [x] 实现 `update(batch)`：MSE TD loss 反向传播，返回 loss 值
  - [x] 实现 `sync_target()`：hard copy 在线网络到目标网络
  - [x] 实现 `save(path)` / `load(path)`：torch.save / torch.load

- [x] Task 3: 实现 train_dqn.py 训练脚本
  - [x] 初始化 `GazeboEnv` + `DQNAgent(16, 6)` + `ReplayBuffer(10000)`
  - [x] 实现 epsilon-greedy 衰减：`epsilon = max(epsilon_end, epsilon * epsilon_decay)`
  - [x] 每步执行完整流程：select → step → push → (optional) update → (optional) sync
  - [x] 每回合输出：episode、total_reward、steps、epsilon、result
  - [x] 训练结束后：保存 `model.pt`，写 `training_log.csv`
  - [x] 支持命令行参数覆盖 episodes/max_steps（可选，默认值即可）

- [x] Task 4: 极小训练验证
  - [x] 在仿真环境下运行 `python3 train_dqn.py`（episodes=5, max_steps=100）
  - [x] 确认 5 个 episode 完整执行，无异常
  - [x] 确认 loss 输出为有限正浮点数（0.0026 ~ 13.18）
  - [x] 确认 `model.pt` 生成且文件大小 79KB > 0
  - [x] 确认 `training_log.csv` 包含 5 行记录

# Task Dependencies
- Task 2 依赖 Task 1（update 需要 buffer.sample）
- Task 3 依赖 Task 1、Task 2（train_dqn 引用 buffer + agent + env）
- Task 4 依赖 Task 3（需要完整训练脚本才能跑）
