# 极小 DQN 训练系统 Spec

## Why
`gazebo_env.py` 已实现标准的 `reset()/step()` 接口（16 维观测、6 离散动作）。`dqn_agent.py` 为空文件。需要在 Gazebo 仿真环境上跑通最小可用的 DQN 训练闭环——不追求收敛到避障策略，只验证训练流程能完整运转。

## What Changes
- 新增 `src/rl_obstacle_avoidance/scripts/replay_buffer.py`：经验回放缓冲区
- 重写 `src/rl_obstacle_avoidance/scripts/dqn_agent.py`：QNetwork + DQNAgent
- 新增 `src/rl_obstacle_avoidance/scripts/train_dqn.py`：训练主循环
- `dqn_agent.py` 使用 PyTorch（已在环境中可用）

## Impact
- Affected specs: 无（新功能）
- Affected code:
  - `src/rl_obstacle_avoidance/scripts/replay_buffer.py`（新增）
  - `src/rl_obstacle_avoidance/scripts/dqn_agent.py`（重写，原为空文件）
  - `src/rl_obstacle_avoidance/scripts/train_dqn.py`（新增）

## ADDED Requirements

### Requirement: 经验回放缓冲区
系统 SHALL 提供一个 `ReplayBuffer` 类，存储五元组 `(state, action, reward, next_state, done)`，支持 `push()` 和 `sample()`。

#### Scenario: 存入和采样
- **WHEN** 调用 `buffer.push(state, action, reward, next_state, done)` 存入若干条经验
- **THEN** `buffer.sample(batch_size)` SHALL 返回 batch_size 条经验，每条包含上述 5 个字段

#### Scenario: 容量限制
- **WHEN** 存入经验数超过 `capacity`
- **THEN** 最早的旧经验 SHALL 被覆盖（循环缓冲区）

#### Scenario: 样本不足
- **WHEN** 缓冲区中经验数 < batch_size
- **THEN** `sample()` SHALL 返回 None 或当前全部经验

### Requirement: DQN 智能体
系统 SHALL 在 `dqn_agent.py` 中提供 `QNetwork`（神经网络）和 `DQNAgent`（智能体）两个类。

#### Scenario: QNetwork 结构
- **WHEN** 创建 `QNetwork(state_dim=16, action_dim=6)`
- **THEN** 网络 SHALL 为两层 MLP：`Linear(16, 128)` → `ReLU` → `Linear(128, 128)` → `ReLU` → `Linear(128, 6)`

#### Scenario: select_action epsilon-greedy
- **WHEN** 调用 `agent.select_action(state, epsilon=0.1)`
- **THEN** 以 0.9 概率选 Q 值最大的动作，0.1 概率随机动作

#### Scenario: update 更新 Q 网络
- **WHEN** 调用 `agent.update(batch)` 传入一批经验
- **THEN** SHALL 计算 TD error（MSE loss），反向传播更新在线网络参数，返回 loss 值

#### Scenario: sync_target 同步目标网络
- **WHEN** 调用 `agent.sync_target()`
- **THEN** SHALL 将在线网络权重复制到目标网络（hard update）

#### Scenario: save / load 模型持久化
- **WHEN** 调用 `agent.save("model.pt")`
- **THEN** SHALL 将在线网络 state_dict 保存到文件
- **WHEN** 调用 `agent.load("model.pt")`
- **THEN** SHALL 从文件恢复在线网络和目标网络权重

#### Scenario: 双网络结构
- **WHEN** 创建 DQNAgent
- **THEN** SHALL 维护两个 QNetwork 实例：`q_network`（在线）和 `target_network`（目标）
- **THEN** 计算 TD target 时使用 `target_network`

### Requirement: 训练主循环
系统 SHALL 在 `train_dqn.py` 中提供训练主循环，连接 GazeboEnv 与 DQNAgent。

#### Scenario: 超参数配置
- **WHEN** 启动训练脚本
- **THEN** 默认超参数 SHALL 为：
  - `episodes = 5`
  - `max_steps = 100`
  - `batch_size = 32`
  - `buffer_capacity = 10000`
  - `epsilon_start = 1.0`
  - `epsilon_end = 0.05`
  - `epsilon_decay = 0.995`
  - `gamma = 0.99`
  - `lr = 1e-3`
  - `target_update_freq = 10`（每 10 step 同步一次）
  - `train_start = 64`（缓冲区满 64 条后才开始训练）

#### Scenario: 每步执行流程
- **WHEN** 训练循环中执行一步
- **THEN** SHALL 依次执行：
  1. `action = agent.select_action(state, epsilon)`
  2. `next_state, reward, done, info = env.step(action)`
  3. `buffer.push(state, action, reward, next_state, done)`
  4. 若 `len(buffer) >= train_start`：`loss = agent.update(buffer.sample(batch_size))`
  5. 若 `step % target_update_freq == 0`：`agent.sync_target()`

#### Scenario: 每回合输出
- **WHEN** 一个 episode 结束
- **THEN** SHALL 打印一行：
  `Episode X | reward: X.XX | steps: X | epsilon: X.XXX | result: success/collision/timeout`

#### Scenario: 训练日志记录
- **WHEN** 训练结束
- **THEN** SHALL 将每回合的 episode, total_reward, steps, epsilon, result 写入 `training_log.csv`

#### Scenario: 模型保存
- **WHEN** 训练结束（所有 episode 完成）
- **THEN** SHALL 调用 `agent.save()` 将模型保存为 `model.pt`

### Requirement: 极小训练验证
系统 SHALL 支持先跑 5 episodes × 100 steps 的极小训练，验证训练管线完整。

#### Scenario: 训练不崩溃
- **WHEN** 运行 `python3 train_dqn.py`（需 ROS 仿真环境就绪）
- **THEN** 5 个 episode 完整执行，无 Python 异常

#### Scenario: loss 能正常产生
- **WHEN** 训练进入 update 阶段
- **THEN** 输出的 loss 值为有限正浮点数，不是 NaN 或 inf

#### Scenario: 模型保存
- **WHEN** 训练完成
- **THEN** 当前目录下生成 `model.pt` 文件，大小 > 0

#### Scenario: 日志记录
- **WHEN** 训练完成
- **THEN** 当前目录下生成 `training_log.csv`，包含 5 行 episode 记录
