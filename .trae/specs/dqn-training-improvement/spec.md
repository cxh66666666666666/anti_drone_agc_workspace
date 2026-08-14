# DQN 训练效果改进 Spec

## Why
最近一次训练 (`dqn_20260813_211348_306998`) 的评估数据显示碰撞率 40-100%，成功率不稳定（0-60%），模型未收敛。根本原因包括：ε 衰减过快（3000 步即到最小值）、奖励信号过于稀疏二元化、网络架构过小、经验回放无优先级、探索策略不足。

## What Changes
- **Double DQN**：用在线网络选动作、目标网络估值，减少 Q 值高估
- **Dueling Network**：分离状态价值 V(s) 和动作优势 A(s,a)，更好地学习状态价值
- **Prioritized Experience Replay**：按 TD-error 优先级采样，优先学习关键经验
- **ε 衰减策略优化**：从指数衰减改为线性衰减，延长探索期
- **奖励塑形**：增加 progress reward、距离引导奖励，减少二元化
- **网络架构增强**：增加网络层数和宽度
- **学习率调度**：加入余弦退火或阶梯衰减
- **梯度裁剪增强**：降低 max_norm 防止梯度爆炸
- **训练超参数调优**：调整 batch_size、buffer_capacity、target_update 频率等

## Impact
- Affected specs: DQN 训练流程
- Affected code: `dqn_agent.py`, `train_dqn.py`, `replay_buffer.py`, `gazebo_env.py`

## ADDED Requirements

### Requirement: Double DQN
系统 SHALL 使用 Double DQN 算法，即用在线网络选择最佳动作，用目标网络评估该动作的 Q 值，替代当前直接用目标网络 max Q 值的方式。

#### Scenario: Q 值计算更准确
- **WHEN** 计算 TD target
- **THEN** 使用 `q_network(next_state).argmax()` 选动作，用 `target_network(next_state)[action]` 估值

### Requirement: Dueling Network Architecture
系统 SHALL 将 Q 网络改为 Dueling 架构，分离状态价值流 V(s) 和动作优势流 A(s,a)，最终 Q(s,a) = V(s) + A(s,a) - mean(A(s,·))。

#### Scenario: 网络结构
- **WHEN** 构建 Q 网络
- **THEN** 共享特征层后分两支：value stream (→1) 和 advantage stream (→n_actions)，合并为 Q 值

### Requirement: Prioritized Experience Replay
系统 SHALL 使用基于 TD-error 优先级的经验回放，采样概率 P(i) ∝ |δ_i|^α，并用重要性采样权重修正 bias。

#### Scenario: 优先采样关键经验
- **WHEN** 从 replay buffer 采样
- **THEN** 按 |TD-error| 加权的概率采样，返回 (batch, indices, weights)

### Requirement: 线性 ε 衰减
系统 SHALL 将 ε 从指数衰减改为线性衰减，使探索更均匀分布在整个训练过程中。

#### Scenario: ε 线性下降
- **WHEN** 计算当前 ε
- **THEN** ε = max(ε_end, ε_start - (ε_start - ε_end) * step / decay_steps)

### Requirement: 奖励塑形增强
系统 SHALL 在 `compute_reward` 中增加基于距离的 progress reward 权重，并调整步级奖励与终端奖励的比例，使奖励信号更稠密。

#### Scenario: 奖励信号更平滑
- **WHEN** 计算每步奖励
- **THEN** progress reward 权重从 1.0 提高到 2.0，减小终端奖励幅度（30 → 15），增加步级奖励（safe_reward 0.05 → 0.1）

### Requirement: 网络架构增强
系统 SHALL 将网络从 2 层 128 单元扩展到 3 层 256 单元，支持更大的表示能力。

#### Scenario: 更大的网络
- **WHEN** 构建 Q 网络
- **THEN** 网络结构为 21 → 256 → 256 → 256 → (value/advantage)，增加容量

### Requirement: 学习率调度
系统 SHALL 在训练过程中使用学习率调度，从初始 lr 逐步衰减。

#### Scenario: 学习率逐步下降
- **WHEN** 每个 episode 结束后
- **THEN** 按余弦退火调整学习率 lr = lr_min + 0.5 * (lr_max - lr_min) * (1 + cos(π * episode / total_episodes))

## MODIFIED Requirements

### Requirement: 训练超参数
训练超参数从固定值改为可配置，且默认值调整：
- epsilon_decay_steps: 3000 → 20000（或总步数的 80%）
- batch_size: 32 → 64
- buffer_capacity: 50000 → 100000
- target_update_steps: 500 → 1000
- learning_rate: 0.001 → 0.0005
- gamma: 0.99 → 0.99（不变）
- PER alpha: 0.6, beta: 0.4→1.0

#### Scenario: 超参数已优化
- **WHEN** 启动训练
- **THEN** 使用新的默认超参数值