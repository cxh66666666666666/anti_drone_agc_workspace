# Checklist

- [x] DuelingQNetwork 类在 `dqn_agent.py` 中实现并可通过 pickle 保存/加载
- [x] Double DQN 的 TD target 使用在线网络选动作、目标网络估值
- [x] PrioritizedReplayBuffer 基于 SumTree 实现，支持优先采样和重要性采样权重
- [x] `train_dqn.py` 使用 PrioritizedReplayBuffer 替代普通 ReplayBuffer
- [x] ε 使用线性衰减，探索期更长且均匀
- [x] `compute_reward` 中 progress reward 权重提升，终端奖励幅度降低
- [x] 训练超参数默认值已更新（batch_size=64, buffer=100000, lr=5e-4, target_update=1000）
- [x] 学习率调度器在每个 episode 后更新
- [x] 梯度裁剪 max_norm 保持在合理范围（10.0）
- [x] 所有 .py 文件无语法错误（已通过 import 验证）