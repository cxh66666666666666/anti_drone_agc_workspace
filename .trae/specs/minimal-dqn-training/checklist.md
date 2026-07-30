# Checklist

- [x] `replay_buffer.py` 的 `ReplayBuffer` 类支持 push / sample / __len__（已单元测试通过）
- [x] `sample()` 在缓冲区不足时优雅处理（返回 None 或少于 batch_size）
- [x] `dqn_agent.py` 的 `QNetwork` 结构为 16→128→128→6（两层 MLP + ReLU）
- [x] `DQNAgent` 维护在线网络和目标网络两套参数
- [x] `select_action()` 实现 epsilon-greedy，返回 int 动作
- [x] `update()` 使用 MSE loss + optimizer，返回 loss 浮点数
- [x] `sync_target()` 将在线网络参数 hard copy 到目标网络
- [x] `save()` / `load()` 使用 `torch.save` / `torch.load`
- [x] `train_dqn.py` 正确导入 `GazeboEnv`、`DQNAgent`、`ReplayBuffer`
- [x] 训练循环遵循：select → step → push → update → sync 流程
- [x] epsilon 按 `epsilon_decay` 从 1.0 衰减到 0.05
- [x] 每回合打印 episode / reward / steps / epsilon / result
- [x] 训练完成后 `model.pt` 文件生成且大小 > 0（79KB）
- [x] 训练完成后 `training_log.csv` 生成且包含 5 行
- [x] 5 episodes × 100 steps 训练全程无 Python 异常崩溃
- [x] loss 输出为有限正浮点数（非 NaN、非 inf）
