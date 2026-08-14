# DQN 避障训练

## 启动

先启动只含感知、不含规则规划器的训练环境。障碍物坐标可按场景替换：

```bash
./scripts/start_sim.sh lqr --training --headless \
    --obstacle 0,-3,5 --obstacle 0,3,5
```

仿真完成起飞并开始 LQR 跟踪后，在另一个终端启动训练：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/rl_obstacle_avoidance/scripts/train_dqn.py
```

默认训练 300 个 episode，每个 episode 最多 150 步。epsilon 按环境步线性衰减，
默认在总训练预算的 90% 时从 `1.0` 降至 `0.05`（可通过 `--epsilon-decay-steps` 显式指定衰减步数）。
快速检查管线时可以显式缩小预算：

```bash
python3 src/rl_obstacle_avoidance/scripts/train_dqn.py \
    --episodes 5 --max_steps 20 --eval-interval 5 --eval-episodes 1
```

快速检查只验证训练管线，不用于判断策略效果。

## 训练产物

每次运行默认创建独立的 `training_runs/dqn_<时间>/` 目录：

- `config.json`：命令行、随机种子、超参数和实际环境参数（含观察/奖励版本、经验回放类型）。
- `training_log.csv`：含探索动作的训练指标。
- `evaluation_log.csv`：固定 `epsilon=0`，且不更新网络或回放缓冲区。
- `step_log.csv`：逐步级日志（动作、奖励、终止原因、障碍距离、偏移量等）。
- `best_model.pt`：按成功率、碰撞率、平均奖励依次选择的最佳评估模型。
- `last_model.pt`：最后状态；正常结束、重置失败中止和 Ctrl+C 都会保存。
- `status.json`：结束状态、完成 episode 数和总训练步数。

通过 `start_sim.sh --training` 启动时，`config.json` 还会记录障碍物坐标、轨迹类型和飞行高度。

根目录旧的 `model.pt` 和 `training_log.csv` 属于旧版流程，不会被新训练读取或覆盖。
测试策略时优先使用同一运行目录中的 `best_model.pt`，并结合
`evaluation_log.csv` 确认它对应的评估结果。

当前环境使用 21 维观测和 5 个动作。21 维观测组成为：12 维扇区最小距离
（旋转到轨迹运动方向）+ 归一化位置 3 维 + 归一化跟踪误差 1 维 + 上一步动作
one-hot 5 维。动作分别为沿原路径、左移、右移、左移并上升、右移并上升；
横向幅值按障碍距离在 `0.3-1.2 m` 间缩放并经过低通平滑。
旧版 16 维/6 动作模型与此结构不兼容，需要重新训练。

DQN 实现细节：Dueling Q 网络（三层 256 隐藏单元 + value/advantage 分流）、
Double DQN 目标、优先经验回放（PER，`alpha=0.6`、`beta=0.4`、容量 100000）、
SmoothL1 损失、Adam 优化器。默认超参数：学习率 `5e-4`、折扣因子 `gamma=0.99`、
batch size `64`、目标网络同步周期 `1000` 步、训练起始步数 `1000` 步。

## 指标语义

`done_reason=timeout` 只表示跑满了最大步数，不等同于成功。只有同时满足以下条件，
环境才会立即以 `done_reason=avoidance_complete` 结束，并把训练日志记为 `success`：

1. 本回合曾进入 `3.5 m` 避障区，确认确实遇到过障碍；
2. 障碍距离随后恢复到 `3.5 m` 以外；
3. LQR 跟踪误差恢复到 `1.0 m` 以内；
4. 安全距离与跟踪恢复连续保持 10 步。

成功完成时发放一次终局奖励（默认 `~avoidance_success_reward = +25`，随后按 `±30` 裁剪）。
碰撞、发散、低高度和超时终止会扣除 `~terminal_failure_penalty`（默认 `-20`）。
开局一直无障碍不会触发成功奖励，恢复过程中再次靠近障碍会重新累计连续安全步数。

默认距离配置为：LiDAR 最小量程 `0.5 m`、碰撞风险 `0.6 m`、紧急距离
`1.5 m`、安全距离 `3.5 m`。启动时会检查这些阈值的顺序；碰撞阈值落入
LiDAR 盲区时训练会直接报错，不再生成无效日志。

`0.5 m` 最小量程沿用规则避障基线，用于过滤无人机机体自身回波。训练器还会
在第 20 个 episode 检查累计环境步数；若少于 100 步，会以
`aborted_invalid_environment` 中止，而不是把未更新的随机模型标记为训练完成。
reset 清零局部偏移后，会等待轨迹误差恢复且 360 度净空达到 `1.5 m`。近障碍表示
环境尚未准备好，不会立即把它误记为 reset 故障；只有等待超时后才记录失败。

训练表现包含 epsilon-greedy 探索，不能用于判断最终策略。以
`evaluation_log.csv` 的 `success_rate`、`collision_rate` 和
`timeout_rate`、`mean_min_obstacle` 为主要判断依据。
