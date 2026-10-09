# 读后记录：DQN（DeepMind，Nature 2015）——方法之源

> 记录日期：2026-09-07
> 重要级别：S 级（方法根源）——本人全部 DQN 系工作的源头，论文引言与方法部分必引，深度笔记便于写作时快速取用。

## 1. 基本信息

- **标题**：Human-level control through deep reinforcement learning
- **作者**：Mnih V, Kavukcuoglu K, Silver D, et al.（DeepMind）
- **期刊**：Nature, 2015, 518(7540): 529-533
- **电子版**：arXiv:1312.5602（Playing Atari with Deep Reinforcement Learning，已下载）

## 2. 核心方法（DQN）

- **经验回放（Experience Replay）**：将转移样本 (s,a,r,s′) 存入回放池随机采样，去相关、提高样本利用率；
- **目标网络（Target Network）**：延迟更新参数固定 Q̂，稳定训练：
  y = r + γ·max_{a'} Q̂(s′,a′;θ⁻)
- **CNN 端到端**：从原始像素学控制策略；
- 训练技巧：奖励修剪（clip）、帧跳等（Atari）。

## 3. 关键结果与意义

- 49 个 Atari 游戏中 29 个超人类水平（部分游戏）；
- 证实"端到端 + 稳定化技巧"能使 DQN 在复杂控制任务上达到人类级，开启深度强化学习落地时代。

## 4. 与本人工作的关系

- 本人外环（残差 DQN 变体）直接继承其**回放 + 目标网络**两大稳定化机制；
- 其对"过估计/不稳定"的讨论是 Double/Dueling/PER 三大改进的出发点（见 A 组三篇笔记）——本人在论文方法部分应用"DQN 三缺陷"叙事链条。

## 5. 引用要点

- 引言："深度强化学习已在 Atari〔Mnih et al., 2015〕等任务达到人类水平，激励其在机器人/无人机闭环控制中的探索；"
- 方法：经验回放与目标网络定义直接引用。

## 6. 行动项

- [ ] 若正文用 Nature 版引用格式，确认 DOI（10.1038/nature14236）入文献库。