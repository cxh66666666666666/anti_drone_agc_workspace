# A 类快读：Double DQN（van Hasselt，AAAI 2016）

> 记录日期：2026-09-07；方法源（本人 B2 基线组件，必引）。

**Deep Reinforcement Learning with Double Q-learning**——van Hasselt H, Guez A, Silver D；AAAI 2016, 30(1): 2094-2100（arXiv:1509.06461，已下载）。

- **问题**：标准 DQN 用 max 算子导致 Q 值系统性高估，在部分环境下显著恶化策略。
- **方法**：解耦动作选择与价值评估——评估网络选动作，目标网络评价值：
  y = r + γ·Q̂(s′, argmax_a Q(s′,a;θ); θ⁻)，可证上界抑制过估计。
- **关键结果**：Atari 上降低过估计、部分游戏显著提升，训练稳定。
- **与本人关系**：本人外环采用 Double 机制（对应 B2 基线一半），论文方法部分直接引用；可与魏瑶 DRDQN 笔记（中文竞品同机制）互相参照。
- **引用要点**：'Double DQN 通过双网络解耦选择与评估以缓解 Q 值过估计〔van Hasselt et al., 2016〕'。