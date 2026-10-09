# A 类快读：Prioritized Experience Replay（Schaul，ICLR 2016）

> 记录日期：2026-09-07；方法源（本人 B2 基线组件，必引）。

**Prioritized Experience Replay**——Schaul T, Quan J, Antonoglou I, Silver D；ICLR 2016（arXiv:1511.05952，已下载）。

- **问题**：均匀回放重用低价值样本，学习慢。
- **方法**：按 TD 误差优先级采样（比例优先 p_i=|δ_i|+ε，或排序优先），并配**重要性采样权重** ω_j=(1/N·1/p_j)^β（β 退火）消除采样偏差。
- **关键结果**：Atari 上收敛显著加速、最终性能提升；与 DQN/DDQN 等正交可叠加。
- **与本人关系**：本人回放即 PER 实现，方法与超参逐项可比（可对照魏瑶 DRDQN、A-DDQN 两篇中文竞品）。
- **引用要点**：'采用 PER 按 TD 误差优先级采样并配重要性采样权重以加速收敛〔Schaul et al., 2016〕'。