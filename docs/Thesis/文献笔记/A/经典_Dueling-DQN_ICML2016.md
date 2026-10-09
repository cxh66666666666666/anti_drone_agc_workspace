# A 类快读：Dueling DQN（Wang，ICML 2016）

> 记录日期：2026-09-07；方法源（本人 B2 基线组件，必引）。

**Dueling Network Architectures for Deep Reinforcement Learning**——Wang Z, Schaul T, Hessel M, et al.；ICML 2016: 1995-2003（arXiv:1511.06581，已下载）。

- **问题**：许多状态下动作选择对价值影响小，单一 Q 网络学习低效。
- **方法**：Q 分解为状态价值 V(s) 与优势 A(s,a) 两流后汇聚：
  Q(s,a) = V(s) + A(s,a) − mean_{a′}A(s,a′)；价值流在新状态可直接估计，提高样本效率与泛化。
- **关键结果**：Atari 多数游戏超 DQN；对动作选择不敏感状态学习更稳健。
- **与本人关系**：本人外环即 Dueling 结构（B2 基线另一半），方法部分引用；优势分解思想也可用于解释"安全过滤只在危及动作上干预"。
- **引用要点**：'采用 Dueling 结构将 Q 解耦为状态价值与优势流，提升样本效率〔Wang et al., 2016〕'。