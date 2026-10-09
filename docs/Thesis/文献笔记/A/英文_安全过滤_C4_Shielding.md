# B 类快读：Safe RL via Shielding（Alshiekh，C4）

> 记录日期：2026-09-07；背景（辅助）文献。

**Safe Reinforcement Learning via Shielding**——Alshiekh, Bloem, Ehlers, Könighofer, Niekum, Topcu；arXiv:1708.08611。

- **方法**：针对时序逻辑（GR(1)/安全自动机）规范综合反应式"盾（shield）"，两种接入方式——决策前提供合法动作、或决策后监测纠正违规动作，保证不破坏原学习器收敛性。
- **证据强度**：4 个仿真域（grid 世界、自动驾驶、Atari Seaquest 等），有 reward 曲线对照，无真机、无统计量。
- **作用**："RL + 在线安全盾"即 RL 侧安全过滤的早期代表，可直接引用支撑本人"残差 DQN 输出再经安全过滤"的设计动机（对比：本人为离散时间过滤与状态空间约束）。