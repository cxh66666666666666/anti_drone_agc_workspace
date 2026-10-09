# B 类快读：轻量安全 RL 无人机导航（Zhang，U3）

> 记录日期：2026-09-07；背景（辅助）文献。

**Lightweight Safe Reinforcement Learning for End-to-End UAV Navigation**——Shenghui Zhang；arXiv:2607.01794。

- **方法**：把无人机避障建模为带约束 MDP，稀疏多线 LiDAR（1×36×4 距离张量）输入，轻量卷积编码器（非对称核 + 深度可分离卷积）提取碰撞风险特征，Lagrangian 安全 PPO 在层次化框架输出连续机体期望速度，配课程学习分阶段训练。
- **证据强度**：纯仿真（NVIDIA Isaac Sim 40×40m 森林，100/200/300 圆柱障碍），报告成功率/碰撞率/平均回报；Vanilla PPO 0.836→Safe RL 0.969（100 障碍），课程学习进一步稳定。
- **作用**："稀疏 LiDAR + 安全约束 PPO + 成功/碰撞率度量"与本人安全过滤体系在安全性与稀疏感知设计上可对标引用（对比自身离散安全过滤 vs 其 Lagrangian 软约束）。