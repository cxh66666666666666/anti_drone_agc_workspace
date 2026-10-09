# B 类快读：Embodied Escaping 窄环境导航（Zheng，U4）

> 记录日期：2026-09-07；背景（辅助）文献。

**Embodied Escaping: End-to-End Reinforcement Learning for Robot Navigation in Narrow Environment**——Han Zheng 等；arXiv:2503.03208 (2025)。

- **方法**：面向扫地机器人狭窄死区脱困，SAC + Transformer 编码器，激光+IMU 端到端映射离散动作；均匀转弯半径重塑离散动作空间（42 个动作），用"动作掩码"快速剔除无效动作，A* 混合训练缓解稀疏奖励。
- **证据强度**：真机实验（激光+IMU）＋2D 仿真；成功率分解为规划/控制成功率统计，对比 SAC/PPO/ROS Navigation 等占优（有真机视频）。
- **作用**："动作掩码 + 离散动作空间重塑"可为本人残差 Double DQN 离散动作设计提供借鉴（关联度中等，非无人机、非安全过滤）。