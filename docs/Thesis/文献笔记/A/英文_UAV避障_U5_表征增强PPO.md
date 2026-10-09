# B 类快读：表征增强 PPO 无人机规划（Huang，U5）

> 记录日期：2026-09-07；背景（辅助）文献。

**Representation Enhancement-Based Proximal Policy Optimization for UAV Path Planning and Obstacle Avoidance**——Xiangxiang Huang；Int. J. Aerospace Engineering 2023, 6654130。

- **方法**：面向部分可观测 + 随机目标 + 连续动作的 3D 无人机规划。表征增强模块 RE＝OMI（感知特征与状态特征分离提取，GRU 增强观测记忆降碰撞）+ DRPAR（回合轨迹从绝对系变换到局部系压缩状态空间），配三种逐步奖励。
- **证据强度**：Unity3D 两个 3D 场景（柱状+墙体、随机目标）仿真，报告收敛/成功/超时/碰撞率对比，无真机。
- **作用**：GRU 记忆增强与"部分可观测下减少碰撞"思路可用于本人残差 RL 状态表征设计（关联度中等）。