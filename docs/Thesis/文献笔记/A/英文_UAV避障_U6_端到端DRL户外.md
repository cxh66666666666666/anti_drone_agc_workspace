# B 类快读：端到端 DRL 户外无人机运动规划（Doukhi，U6）

> 记录日期：2026-09-07；背景（辅助）文献。

**Deep Reinforcement Learning for End-to-End Local Motion Planning of Autonomous Aerial Robots in Unknown Outdoor Environments: Real-Time Flight Experiments**——Doukhi, Lee；Sensors 2021, 21, 2534。

- **方法**：GPS 拒止环境下 2D 激光（Hokuyo）+ 状态估计（Intel RT265）+ 目标信息，Actor-Critic（策略梯度）在 Gazebo 3D 仿真端到端训练，直接映射噪声状态与激光到连续前进速度/航向角速率，无建图无规划。
- **证据强度**：仿真（与 NMPC 比路径长度/定位噪声鲁棒性）＋真机实时飞行（NVIDIA Jetson TX2 板上运行），有飞行视频。
- **作用**：与本人项目最贴近（Gazebo 仿真训练 + 闭环 + 端到端 RL），其 Gazebo→真机 sim-to-real 落地流程与传感器配置可直接对标借鉴。