# B 类快读：UAV CBF-QP 安全控制器（Panja，C8）

> 记录日期：2026-09-07；背景（辅助）文献。

**Control Barrier Function Based UAV Safety Controller in Autonomous Airborne Tracking and Following Systems**——Panja, Hoagg, Baidya；arXiv:2312.17215。

- **方法**：快速非协同目标跟飞场景，用 CBF-QP 构造安全控制器，最小干预地修改轨迹跟踪控制器输出，兼顾贴近跟随目标与防撞；可与已有飞行控制器直接搭配。
- **证据强度**：AirSim + PX4 自动驾驶仪 SITL 仿真，多场景/轨迹数值验证，无真机、无统计量。
- **作用**：与本人"SITL + PX4 闭环 + 防撞安全过滤"技术栈同构，其最小干预 CBF-QP 与 PX4 集成方式可直接借鉴。