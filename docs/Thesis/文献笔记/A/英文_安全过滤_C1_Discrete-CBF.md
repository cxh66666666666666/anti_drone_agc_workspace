# B 类快读：Discrete CBF（Agrawal，C1）

> 记录日期：2026-09-07；背景（辅助）文献。

**Discrete Control Barrier Functions for Safety-Critical Control of Discrete Systems with Application to Bipedal Robot Navigation**——Agrawal, Sreenath；RSS 2017。

- **方法**：将连续时间 CBF 扩展到离散时间域，分析倒数型与零化型两类 CBF 构造（含离散指数 CBF），得到非线性规划（条件满足时退化为 QCQP）；用于高维双足机器人穿越动态障碍环境的导航。
- **证据强度**：数值仿真（线性系统 + 双足导航），无真机、无统计量。
- **作用**：本人为离散时间安全过滤，此篇是"离散域 CBF"理论基石，可支撑离散安全过滤的可行性与安全性保证论证。