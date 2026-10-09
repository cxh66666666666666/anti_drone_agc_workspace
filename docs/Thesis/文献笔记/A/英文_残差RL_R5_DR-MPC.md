# B 类快读：DR-MPC 社交导航（Han，R5）

> 记录日期：2026-09-07；背景（辅助）文献。

**DR-MPC: Deep Residual Model Predictive Control for Real-world Social Navigation**——Han, Thomas, Barfoot 等；arXiv:2410.10646；IEEE RA-L 2025。

- **方法**：以 MPC 路径跟踪为基控制器，动作 a=α·a_HA+(1−α)·a_MPC（HA＝人群避障候选动作），初始 α 抑制使行为近似 MPC 再逐步释放；RL 用离散 SAC（DSAC）；配 OOD 状态检测（潜在空间 KNN）+ CVMM 安全性校验 + 启发式安全策略。
- **证据强度**：仿真优于传统 DRL 与残差 DRL；真机拥挤人群导航几乎无失误，仅 <4 小时训练数据，无需仿真数据。
- **作用**：MPC 基控制器 + "安全校验/启发式兜底"架构与本人"安全过滤 + 闭环"设计高度同构，可作安全层论点的直接借鉴。