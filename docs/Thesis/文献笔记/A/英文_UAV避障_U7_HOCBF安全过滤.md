# B 类快读：HOCBF 安全过滤高速避障飞行（Zhang，U7）

> 记录日期：2026-09-07；背景（辅助）文献。

**High-Speed Vision-Based Flight in Clutter with Safety-Shielded Reinforcement Learning**——Jiarui Zhang, Chengyong Lei 等；arXiv:2602.08653 (2026)。

- **方法**：端到端 PPO + 基于模型的安全机制：训练时用物理先验（Dijkstra 全局引导 + 位置奖励 + CBF 安全奖励 + 课程学习）塑造奖励；部署时用**实时 HOCBF 安全过滤器把策略输出投影到可证安全集**强制避碰。
- **证据强度**：Isaac Lab 大规模并行仿真（1000 架四旋翼、16 个渐进场景）＋户外森林稠密障碍实机飞行，速度 7.5 m/s；有消融（Dijkstra/CBF 奖励、HOCBF 过滤器）与单次优化耗时统计。
- **作用**：与本人"残差 RL + 离散时间安全过滤"方案几乎同构，其"奖励引导 + 运行时投影安全过滤 + sim-to-real"是可直接对标的最强论据来源，建议论文重点引用并划界。