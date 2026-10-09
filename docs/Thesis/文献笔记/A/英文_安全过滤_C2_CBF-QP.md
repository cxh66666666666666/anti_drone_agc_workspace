# B 类快读：CBF-QP（Ames，C2）

> 记录日期：2026-09-07；背景（辅助）文献。

**Control Barrier Function Based Quadratic Programs for Safety Critical Systems**——Ames, Xu, Grizzle, Tabuada；arXiv:1609.06408。

- **方法**：将安全条件（CBF，保证集合前向不变）与性能目标（CLF，作软约束）统一进实时求解的二次规划（QP），以最小干预方式调解安全与性能冲突，得到反馈控制律。
- **证据强度**：自适应巡航（ACC）与车道保持仿真验证；指出 QP 型控制器已在双足行走实时运行。
- **作用**：CBF-QP"最小干预 + 前向不变"的正统出处，是本人安全过滤模块可直接引用的核心框架文献。