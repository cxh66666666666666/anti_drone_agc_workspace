# B 类快读：四旋翼外环自适应 RL 控制（Saj，R6）

> 记录日期：2026-09-07；背景（辅助）文献。

**Adaptive Outer-Loop Control of Quadrotors via Reinforcement Learning**——Saj, Vemuri, Kalathil, Benedict（Texas A&M）；arXiv:2605.16015。

- **方法**：无人机级联控制——上层外环 RL 策略（PPO+RNN，处理部分可观测）输出期望推力与机体角速率，底层高带宽板载 PID 速率环执行；先用地面真值扰动训练最优外环策略，再用**残差动力学预测器（RDP，GRU）**在线估计外部力/力矩，配数据高效的线性标定桥 + 在线推力修正完成 sim-to-real。
- **证据强度**：仿真（8192 并行环境）＋真机 Crazyflie 2.X；与基础控制器、Domain Randomization 稳健控制器对比，在质量变化、非对称载荷、动态悬吊负载等大不确定下轨迹跟踪显著更优。
- **作用**：与本人（四旋翼 + 外环 RL + 内环 PID + SITL）话题最贴近，其"在线残差动力学/扰动估计 + 标定桥"思路可直接借鉴到 PX4/SITL 的安全过滤补偿环节。