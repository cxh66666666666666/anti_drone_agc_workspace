# Tasks

> 范围：根目录 `docs/指导书/stage_logs/two/publication_tasks.md` §2 前置补正 P0—P4。**不含 §3 的 N0—N4**（本轮完成前不开始下一阶段）。
> 交付目录：`docs/指导书/stage_logs/two/`；代码/数据用 `experiments/paper_v3/` 的新版本文件（不覆盖 v1）。
> 命名避让：N0—N4 已占用 `two/00_README.md` 与 `02`—`06`；本轮 P 交付物用 `01` 与 `07`—`12`。
> 旧 one 的原始 data/meta/trace 冻结不改；已有代码在带内容哈希的前提下以 v2 版本新增。
> **状态：P0—P4 全部完成（见 `two/12_p0_p4_summary.md`）。**

- [x] Task 1: P0 统一执行接口、备份与方法标签
  - [x] SubTask 1.1: 新建 `experiments/paper_v3/protocols/execution_contract_v2.md`：选定唯一主接口（直接 PX4 位置设定点台架），并逐项列出任务参考、实际发布的位置命令、速度反馈、历史命令、坐标系、单位、发布者、发布频率；每个量标注物理单位与来源。
  - [x] SubTask 1.2: 在同一契约中定义**共同备份**：何时把参考改到什么值、如何限幅、如何持续发布（满足 OFFBOARD 频率）、何时宣布任务失败、物理观察何时结束，以及多发布者的接管/释放规则。
  - [x] SubTask 1.3: 新建 `docs/指导书/stage_logs/two/01_design_corrections.md`（P0 部分）：固化 `Legacy`、`B1`、`B2`、`P` 标签；纠正旧文档「B1/B2 只差一个 κ 或一个标量集合」的表述。
  - [x] SubTask 1.4: 把接口结论改写为「本阶段选用的研究接口」；明确该台架**替换**原 LQR 外环、需重新辨识；不宣称已验证原 LQR＋PX4 组合系统。
  - [x] SubTask 1.5: 验证：从任一实际发布命令可推导下一步预测与对应备份；无单位混用（含旧 `max_offset_xy` 当速度的源码级问题已记录）、无未定义状态转移、无隐含真值输入。

- [x] Task 2: P1 制动采集/时间/指标口径修正（含小样例与异常样例）
  - [x] SubTask 2.1: 新建 `experiments/paper_v3/protocols/brake_hold_protocol_v2.md`：v2 时基、时间戳、指标定义、状态与进程管理口径，并列出对 v1 的修订点。
  - [x] SubTask 2.2: 新建 `experiments/paper_v3/scripts/brake_hold_test_v2.py`：`/use_sim_time=true` + `/clock` + 单调墙钟；启动失败/时钟跳变/时钟停滞独立状态；setpoint 发布序号/值/时间；MAVROS 位姿与速度的源时间戳与接收时间分开；Gazebo 接收时刻+对应仿真时刻并声明同步精度；只管理本会话进程（无全局 `pkill -9 -f`）；保存脚本哈希/参数/协议版本/发布者/终止原因。
  - [x] SubTask 2.3: 新建 `experiments/paper_v3/scripts/brake_hold_metrics_v2.py`：由日志重算沿向最大外扩、水平最大位移、终态净偏移、停稳确认时刻、尾段最大偏移、尾段最大速度；修复 `max_drift_m`/`tail_drift_m` 错名与单点计算。
  - [x] SubTask 2.4: 用 v2 采集小样例 `smoke_01` 到 `experiments/paper_v3/identification/brake_hold_v2/`，含完整尾段。
  - [x] SubTask 2.5: 采集丢帧样例 `drop_01`，被独立状态 `drop_frame` 正确标记，未误判为物理失败。
  - [x] SubTask 2.6: 协议中将「速度首次下降 5% 的时间」命名为**阈值响应时间**，并声明纯延迟须单独辨识。
  - [x] SubTask 2.7: 写 `docs/指导书/stage_logs/two/07_measurement_v2.md`：v2 口径、可复算汇总、丢帧样例标记结果。
  - [x] SubTask 2.8: 验证：旧 one 的 `identification/brake_hold/*` meta/trace 哈希未变（16 个 SHA-256 一致）。

- [x] Task 3: P2 三个反例与公平比较的证据等级更正
  - [x] SubTask 3.1: 在 `two/01_design_corrections.md` 增补 P2 部分：CE1/CE2/CE3 替代解释（CE1 实际无障碍轨迹＋假设净空评分；CE2 已知误差与椭圆集合例子分开；CE3 合成一阶速度模型）。
  - [x] SubTask 3.2: 区分物理碰撞（几何包络相交）与未满足设计裕量；CE2 的 0.10 m 净空未写成碰撞。
  - [x] SubTask 3.3: 新增 `FAIR_BRAKE_COMPARE_RETRACTION.md`，将 `all_survive=true` 标为不构成公平对照证据；历史 json 未修改（SHA-256 `209db7ea…b965`）。
  - [x] SubTask 3.4: 验证：`two/` 下索引与摘要不再引用该布尔汇总为实验通过；历史原始结果可追溯。

- [x] Task 4: P3 未来预测/回放协议 v2 与三个离线核对样例
  - [x] SubTask 4.1: 新建 `experiments/paper_v3/protocols/prediction_replay_format_v2.md`（不覆盖 v1）：三种任务分离、预测只用可得信息、日志分离、四类判据独立定义、几何检查前提、缺失/新鲜度/时基/插值/统计单位/隔离。
  - [x] SubTask 4.2: 实现三个离线样例到 `experiments/paper_v3/replay_examples/`：中途改命令、采样间曲线接近障碍、未知延迟边界；各含结果 JSON 与 README。
  - [x] SubTask 4.3: 验证：三个样例离线跑通可复算，真值仅在评分侧（`score_truth_*`/`truth` 字段）。

- [x] Task 5: P4 研究决策与来源材料校正
  - [x] SubTask 5.1: 写 `two/08_nearest_neighbor_table_v2.md`：以 B2 为最近邻主对照，P 差异限定为执行历史细化/分支保留与计算预算；核对 Wabersich–Zeilinger 2018（arXiv:1803.08552, CDC 2018）与 2021（arXiv:1812.05506, Automatica 129:109597）身份并纠正混引，修复链接。
  - [x] SubTask 5.2: 写 `two/09_research_decision_v2.md`：R0 决策在六周预研预算末；本轮不做确认性测试；预算与 84—140 h/四周一致（v1 的 110—180 h 作废）。
  - [x] SubTask 5.3: 写 `two/10_sources_archival.md`：只存档已获取内容并标日期，尚缺列待办；AIGC 政策核到作者章 1—6 条原句并标「交导师评估」。
  - [x] SubTask 5.4: 写 `two/11_advisor_memo_v2.md`：保留「未讨论/已讨论日期与反馈」真实状态；未编造认可；导师回复与学校认定非技术阻塞。
  - [x] SubTask 5.5: 写 `two/12_p0_p4_summary.md`：P0—P4 完成情况索引、未完成项、未开始 N0—N4 声明、未引用 `all_survive` 声明。

# Task Dependencies

- Task 2 依赖 Task 1（接口契约先定）。
- Task 3 依赖 Task 1（写在同一份 `01_design_corrections.md`）。
- Task 4 可独立并行（协议与样例不依赖接口）。
- Task 5 依赖 Task 1—3（标签与证据等级结论）。
