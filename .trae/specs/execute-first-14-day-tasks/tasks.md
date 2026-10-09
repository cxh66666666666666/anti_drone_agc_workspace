# Tasks

> 目标：执行 [publication_plan.md](../../../docs/指导书/publication_plan.md) §9.3「前 14 天具体任务」四个子阶段。
> 日志统一写入 `docs/指导书/stage_logs/one/`；实验脚本/协议/原始数据写入 `experiments/paper_v3/`。
> 约束：不改写旧源码与旧实验结果，不删除任何目录，仿真一律 headless。

- [x] Task 1: 建立可解释的起点（days 1—3）
  - [x] SubTask 1.1: 编写冻结脚本 `experiments/paper_v3/scripts/freeze_baseline.py`：对关键源码、checkpoint 配置与模型、实验索引、场景清单计算 SHA-256 与字节数，记录 `git_revision` 与 UTC 时间戳，输出 `docs/指导书/stage_logs/one/frozen_manifest.json`；缺失文件记 `null`，重复运行结果一致。
  - [x] SubTask 1.2: 运行冻结脚本生成清单，核对关键源码/checkpoint 哈希非空且与现场重算一致。
  - [x] SubTask 1.3: 写 `docs/指导书/stage_logs/one/01_baseline_audit.md` 的「33/38 维与 masked/raw 差异」：给出 `observation_space_dim`、`PATH_FEATURES`、`num_sectors` 源码行与 `training_runs/*/config.json` 的 `observation_dim`，列出 b2/b2m/a1/p 的 `action_interface` 映射与 raw 档行为。
  - [x] SubTask 1.4: 在同一日志补「唯一真实执行链路」：任务路径 → 候选参考 → 控制器 → 位置设定点 → PX4 内环 → 机体，逐层标注物理量、话题、频率、源码位置与延迟（引用 `control_validation_protocol_v1.md` 的冻结值）。
  - [x] SubTask 1.5: 补 `_safety_obstacles()` 真值权限复查结论（是否/何时读 `scenario_obstacles_typed`、可否支持纯 LiDAR 主张）。
  - [x] SubTask 1.6: 补 `cross_validation_ok` 拆分：把现有判据归为「实现一致性检查」，并具体描述缺失的「对齐预测发出时刻/实际输入/未来观测」验证；确认未改动任何旧结果文件。
  - [x] SubTask 1.7: 验证 Task 1：`git status` 中旧源码/旧实验文件无改动；重复运行冻结脚本哈希一致。

- [x] Task 2: 验证「刹车」（days 4—7，实测部分）
  - [x] SubTask 2.1: 写制动/保持命令规范 `experiments/paper_v3/protocols/brake_hold_protocol_v1.md`：制动触发条件、制动的参考/命令定义、保持策略、终止判据（速度阈值 + 稳定窗口）、尾段观察时长。
  - [x] SubTask 2.2: 实现测试脚本 `experiments/paper_v3/scripts/brake_hold_test.py`：headless 启动仿真，无障碍，纵横向低速短段，从制动命令起持续记录到物理停止并继续尾段观察；输出逐次原始 CSV（时间戳、参考、实际位姿/速度、MAVROS 与 Gazebo 两路）。
  - [x] SubTask 2.3: 运行纵向 ≥3 次、横向 ≥3 次（0.5 m/s 级），原始数据存 `experiments/paper_v3/identification/brake_hold/`。
  - [x] SubTask 2.4: 写 `docs/指导书/stage_logs/one/02_brake_hold.md`：减速过程、最大漂移、命令生效延迟、失败形态、样本量与适用范围，并显式声明不沿用旧 `a_brake` p10。
  - [x] SubTask 2.5: 若仿真在限次尝试内无法产出有效数据，在日志中记录命令、原始报错与阻塞原因并标「未完成」，不得伪造或以旧值代替。

- [x] Task 3: 标准方法最小设计（days 4—7，文献部分）
  - [x] SubTask 3.1: 阅读安全过滤综述与 ≥2 篇最近邻原文（本地 PDF 或 arXiv），记录控制输入、内环假设、延迟处理、误差集合、备份策略、终端条件。
  - [x] SubTask 3.2: 写 `docs/指导书/stage_logs/one/03_b1_b2_design.md`：B1/B2 的状态/输入定义、预测模型、误差集合表示、约束判据、备份/终端接口；明确指出两者只差「误差集合表示/新增步骤」这一项。
  - [x] SubTask 3.3: 标注来源 DOI/arXiv 版本；未获原文的结论标「待核验」。

- [x] Task 4: 三个机制反例（days 8—10）
  - [x] SubTask 4.1: 定义三反例（同位置不同速度或命令历史 / 同误差模长不同方向 / 同路径不同执行滞后）的初态、控制与运行命令，落地 `experiments/paper_v3/mechanisms/`。
  - [x] SubTask 4.2: 给规则法与标准预测法同等制动能力后做对照运行，确认问题仍存在。
  - [x] SubTask 4.3: 写 `experiments/paper_v3/protocols/prediction_replay_format_v1.md`：预测保存字段（模型版本、实际初态、候选/备份输入序列、预测范围、真实未来状态）、输入回放对齐规则、扫掠净空下界判定。
  - [x] SubTask 4.4: 写 `docs/指导书/stage_logs/one/04_mechanism_counterexamples.md`：逐反例的初态/设置/运行命令/原始产物路径，区分物理事实与差模型，说明基线是否含制动。

- [x] Task 5: 第一次研究决策（days 11—14）
  - [x] SubTask 5.1: 写 `docs/指导书/stage_logs/one/05_nearest_neighbor_table_v1.md`：覆盖 DDSF、Layered Safety、Wabersich–Zeilinger 2021、Kousik RTD、Chen FaSTrack，按「假设 A / 限制 B / 新步骤 C / 收益 D / 代价 E / 检验实验 F」填写或标「填不出」。
  - [x] SubTask 5.2: 写 `docs/指导书/stage_logs/one/06_research_decision.md`：接口选择依据（保留位置参考接口或切换速度接口）与前 3 个月工作量预算。
  - [x] SubTask 5.3: 写 `docs/指导书/stage_logs/one/07_journal_requirement_check.md`：访问 URL、日期、结果（含 HTTP 403 等失败）与本地格式文件退路，不断言未核实的现行效力。
  - [x] SubTask 5.4: 写 `docs/指导书/stage_logs/one/08_advisor_memo.md`：一页「问题 / 证据 / 当前结论 / 下次能否证伪」。
  - [x] SubTask 5.5: 写 `docs/指导书/stage_logs/one/00_README.md`：stage_logs/one 索引 + 两周结论（旧模型在哪种可重复条件下失败、标准方法解决多少、剩余具体问题、未完成项）。

- [x] Task 6: 修复核验发现的两处数字不一致（核验后新增）
  - [x] SubTask 6.1: 修正 `docs/指导书/stage_logs/one/04_mechanism_counterexamples.md` §3.1 残差表：L=0.12/0.40/0.60 三行与 `experiments/paper_v3/mechanisms/results/ce3_delay_curve.csv` 不符（标签错位），按 csv 更正为 L=0.12→0.934(边际0.112)、L=0.40→1.161(0.339)、L=0.60→1.286(0.464)；并据 `residual_delay_marginal_m` 列复核「边际 ≈ V·L」这句话是否成立（V=1.4 时实际边际约为 0.65·V·L）。
  - [x] SubTask 6.2: 修正 `docs/指导书/stage_logs/one/02_brake_hold.md` §4：净漂移表（`meta.max_drift_m`）无法由 `trace.csv` 复算，且与 `04`/`ce1` 的 `net_drift_m`（Gazebo 真值、brake 首末样本）定义不同却未声明。需给出可复算的明确定义与来源、量化两者差异并加交叉说明，消除表面矛盾。
  - [x] SubTask 6.3: 复查 `02` 与 `04` 中其余数值表，凡声明为「来自原始数据」者必须给出复算规则或标注不可复算的原因。

# Task Dependencies

- Task 6 依赖 Task 1—5（核验发现的问题）。
- Task 2 依赖 Task 1（需先确认真实执行链路与接口）。
- Task 4 依赖 Task 2（需制动能力）与 Task 3（需 B1/B2 设计）。
- Task 5 依赖 Task 1—4。
- Task 3 可与 Task 2 并行（文献与仿真互相独立）。