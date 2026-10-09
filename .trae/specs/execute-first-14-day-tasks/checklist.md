# Checklist

> 核验方式：独立核验子代理逐条打开对应文件/数据核对，并复算关键数字与源码行号引用。
> 结论：全部通过（其中 2 处数字不一致由 Task 6 修复后复核通过）。

## Task 1：建立可解释的起点（days 1—3）

- [x] `docs/指导书/stage_logs/one/frozen_manifest.json` 存在，含关键源码、checkpoint 配置与模型、实验索引、场景清单的 SHA-256 与字节数，以及 `git_revision` 和 UTC 时间戳；缺失文件记为 `null`。（47 项：45 有哈希 / 2 记 null）
- [x] 冻结脚本重复运行（工作区内容不变）时哈希完全一致，实现幂等。（仅 `generated_utc` 变化，`files`/`checkpoint_summary` 逐项一致）
- [x] `01_baseline_audit.md` 给出 33/38 维差异，并能定位到 `observation_space_dim`/`PATH_FEATURES`/`num_sectors` 源码行与 `training_runs/*/config.json` 的 `observation_dim`。
- [x] `01_baseline_audit.md` 给出 masked/raw 差异与 b2/b2m/a1/p 接口映射，说明 raw 档距离掩码退出安全裁决。
- [x] `01_baseline_audit.md` 含唯一真实执行链路，逐层给出物理量、话题、频率、源码位置与延迟数值。
- [x] `01_baseline_audit.md` 含 `_safety_obstacles()` 真值权限复查结论（何时读真值几何、可否支持纯 LiDAR 主张）。
- [x] `01_baseline_audit.md` 含 `cross_validation_ok` 拆分：现有判据归为「实现一致性检查」，并写明缺失的未来预测验证。
- [x] 旧 `cross_validation.json`、`audit_index.*` 等既有分析产物内容未被改写。（哈希未变）

## Task 2：制动/保持规范与实测（days 4—7）

- [x] `experiments/paper_v3/protocols/brake_hold_protocol_v1.md` 含制动触发条件、参考/命令定义、保持策略、终止判据（速度阈值+稳定窗口）与尾段观察时长。
- [x] 制动实测原始数据落在 `experiments/paper_v3/identification/brake_hold/`（纵横向各 ≥3 次），或已如实记录仿真阻塞原因、命令行与报错。（纵 long_03/04/05、横 lat_01/02/03 均 `status=ok`；trace.csv 24 列、≈50 Hz）
- [x] `02_brake_hold.md` 给出减速过程、最大漂移、命令生效延迟、失败形态、样本量与适用范围，并声明不沿用旧 `a_brake` p10。（§4 经 Task 6 修复：明确两套口径与数据源）

## Task 3：B1/B2 最小模型级设计（days 4—7）

- [x] `03_b1_b2_design.md` 给出 B1/B2 的状态/输入、预测模型、误差集合、约束判据、备份/终端接口。
- [x] 文档明确指出 B1 与 B2 只差「误差集合表示/新增步骤」这一项。
- [x] 关键外部结论标注 DOI/arXiv 版本，未读原文的结论标「待核验」。

## Task 4：三个机制反例（days 8—10）

- [x] `04_mechanism_counterexamples.md` 含三个反例的初态、控制/误差设置、运行命令与原始产物路径。
- [x] 明确区分「物理事实」与「差模型造成」的结论。
- [x] 说明规则法与标准预测法是否被赋予同等制动能力，并在含制动条件下确认问题仍存在。（`fair_brake_compare.json`，`all_survive=true`）
- [x] `prediction_replay_format_v1.md` 含预测保存字段、输入回放对齐规则与扫掠净空下界判定方法。

## Task 5：第一次研究决策（days 11—14）

- [x] `05_nearest_neighbor_table_v1.md` 至少覆盖 DDSF、Layered Safety、Wabersich–Zeilinger 2021、Kousik RTD、Chen FaSTrack，并按规定句式填写或标「填不出」。（2 行填得出、3 行显式标「填不出」）
- [x] `06_research_decision.md` 含接口选择依据与前 3 个月工作量预算。
- [x] `07_journal_requirement_check.md` 记录访问 URL、日期、结果（含失败状态码）与本地格式文件退路，无未核实的现行效力断言。（当日 HTTP 200，已声明为快照）
- [x] `08_advisor_memo.md` 含「问题 / 证据 / 当前结论 / 下次能否证伪」四项。
- [x] `00_README.md` 含 stage_logs/one 索引与两周结论（失败条件、标准方法解决量、剩余问题、未完成项）。

## Task 6：核验后修复（新增）

- [x] `04` §3.1 残差表 L=0.12/0.40/0.60 已与 `ce3_delay_curve.csv` 对齐，边际表述已更正为 ≈0.65·V·L。
- [x] `02` §4 已拆分为「采集脚本口径」与「trace 复算口径」，并加与 `04`/`ce1` 的交叉说明。

## 全局约束

- [x] 未修改、未删除任何旧源码、旧清单与旧实验结果（`find … -newermt "2026-09-24 15:20"` 为空；`.trae/specs/` 下其余删除项在本次开始前既已存在）。
- [x] 所有仿真以 headless 运行，未启动 Gazebo GUI；核验时无残留仿真进程。