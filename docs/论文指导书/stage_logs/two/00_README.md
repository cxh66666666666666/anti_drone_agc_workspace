# 00 阶段 two 审查入口与索引（N1—N4 更新）

> **任务位置**：本阶段任务表已移入 [`publication_tasks.md`](publication_tasks.md)；当前执行计划在 [`three/第三波任务.md`](../three/第三波任务.md)。

> **14—16 二次结果复审**：见 [`17_followup_review.md`](17_followup_review.md)。当前结论为终止 `P` 投稿主张；`16` §4 重复了 `14` 已完成的备份修正，下一步是有时限的新候选方法检验与选题决定。

> **2026-09-25 独立复审**：N0—N4 交付完成，但 N2/N3 的关键推论存在实现与覆盖问题；R0 证据链暂不通过。先看 [`13_acceptance_review.md`](13_acceptance_review.md) 与 [复算结果](13_review_checks.json)。原 00—12 保留为当时的实验和决策记录。

> 阶段：two（统一执行接口、落实标准预测基线并检验剩余问题）　日期：2026-09-25
> 依据：[第二波任务书](publication_tasks.md) §3、§4；[`publication_plan.md`](../../publication_plan.md) §9.2。
> **用途**：本目录的**审查入口**。先看 [`06_r0_decision.md`](06_r0_decision.md)（N4 结论）与 [`12_p0_p4_summary.md`](12_p0_p4_summary.md)（P0—P4），再按 §2 的 `01`→`12` 顺序阅读，最后按 §3 **随机复算**关键结果。
> **声明**：审查**不以退出码 0 或人工勾选单独认定通过**；不以 `all_survive` 作证据。零违例只写「本批未观测到」。

## 1. 本轮四份新交付（N1—N4，2026-09-25）

| 文档 | 对应任务 | 作用 | 配套代码 / 结果目录 |
|---|---|---|---|
| [`03_identification_validation.md`](03_identification_validation.md) | **N1** | 选定接口补采小规模辨识（14 run）＋独立验证；报告命令速度/制动初速/巡航稳态 | `experiments/paper_v3/identification/brake_hold_v3/`、`identification/identify_v3.py` |
| [`04_fair_comparison.md`](04_fair_comparison.md) | **N2** | 可证伪的公平制动机制对照（同执行器＋同备份） | `experiments/paper_v3/fair_compare/`（`results/summary.json`） |
| [`05_history_value_probe.md`](05_history_value_probe.md) | **N3** | 执行历史细化的独立价值探针（正例/边界负例） | `experiments/paper_v3/history_probe/`（`results/{summary,timing}.json`） |
| [`06_r0_decision.md`](06_r0_decision.md) | **N4** | **R0 复审与下轮任务决定**（三选一结论、最小实验与预算、R1 门槛） | 引用上述三份结果 |

> 另在 [`08_nearest_neighbor_table_v2.md`](08_nearest_neighbor_table_v2.md) **§6** 追加 N3 等价性结论；[`11_advisor_memo_v2.md`](11_advisor_memo_v2.md) 追加 **§5**。

## 2. 按顺序读 `01`…`12`（作用 ＋ 先看什么）

| 文件 | 阶段/任务 | 作用 | 先看什么 |
|---|---|---|---|
| [`01_design_corrections.md`](01_design_corrections.md) | P0＋P2 | 方法标签固化、旧表述纠正、接口结论修订、三反例证据等级更正 | **§1 标签表**（`Legacy/B1/B2/P`）、**§B.4 撤回**（`all_survive` 不可作证据） |
| [`02_standard_baselines.md`](02_standard_baselines.md) | **N0** | 标准 `B1`/`B2` 模型级原型（定高二维位置接口） | **§4 方法差异**、**§7 四场景数值**、§1 证据级别声明 |
| [`03_identification_validation.md`](03_identification_validation.md) | **N1** | 小规模辨识与独立验证 | **§5—§6 速度/延迟**、**§7 验证集误差**、§9 局限 |
| [`04_fair_comparison.md`](04_fair_comparison.md) | **N2** | 公平制动对照（三类控制变量＋正负对照） | **§5—§6 「已解决/剩余」**、§2 参数来源与差异 |
| [`05_history_value_probe.md`](05_history_value_probe.md) | **N3** | 历史细化独立价值探针 | **§4 边界负例**、**§6 等价性判断**、§7 否决依据 |
| [`06_r0_decision.md`](06_r0_decision.md) | **N4** | R0 决策与 R1 门槛 | **§3 三选一结论**、**§1 汇总表**、§4 最小实验与预算 |
| [`07_measurement_v2.md`](07_measurement_v2.md) | P1 | 制动采集与指标口径 v2 | §1 口径对照表、§2 重算示例 |
| [`08_nearest_neighbor_table_v2.md`](08_nearest_neighbor_table_v2.md) | P4 | 最近邻差异表 v2 | **§1 硬性更正**、§4 可区分性、**§6 N3 结论** |
| [`09_research_decision_v2.md`](09_research_decision_v2.md) | P4 | 研究决策节奏复位与预算改对 | §1 决策节奏、§2 预算 84—140 h、§3 接口 |
| [`10_sources_archival.md`](10_sources_archival.md) | P4 | 来源存档清单与 AIGC 政策摘录 | §1 已获取条目、§3 AIGC 原句 |
| [`11_advisor_memo_v2.md`](11_advisor_memo_v2.md) | P4（＋N4 追加） | 导师讨论备忘录 v2 | §1 讨论状态（全「未讨论」）、**§5 N1—N4 纳入与要问的问题** |
| [`12_p0_p4_summary.md`](12_p0_p4_summary.md) | P4 | P0—P4 完成情况与交付物索引 | §1 完成表、§2 交付物索引 |

## 3. 先看什么、再随机复算什么

**先看（自上而下）**：`06` §3 结论 → `06` §1 汇总表 → `12` §1 完成情况 → `04` §6「已解决/剩余」→ `05` §6「等价性判断」。

**再随机复算（确定性，可离线，无需 ROS）**：

```
python3 experiments/paper_v3/baselines/run_baselines.py                     # N0（四场景）
python3 experiments/paper_v3/identification/identify_v3.py \
    --root experiments/paper_v3/identification/brake_hold_v3               # N1（辨识＋验证）
python3 experiments/paper_v3/fair_compare/run_fair_compare.py              # N2（summary.json SHA 稳定）
python3 experiments/paper_v3/history_probe/run_probe.py                    # N3（summary.json SHA 稳定）
```

- **N1 采集**需 ROS/Gazebo（headless）：`python3 experiments/paper_v3/scripts/brake_hold_test_v2.py …`（单 run ~50 s；命令见 [`03`](03_identification_validation.md) §10）。
- 抽查点：`03` §6 的 `T`/延迟区间是否与 `identified_params.json` 一致；`04` §5 的 `by_method` 是否与 `summary.json` 一致；`05` §4 的覆盖违例是否与 `summary.json` 的 `aggregates` 一致。

## 4. 证据级别与纪律（贯通）

- 级别标注：**N1＝`sim-observation`**；**N0/N2/N3＝`model-level`**（合成/借用参数）。**尚无 PX4 闭环/实飞验证**。
- 真值只进**评分侧**；零违例只写「**本批未观测到**」，不写「已证明安全/一般安全」。
- **未引用** `all_survive` / `fair_brake_compare` 作证据；未改 `one/` 冻结数据与 `src/` 生产源码（见各文档「改动范围」）。
- **R0 不由目录齐全自动过关**：见 [`06`](06_r0_decision.md) §5。
