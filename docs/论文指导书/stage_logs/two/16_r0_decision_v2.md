# 16 二次 R0 决策 v2（阶段 two，Task 4）

> **后续复审**：[`17_followup_review.md`](17_followup_review.md) 指出本文件 §1/§4 把 `14` 已完成的备份证书修正再次列为下一步，且 `no_counterexample_requires_P_v2` 不能用来判断 `P` 是否有收益。原文保留为当时决策记录；下一步按 `17` 执行。

> 日期：2026-09-25　阶段：two　依据：[`13_acceptance_review.md`](13_acceptance_review.md) §3；`.trae/specs/execute-two-review-corrections/spec.md`「二次 R0 决策（离线修正优先）」；`tasks.md` Task 4；[`publication_plan.md`](../../publication_plan.md) §9.2。
> 前置产物：[`14_offline_protocol_and_comparison.md`](14_offline_protocol_and_comparison.md)（F1/F2）、[`15_time_varying_coverage.md`](15_time_varying_coverage.md)（F3）。
> 证据级别：**`model-level`**（离线）；N1 为 `sim-observation`；**本轮未做 SITL/闭环**。
> 措辞纪律：零违例只写「本批未观测到」；**不引用** `all_survive`/`fair_brake_compare`；**不编造导师认可**。

---

## 1. 证据汇总（完成 / 失败 / 未解决 / 耗时）

| 类别 | 项 | 结论 | 原始证据（文件：字段） |
|---|---|---|---|
| 完成 | **F1** 唯一执行协议 | 每拍重决策、候选 1 拍、转备份持续发布 | `fair_compare/results_v2/summary_v2.json` 的 `protocol_version=fair_compare_v2`；[`14`](14_offline_protocol_and_comparison.md) §2 |
| 完成 | **F1** 证书＝执行（同序列、全分支） | 一致性断言通过 | `summary_v2.json:certificate_input_matches_executed_all_trials=true`；`by_param_set.*.by_method.*.certificate_input_matches_executed_all=true`；`n_trials=140`、`n_scenarios=14` |
| 完成 | **F1** 五类判定（区分可辨） | 五类齐备，全部由测量计算 | `summary_v2.json:five_classes_all_present=true`、`five_classes=[certificate_unavailable,margin_violation,physical_collision,false_reject,pass]`、`all_measured=true`；非互斥 `five_classes_present`、互斥 `verdict_counts_all_trials` |
| 完成 | **F2** 参数版本化与独立验证 | 位置通道模型判定＝**受限** | `fair_compare/results_v2/params_v2.json:f2_position_channel_verification.judgment="受限"`（10 步 0.1904 m > 裕量 0.15 m）；`delay_branch_definition` |
| 完成 | **F2** 位置通道重跑对照（＋合成敏感性） | 14 场景×5 方法×2 参数集＝140 trial | `summary_v2.json:by_param_set.{n1_position_channel,synthetic_sensitivity}.by_method.*`；[`14`](14_offline_protocol_and_comparison.md) §5—§8 |
| 完成 | **F3** 必过样例被 sound 覆盖 | `sound_covers_this_sample=true`；恒定分支/`proto_*` 失败已复现 | `history_probe/results_v2/must_pass_sample.json`（与 `13_review_checks.json:time_varying_delay_counterexample` 逐字段一致） |
| 完成 | **F3** 三类转移规律覆盖 | 见 §1 失败项 | `history_probe/results_v2/summary_v2.json:aggregates`、`conclusion`；[`15`](15_time_varying_coverage.md) §5 |
| **失败/否决** | 仍无「标准强基线解决不了、且新机制有稳定收益」的实例 | **确认无** | `summary_v2.json:resolved.conclusion`、`resolved.no_counterexample_requires_P_v2=false` |
| **失败/否决** | 旧 `proto_*` 剪枝在一般时变延迟**不成立** | **成立否决** | `history_probe/results_v2/summary_v2.json:conclusion.old_proto_pruning_unsound_in_general_time_varying=true`；违例计数见 `aggregates` |
| **失败/否决** | 「端点 2 支覆盖一般时变延迟」 | **撤回**（限原场景） | `summary_v2.json:conclusion.endpoints_cover_general_time_varying=false`、`retained_all_zero_violation_only_in_original_scenarios=true` |
| 未解决（**标准缺口，非 `P`**） | `B1`/`B2` 共用备份分支乐观 | 修复＝延迟改区间 ＋ 备份段扫掠净空 | `summary_v2.json:resolved.mechanism_evidence`（`B_err_toward_wall`：`B1_false_safe=true`、`B2_false_safe=false`、`B2_verdict=pass`）；`resolved.superseded.B1_B2_backup_branch_optimistic` |
| 未解决（模型受限） | 位置通道多步误差超裕量 | 仅可作离线比较 | `params_v2.json:f2_position_channel_verification.judgment="受限"` |
| 未解决（证据级别） | **无 PX4 闭环/实飞验证** | 缺证（本轮离线） | [`14`](14_offline_protocol_and_comparison.md) §10、[`15`](15_time_varying_coverage.md) §8 |
| 耗时 | 本轮均为**离线计算**（无仿真占机） | 140 trial ＋ 覆盖检查 | `summary_v2.json:determinism_note`；`history_probe/results_v2/timing_v2.json`（一次性墙钟，非确定） |
| 耗时 | 主动投入（人工） | **未逐项仪器化**（不虚报） | [第二波任务书](publication_tasks.md) §4 预算口径 |

## 2. 顺序：先完成离线修正；本轮未做 SITL

- 复审 [`13`](13_acceptance_review.md) §3 已明确：**`06_r0_decision.md` §4 的 ≤2 周 SITL 验证此时不宜作为第一步**——尚无一致的离线执行协议，且仅修标准备份并在一两个工况验证，不能直接判断「`P` 才能解决的剩余问题」。
- 本轮执行顺序：**F1/F2 离线证据（Task 1—2）→ F3 时变覆盖（Task 3）→ 二次决策（Task 4）**。
- **本轮未做 SITL/闭环**：SITL 仅在**仍有具体机制差异**时才另行设计；本轮结论为终止，故不启动。

## 3. 明确三选一结论

> **正式终止当前 `P` 的投稿主张**，并按 [`publication_plan.md`](../../publication_plan.md) §9.2 与导师讨论「**改候选方法 vs 改题**」。（**不选**「继续当前具体机制」。）

**理由（由 §1 证据驱动）**：

1. **「继续」不成立**：修正后**仍无**「标准强基线解决不了、且新机制在同覆盖/同预算下有稳定收益」的实例（`resolved.conclusion`；`no_counterexample_requires_P_v2=false`）。
2. **机制证据薄弱且属标准缺口**：可见机制差异**仅剩** `B2` 方向相关鲁棒修复 `B1` 的一处 `false_safe`（`B_err_toward_wall`，`model-level`）；剩余缺口为**标准缺口**（共享备份分支由点延迟 `L_hat` 改区间 ＋ 扫掠净空），**不需要 `P`**。
3. **时变覆盖否决 `P` 候选**：`old_proto_pruning_unsound_in_general_time_varying=true`；sound 处理＝保留所有后继＝标准集合/多模型鲁棒，**超 `B2` 预算**。
4. **不在本轮单方判定 `改候选方法` 或 `改题`**：二者属 §9.2-②/③ 分支；本轮未实测第二个候选机制，不足以单方宣称 §9.2-③ 前提，故交导师讨论（要问的问题见 [`11_advisor_memo_v2.md`](11_advisor_memo_v2.md) §6）。

## 4. 最小实验与追加预算（**先离线**）

- **能改变决策的最小实验（离线）**：把 `B1`/`B2` 共享的**备份分支**由「点延迟 `L_hat` ＋ 仅判末端入 Ω」改为「**延迟区间 `D=[0.18,0.24,0.30]` ＋ 备份段扫掠净空检查**」（标准手段），在 v2 唯一执行协议下重算 `ce1`/`ce3`（`A_clearance_high_speed`、`C_cmd_L_upper`）：
  - 判据 (a)：是否**消除 `false_safe`**（`resolved.by_scenario.*.per_method.*.false_safe`）且**不新增误拒**；
  - 判据 (b)：是否**仍存在 `P` 才能解决**的剩余问题（收紧判据下 `no_counterexample_requires_P_v2` 是否可由任何标准手段翻为 `true`）。
- **必须是「先离线」**：本最小实验**不含** SITL；仅在 (b) 出现**可靠 `P` 残余**时，才另行设计闭环试验。
- **追加预算**：**≤2 周 / 42—70 h**（离线计算为主；为 N0—N4 预算之外）。
- **决策映射**：(a) 成立且 (b) 无 `P` 残余 → 支持 §9.2-③ 改题；若 (b) 出现可靠 `P` 残余 → 支持 §9.2-② 改候选方法；二者任一均**不**支持「继续当前 `P`」。

## 5. 不生成 R1

- **R1（扩大辨识 / 局部感知隔离）只在结论为「继续」且 R0 条件满足时才生成**；本轮结论为终止当前 `P` 投稿主张，故**本轮不生成 R1**。
- **不因目录齐全、文件可打开、退出码 0 或人工勾选单独认定 R0 通过**。

## 6. 措辞纪律与局限

- **零违例只写「本批未观测到」**；全部结论 **`model-level`**（N1 为 `sim-observation`）；真值只进评分侧。
- **模型受限**（位置通道判定＝受限）＋**小样本**（N1 非统计边界）；**本轮未做 SITL/闭环**。
- **未引用** `all_survive`/`fair_brake_compare`；未改动 `one/` 冻结数据与 `src/` 生产源码。

## 7. 可复算入口与证据链接

```
# 在仓库根目录（纯 Python 标准库，无需 ROS）
python3 experiments/paper_v3/fair_compare/run_fair_compare_v2.py     # F1/F2 → fair_compare/results_v2/
python3 experiments/paper_v3/history_probe/run_coverage_v2.py        # F3   → history_probe/results_v2/
```

- 证据链接：F1/F2→`fair_compare/results_v2/{summary_v2.json,params_v2.json}`；F3→`history_probe/results_v2/{summary_v2.json,must_pass_sample.json,timing_v2.json}`。
- 复审基线（只读）：[`13_acceptance_review.md`](13_acceptance_review.md)、[`13_review_checks.json`](13_review_checks.json)。
