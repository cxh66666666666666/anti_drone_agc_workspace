# 06 R0 复审与下轮任务决定（阶段 two，N4）

> **后续独立复审（2026-09-25）**：本文件保留为当时决策记录；N2 候选执行与证书输入不一致，N3 时变延迟覆盖前提不成立，故 R0 证据链暂不通过。见 [`13_acceptance_review.md`](13_acceptance_review.md) 与 [复算结果](13_review_checks.json)。

> 阶段：two　日期：2026-09-25
> 依据：`.trae/specs/execute-two-n1-n4/spec.md`「N4 R0 复审与下轮决定」、`tasks.md` Task 4；[第二波任务书](publication_tasks.md) §3 N4；[`publication_plan.md`](../../publication_plan.md) §9.2；[`09_research_decision_v2.md`](09_research_decision_v2.md)。
> 方法标签以 [`01_design_corrections.md`](01_design_corrections.md) §1 为准：**`Legacy` / `B1` / `B2` / `P`**。
> **证据级别**：**N1＝`sim-observation`**（Gazebo/PX4 SITL 无头仿真实测，**非实飞**）；**N0/N2/N3＝`model-level`**（合成/借用参数，离线）。**尚无 PX4 闭环/实飞验证**。
> **措辞纪律**：零违例只写「**本批未观测到**」；**不引用** `all_survive` / `fair_brake_compare` 作证据；严格区分 `model-level` 与 `sim-observation`。

---

## 1. 汇总表（完成 / 失败 / 未解决 / 实际耗时；每条链接原始证据）

| 类别 | 项 | 结论 | 原始证据（文件：字段 / 原文） |
|---|---|---|---|
| 完成 | **N0** 标准 `B1`/`B2` 模型级原型（四场景） | 四场景预期均被观测到；判定由独立回放度量推出 | [`02_standard_baselines.md`](02_standard_baselines.md) §7—§8；`experiments/paper_v3/baselines/results/summary.json` 的 `all_expected_observed` |
| 完成 | **N1** 小规模辨识与独立验证（14 run 全 `ok`） | 界面已辨识：`T≈0.300 s`、纯延迟区间 `[0.18,0.30] s` | [`03_identification_validation.md`](03_identification_validation.md) §5—§8；`identification/brake_hold_v3/identified_params.json` 的 `nominal_model.T_s`＝0.2999、`nominal_model.delay_interval_s`＝[0.18,0.3]；`trial_manifest.json` 的 `n_runs`＝12、`runs[].status`＝`ok` |
| 完成 | **N2** 可证伪的公平对照（11 场景 × 5 方法＝55 trial） | 三旧反例均**不需 `P`** 即可被标准手段处理 | [`04_fair_comparison.md`](04_fair_comparison.md) §6；`fair_compare/results/summary.json` 的 `resolved.no_counterexample_requires_P`＝true、`n_trials`＝55、`evidence_level`＝`model-level` |
| 完成 | **N3** 历史细化独立价值探针 | 历史细化**无独立方法贡献** | [`05_history_value_probe.md`](05_history_value_probe.md) §6—§7；`history_probe/results/summary.json` 的 `conclusion.no_independent_method_contribution`＝true、`conclusion.verdict`＝`no_independent_contribution` |
| 完成 | **N4** 本决定 ＋ 阶段入口 ＋ 导师备忘录 | 见 §3；不生成 R1 | 本文件；[`00_README.md`](00_README.md)；[`11_advisor_memo_v2.md`](11_advisor_memo_v2.md) §5 |
| **失败/否决** | **`P`（执行历史细化）无独立贡献** | **否决**（等价于标准集合辨识/多模型鲁棒控制） | `history_probe/results/summary.json` 的 `conclusion.time_varying_pruning_unsound_any`＝true、`sound_requires_all_successor_branches_any`＝true；[`05`](05_history_value_probe.md) §7 |
| 未解决（**标准缺口，非 `P`**） | `B1`/`B2` 共用备份分支乐观（点延迟＋仅判末端） | 待修复＝改延迟区间＋备份段扫掠净空（标准手段） | `fair_compare/results/summary.json` 的 `resolved.B1_B2_backup_branch_optimistic`＝true；[`04`](04_fair_comparison.md) §5—§6 |
| 未解决（保守性） | `rule`/`B2` 误拒 | 安全 vs 通行权衡，非 `P` | `fair_compare/results/summary.json` 的 `by_method.rule.false_reject_trials`＝4、`by_method.B2.false_reject_trials`＝2 |
| 未解决（证据级别） | **无 PX4 闭环/实飞验证** | 缺证（N1 仅 `sim-observation`，N2/N3 仅 `model-level`） | [`03`](03_identification_validation.md) §1、[`04`](04_fair_comparison.md) §1、[`05`](05_history_value_probe.md) §1 |
| 耗时 | N1 **仿真占机** | 12 trial 合计 ≈580 s（45.79—49.47 s/run）＋ 2 烟雾 ≈106 s ≈ **686 s（≈11.4 min）** | `trial_manifest.json` 的 `runs[].wall_seconds`；[`03`](03_identification_validation.md) §4（~50 s/run） |
| 耗时 | N2/N3 **离线计算**（无仿真占机） | N3 单步 ~0.64 ms（`proto_sliding`）vs `B2` ~1.19 ms | `history_probe/results/timing.json` 的 `proto_sliding_ms_per_step`＝0.6399、`B2_baseline_ms_per_step`＝1.1929 |
| 耗时 | 主动投入（人工） | **未逐项仪器化**；按任务书 N0—N4 预算 52—82 h 计（**不虚报**） | [第二波任务书](publication_tasks.md) §4；[`09`](09_research_decision_v2.md) §2 |

> 说明：仿真占机时间已由 `wall_seconds` 记录并可复算；主动投入小时本轮**无仪器化记录**，故不逐项编造，仅引预算口径。

## 2. 决定性证据链（N1 → N2 → N3）

1. **N1（接口已辨识，`sim-observation`）**：在选定接口（直接 PX4 位置设定点台架）上 14 run 全 `ok`；辨识出速度通道一阶滞后 `T≈0.300 s`、**纯延迟只能给区间 `[0.18,0.30] s`**（1.0 档 [0.26,0.30]、0.5 档 [0.18,0.22]）；1.0 档实测 1.075—1.105 m/s 为**未稳态＋超调**的制动初速（巡航稳态 1.004—1.032 m/s，**不改标签**）；验证集多步误差：巡航稳态位置 0.061/0.085 m、制动段速度 0.103/0.114 m/s、overshoot 预测 0.446 vs 实测 0.452 m（[`03`](03_identification_validation.md) §5—§8；`identified_params.json` 的 `speed_split`、`validation`）。→ **接口可辨识、延迟必须按区间**。
2. **N2（旧反例不需 `P`；剩余为标准缺口，`model-level`）**：三项旧反例 **均不需执行历史细化 `P`** 即可被标准手段处理（`no_counterexample_requires_P=true`）——CE2 由 `B2` 方向相关集合解决、CE1/CE3 由规则法（速度/延迟相关扫掠净空）解决；**剩余是为标准缺口的 `B1`/`B2` 共用备份分支乐观**（点 `L_hat`＋只判末端入 Ω），修复＝延迟改区间＋备份段扫掠净空，**不属于 `P`**（[`04`](04_fair_comparison.md) §5—§6；`summary.json` 的 `resolved`）。→ **没有留下必须由 `P` 解决的问题**。
3. **N3（历史细化无独立贡献 → 否决，`model-level`）**：`fixed` 延迟下剪枝能缩小分支（7→约 1.5—1.7 支）且**本批未观测到**覆盖违例，但未超越 `B2` 区间端点处理（后者同样 0 覆盖违例且在预算内）；**允许时变延迟下剪枝不成立**（`missed_true_branch=true`，真值分支被排除 35—39 拍、覆盖违例 79—83 拍），sound 处理＝保留所有后继＝多模型/鲁棒集合，**超 `B2` 预算**；现实噪声下退化为「几乎不剪/不可靠剪」（[`05`](05_history_value_probe.md) §3—§6；`summary.json` 的 `conclusion`）。→ **`no_independent_method_contribution=true`，否决 `P`**。

**链条结论**：接口可辨识（N1）→ **旧反例不需 `P`、剩余为标准缺口**（N2）→ **`P` 无独立贡献**（N3）。即本阶段研究问题的回答是：**标准方法（`B2`/规则法）已解决旧反例；剩余问题不需要执行历史细化。**

## 3. 明确三选一结论

> **选「停止当前具体机制（`P`：执行历史细化）」，并按 [`publication_plan.md`](../../publication_plan.md) §9.2 与导师就「改候选方法 / 改题」做一次正式讨论。**（**不选**「继续当前具体机制」。）

**理由（由已产出证据驱动）**：

1. **「继续」不成立**：`P` 由测量被判**无独立方法贡献**（N3）；且 N2 显示**无需要 `P` 才能解决的剩余问题**（`no_counterexample_requires_P=true`；剩余仅标准缺口）。继续当前机制缺乏证据支持。
2. **「停止 `P`」成立**：N3 的否决判据（`missed_true_branch=true`＋覆盖违例）在两个跳变方向、两种剪枝模式、两档噪声均**方向一致复现**，符合 spec「有可靠否决证据直接提交否决分析」。
3. **不在本轮单方判定「改候选方法」或「改题」**：二者属 §9.2 的分支——§9.2-②「问题有价值，但候选方法无新增内容 → 改候选方法（固定计算预算下的安全过滤计算调度/约束筛选）」；§9.2-③「两候选机制均无差异 → 停止算法主线、改题」。本轮**仅实测 `P` 一个候选机制**，不足以单方宣称 §9.2-③ 前提；且 §9.2-② 的备选问题「尚未成立的第二创新点」。故按 §9.2 将「改候选方法 vs 改题」交导师讨论（要问的具体问题见 [`11`](11_advisor_memo_v2.md) §5）。

## 4. 最小实验与追加预算（因现有证据为 `model-level`，无闭环验证）

- **能改变决策的最小实验**：把 `B1`/`B2` 共用的备份分支由「点延迟 `L_hat` ＋ 仅判末端入 Ω」改为「**延迟区间 `[L_min,L_max]` ＋ 备份段扫掠净空检查**」（标准手段；[`04`](04_fair_comparison.md) §5—§6），在 **1 个可重复工况**做**闭环验证**（PX4 SITL 位置设定点台架；1 方向 × 0.5/1.0 两档，各重复 2—3 次）。
- **判据**：(a) 标准缺口修复**是否成立**（`false_safe` 是否消除、是否新增误拒）；(b) **是否仍存在 `P` 才能解决**的剩余问题。
- **决策映射**：
  - 若 (a) 成立且 (b) **无 `P` 残余** → 支持停止算法主线（§9.2-③ 改题）；
  - 若 (b) 出现可靠 `P` 残余 → 支持 §9.2-② 改候选方法；
  - 二者任一均**不**支持「继续当前 `P`」。
- **追加预算**：**≤ 2 周 / 42—70 h**（含闭环采集仿真占机；为 N0—N4 预算 52—82 h **之外**的追加）。**不默认再拖三个月**（[`09`](09_research_decision_v2.md) §1）。

## 5. R1 门槛声明

- **R1（扩大辨识 / 局部感知隔离）只在结论为「继续」且 R0 条件满足时才生成**；本轮结论为「停止当前机制」，故**本轮不生成 R1 任务**。
- **不因 `two/` 目录齐全、文件可打开、退出码 0 或人工勾选单独认定 R0 通过**；通过与否以本文件与 [`00_README.md`](00_README.md) 所列证据、并按其复算入口为准。

## 6. 与 §9.2 / 导师讨论

- 本决定**不代替**已发生的导师讨论；要问导师的具体问题与「未讨论/已讨论日期与反馈」的真实状态见 [`11_advisor_memo_v2.md`](11_advisor_memo_v2.md) §5。
- **导师回复、学校认定、未明确费用不作为技术阻塞项**（[第二波任务书](publication_tasks.md) §2 P4 验收、[`11`](11_advisor_memo_v2.md) §2）。

## 7. 可复算入口与证据链接

```
# N1 辨识（纯标准库、无随机数、重复运行一致）
python3 experiments/paper_v3/identification/identify_v3.py --root experiments/paper_v3/identification/brake_hold_v3
# N2 公平对照（确定性；summary.json 的 SHA-256 稳定，见 two/04 §7）
python3 experiments/paper_v3/fair_compare/run_fair_compare.py
# N3 历史细化探针（确定性；summary.json 的 SHA-256 稳定，见 two/05 §8）
python3 experiments/paper_v3/history_probe/run_probe.py
```

- 证据链接：N1→`identification/brake_hold_v3/{identified_params.json,trial_manifest.json,summary_v2.json}`；N2→`fair_compare/results/summary.json`；N3→`history_probe/results/{summary.json,timing.json}`；N0→`baselines/results/summary.json`。

## 8. 局限与诚实声明

- **全部结论限定在已列场景/噪声档/接口**；N1 为小样本（每档每方向 3 次）**不构成统计安全边界**；N2/N3 为合成/借用参数的 `model-level`（[`03`](03_identification_validation.md) §9、[`04`](04_fair_comparison.md) §1、[`05`](05_history_value_probe.md) §9）。
- **尚无 PX4 闭环/实飞验证**；本轮不据此宣称「已证明安全」，零违例只写「**本批未观测到**」。
- **未引用** `all_survive` / `fair_brake_compare` 作证据；未改动 `one/` 冻结数据与 `src/` 生产源码（[`03`](03_identification_validation.md) §11、[`04`](04_fair_comparison.md) §7、[`05`](05_history_value_probe.md) §8）。

---

## 修正声明（2026-09-25，依 [`13`](13_acceptance_review.md) 复审）

> 本节为**追加**，不改写上文原结论。独立复审 [`13_acceptance_review.md`](13_acceptance_review.md) 判定 **R0 证据链暂不通过**（N2 候选执行与证书输入不一致、N3 时变覆盖前提不成立）；已完成 F1—F4 离线修正与**二次决策**，**最新决策为** [`16_r0_decision_v2.md`](16_r0_decision_v2.md)。本文为**当时决策记录**。

1. **强表述受限**：§2「决定性证据链」与 §3 的结论**以二次决策为准**；§2/N2 的「三项旧反例均不需 `P`」（`no_counterexample_requires_P=true`）依 [`14`](14_offline_protocol_and_comparison.md) §8 收紧判据后为 `no_counterexample_requires_P_v2=false`；「CE1/CE3 已解决」改述为「`rule` **能识别不可认证工况**」。
2. **§4 的 ≤2 周 SITL 不作为第一步**：应先完成离线修正；SITL 仅在仍有具体机制差异时另行设计（[`16`](16_r0_decision_v2.md) §2、§4）。
3. **指向**：本文 §3 的三选一「停止当前机制」与二次决策方向一致，但二次决策已**正式终止当前 `P` 的投稿主张**；具体证据与最小实验见 [`16`](16_r0_decision_v2.md)。
