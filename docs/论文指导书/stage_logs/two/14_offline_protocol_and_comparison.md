# 14 离线唯一执行协议与对照重跑（阶段 two，F1/F2）

> 日期：2026-09-25　阶段：two　依据：[`13_acceptance_review.md`](13_acceptance_review.md) §2 F1/F2、§3；`.trae/specs/execute-two-review-corrections/spec.md` F1/F2；`tasks.md` Task 1—2。
> 交付物：本文档；v2 代码 `experiments/paper_v3/fair_compare/{closed_loop_v2.py,certificate_v2.py,scenarios_v2.py,run_fair_compare_v2.py}`；v2 结果 [`results_v2/`](../../../../experiments/paper_v3/fair_compare/results_v2/)。
> 证据级别：**`model-level`**（离线；位置通道小样本辨识参数**版本化**输入，非 SITL/真机）。
> 措辞纪律：零违例只写「本批未观测到」；真值仅进评分侧；**未引用** `all_survive`/`fair_brake_compare`。

---

## 1. 修正目标（F1/F2）

- **F1**：固定**唯一执行协议**——每拍重新观测、重新决策；候选执行与证书基于**同一输入序列**；证书须在**全部允许延迟分支**与**方向误差集合**上验证（全时域**扫掠净空** ＋ 终端条件），不再只判 1 拍候选 ＋ 点延迟 ＋ 仅终端。
- **F2**：用**辨识并独立验证**的位置命令模型重跑对照，参数**版本化**；延迟取辨识区间**下界/中点/上界**；原合成参数仅作**敏感性对照**；「已解决」判据收紧。

> 本文只覆盖 F1/F2；F3 见 [`15_time_varying_coverage.md`](15_time_varying_coverage.md)，二次决策见 [`16_r0_decision_v2.md`](16_r0_decision_v2.md)。

## 2. 唯一执行协议（F1）

对任一方法的 trial，逐控制拍 `k`（`n_exec_steps=300`，`dt=0.02 s`）：

1. 用**控制侧可得**状态观测（实测位置 `p_k`、速度 `v_k`、已发布命令历史）**重新决策**；
2. **只发布本拍选定命令（1 拍）**；
3. 经**真值闭环**（注入真值延迟 `L_true` 与真值扰动 `e_true`）推进 1 拍；
4. 重复至 300 拍。

- 若本拍判为**转备份/不可认证**，则**持续发布共同备份保持命令**（不再从候选重启）。
- **候选每拍重建**；**禁止**把「候选一拍 ＋ 备份的预测」与「持续发布候选 300 拍」混用（这是 v1 的口径缺陷，见 §9）。
- 真值 `L_true`/`e_true` 只在执行/评分侧，控制决策不可读。

## 3. 全分支证书（F1）

- 证书在与执行**相同的输入序列**上评估。
- 覆盖**全部允许延迟分支**（参数集 `n1_position_channel`：`D_branches=[0.18,0.24,0.30] s`；合成对照集：`[0.15,0.20,0.25] s`）与**方向误差集合**（`σ_E(−∇h)`：`a_long=0.20 m`、`a_lat=0.05 m`）。
- 逐分支做**全时域扫掠净空**检查（`margin=0.15 m`）＋ **终端条件**；点延迟点估计、只判末端（无扫掠净空）的旧证书**不再**作为 `B1`/`B2` 通过依据。

## 4. F1 一致性断言

- `results_v2/summary_v2.json` 的 `certificate_input_matches_executed_all_trials=true`；且 `by_param_set.*.by_method.*.certificate_input_matches_executed_all=true`（逐方法）。
- 规模：`n_trials=140`、`n_scenarios=14`（2 参数集 × 14 场景 × 5 方法）。
- 因此本批 `false_safe` 的归因**不再混入**「候选持续发布」差异。
- **断言语义（如实说明）**：`certificate_input_matches_executed` 比较的是「**证书实际评估展开序列**的首元素（`B1`/`B2` 取最坏延迟分支 rollout 的 `seq[0]`＝候选命令或本拍实测 `p_k`；`rule`/负对照取判据选定命令）」与「**执行器实际发布命令**」逐拍是否一致（来源记于 `closed_loop.certificate_sequence_source`），并非对同一变量回显；`closed_loop.first_certificate_input_mismatch_index` 记录首个不一致下标（本批全为 `null`）。该断言**可检出**改写：`summary_v2.json:certificate_assertion.self_test` 临时改写证书展开序列 → 断言翻为 `false`（首个不一致下标 `0`）、恢复后回到 `true`；另含「单点不一致（下标 `7`）→ `false`」的单元检查。

## 5. F2 位置通道可用性判断与多步误差

`results_v2/params_v2.json: f2_position_channel_verification`：

- `channel_discipline`：**速度通道** `T=0.2999 s` 与**位置通道** `pos_lag_along=1.1732 s` 是**不同模型量，不得互换**；位置命令回放用**位置通道**。
- `judgment = "受限"`；`criterion`：10 步（0.2 s）多步位置误差 max ≤0.05 m → 可用；≤0.30 m → 受限；>0.30 m → 不可用（与设计裕量 `0.15 m` 对照）。
- 实测多步位置误差（`aggregate_max`，跨 4 个独立验证 run 的最大值）：
  | 时域 | 位置通道 `T=1.1732` | 速度通道 `T=0.2999`（仅对照） |
  |---|---|---|
  | 1 步 | 0.0333 m | 0.0628 m |
  | 5 步 | 0.1048 m | 0.2026 m |
  | **10 步** | **0.1904 m**（> 裕量 0.15 m） | **0.3464 m** |
  | 20 步 | 0.3061 m | 0.5190 m |
  | 50 步 | 0.4505 m | 0.6689 m |
- `reason`：位置通道首过模型**短时域**误差可控（1 步 0.0333、5 步 0.1048），但 **10 步 0.1904 m 已超过设计裕量 0.15 m**，仅可用于**离线协议/一致性比较与量级判据**，**不能**充当安全边界；速度通道在本回放下误差更大（10 步 0.3464 m），故**不得换通道**。

## 6. 参数版本化与延迟取值（F2）

`params_v2.json`：

- `delay_branch_definition`：`D_branches=[0.18,0.24,0.30]`、`identified_interval_s=[0.18,0.30]`、`truth_delay_upper_s=0.30`，`check=「真值延迟上界 0.30 s ≥ 辨识区间上界 0.30 s」`（来源 `identification/brake_hold_v3/identified_params.json`）。
- `param_sets.n1_position_channel`（**主选定**）：`T=1.1732`（位置通道 `nominal_model.pos_lag_along_s`）、`L_hat=0.24`、`L_interval=[0.18,0.30]`、`L_true_default=0.3`、`dt=0.02`、`margin=0.15`、`n_exec=300`、`speed_channel_T_s=0.2999`（仅比较）。
- `param_sets.synthetic_sensitivity`（**敏感性对照**）：`T=0.85`、`L_interval=[0.15,0.25]`、`L_true_default=0.25`（原合成/借用参数）。
- **版本化**：来源、区间与验证结论均写入 `params_v2.json`；保留 N1 小样本「**非统计边界**」声明。
- `generated_at` 为唯一时间字段，其余确定性可复算。

## 7. 五类判定（口径说明）

五类为**非互斥的测量分类**（同一 trial 可属多类）；另有**互斥主判定** `verdict`。被拒候选分叉标 `model-level`。

`summary_v2.json`：

- `five_classes_all_present=true`；`five_classes=[certificate_unavailable, margin_violation, physical_collision, false_reject, pass]`。
- **非互斥**计数 `five_classes_present`：`certificate_unavailable=51`、`false_reject=20`、`margin_violation=56`、`pass=84`、`physical_collision=20`（合计 >140，因一个 trial 可同时命中多类，如 `certificate_unavailable` 且 `margin_violation`）。
- **互斥**主判定 `verdict_counts_all_trials`：`certificate_unavailable=14`、`margin_violation=36`、`physical_collision=20`、`pass=70`、`false_reject=0`（合计 =140）。
- 全部**由测量计算**（`all_measured=true`）；判定区分：`certificate_unavailable`（无证书/不可认证）、**裕量违例**、**物理碰撞**、**误拒**、**通行**。

## 8. 收紧后的 `resolved`（F2 SubTask 2.4）

`summary_v2.json: resolved`：

- `criterion`：(i) 无 `false_safe` **且** (ii) 在**可行工况取得任务进展**（`net_progress ≥ 0.05 m`，通行）才算「解决」；不可行工况只计「**不可认证**」，**不得**记为已解决。
- `by_scenario`：`ce2_direction_blind`（`B_err_parallel_wall`）`handled_by=[B1,B2]`（`feasible_observed=true`）；`ce1_speed_blind`、`ce3_delay_blind` 的 `handled_by=[]`，`uncertifiable_by=[rule,B1,B2]`（`feasible_observed=false`）。
- `mechanism_evidence`（`B_err_toward_wall`，`model-level`）：`B1_false_safe=true`、`B2_false_safe=false`、`B2_verdict=pass` → 可见机制差异**仅限 `B2` 方向相关鲁棒修复 `B1` 的一处 `false_safe`**。
- `no_counterexample_requires_P_v2=false`：修正后**仍无**「标准强基线解决不了、且新机制在同覆盖/同预算下有稳定收益」的实例。
- `superseded`：旧 `no_counterexample_requires_P`（`old_value=true`）、`B1_B2_backup_branch_optimistic`（`old_value=true`）均标 `superseded=true`；原字段**保留**供追溯。

## 9. 与 v1 的差异

| 项 | v1（`results/`，历史探索材料） | v2（`results_v2/`） |
|---|---|---|
| 候选执行 | 候选被接受后**持续发布 300 拍**（`candidate_replay_steps=300`） | **每拍 1 拍候选 ＋ 重决策** |
| 候选回放 trial | 55 trial 中 `n_candidate_replays=32`、`n_candidate_false_safe=13` | 上述口径**不再成立** |
| 证书 | 点延迟 ＋ 仅终端 | **全分支扫掠净空 ＋ 终端 ＋ 方向鲁棒** |
| 参数 | 应用 N0 声明常量（`T_lag=0.85`） | `n1_position_channel`（`T=1.1732`）＋ 合成敏感性对照 |
| `false_safe` 归因 | 混合「单拍候选＋备份预测」与「持续候选执行」 | 证书与执行同序列，归因清晰 |

> v1 的 13 个候选 `false_safe` 与相关 trial 的**通行/误拒**，**不得**归因于 `B1`/`B2` 证书；旧结果为**历史探索材料**，结论以 `14`/`results_v2` 为准。

## 10. 局限与诚实声明

- **模型受限**（`f2_position_channel_verification.judgment=受限`）：位置通道 10 步误差 0.1904 m > 裕量 0.15 m，仅作离线比较，**不作安全边界**。
- **小样本**：N1 为小样本 SITL 观测，**不构成统计安全边界**。
- 全部结论 **`model-level`**；本轮**未做 SITL/闭环验证**。
- 零违例仅写「**本批未观测到**」；**未引用** `all_survive`/`fair_brake_compare`。

## 11. 运行与可复算

```
# 在仓库根目录（纯 Python 标准库，无需 ROS）
python3 experiments/paper_v3/fair_compare/run_fair_compare_v2.py
```

- 输出：`results_v2/<sid>__<method>__<param_set>.json` 与 `results_v2/summary_v2.json`、`results_v2/params_v2.json`。
- **确定性**：无随机数；唯一时间字段 `params_v2.json:generated_at`；重复运行除该字段外一致（见 `summary_v2.json:determinism_note`）。
- **改动范围**：仅新增 v2 代码与 `results_v2/`、本文档；**未修改** v1 结果、`baselines/`、`one/`、`src/`、`scripts/`、N1 产物。