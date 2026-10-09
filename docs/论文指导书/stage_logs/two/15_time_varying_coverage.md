# 15 时变延迟覆盖（按允许转移规律）（阶段 two，F3）

> 日期：2026-09-25　阶段：two　依据：[`13_acceptance_review.md`](13_acceptance_review.md) §2 F3、§3；`.trae/specs/execute-two-review-corrections/spec.md` F3；`tasks.md` Task 3。
> 交付物：本文档；v2 代码 `experiments/paper_v3/history_probe/{coverage_v2.py,run_coverage_v2.py}`；v2 结果 [`results_v2/`](../../../../experiments/paper_v3/history_probe/results_v2/)。
> 证据级别：**`model-level`**（离线，合成/借用 + N1 辨识延迟区间）。
> 措辞纪律：零违例只写「本批未观测到」；**未引用** `all_survive`/`fair_brake_compare`。

---

## 1. 修正目标（F3）

复审 §2 F3 指出：`probe.py` 的 `proto_all`/`B2_baseline` 展开的 7 支或 2 支是**整个预测时域延迟恒定**的轨迹，两个时变场景只在第 160 拍跳变一次；其零违例**不能**推出逐拍可变时的可靠性。本项按**显式的延迟允许转移规律**重做覆盖检查，逐拍可变时改用**逐拍所有后继可达集传播**，并以复审反例为**必过样例**。

## 2. 显式允许转移规律

`results_v2/summary_v2.json: transition_laws`：

- **`fixed`**：整个预测时域内延迟取值恒定；真值取自允许集合中的一个值（`L=0.18/0.24/0.30 s`）。
- **`single_jump`**：整个时域内允许**一次**跳变；跳变时刻与跳变前后取值遍历允许集合的若干组合（如拍 160 `0.30→0.18`、拍 100 `0.18→0.24`、拍 120 `0.24→0.30`）。
- **`per_step_arbitrary`**：允许**逐拍任意**取允许集合中的延迟（对抗性）；控制侧**必须**按逐拍所有后继传播。

允许延迟集合：`allowed_lags_steps=[9,10,11,12,13,14,15]`（`branches_s=[0.18,0.20,…,0.30]`，`n_branches=7`）；来源 N1 辨识 `delay_interval` `L_min=0.18`、`L_max=0.30`。

## 3. 逐拍所有后继可达集传播；为何恒定分支包络不成立

- **sound 方案 `sound_reachable`**：**逐拍**对**所有可能后继**传播形成**可达集合（保守外包）**，而非只展开恒定延迟整轨迹。
- **恒定分支包络不成立**：一个从端点到端点都取值恒定的分支集合，**不能**未经证明充当逐拍可变时延的安全基线——真值可在拍间切换，落进「所有恒定分支的并集」之外。
  - `must_pass_sample.json`：`constant_branches_fail_this_sample=true`，`constant_endpoints_2_max_x_m=-0.488584849`（端点 2 支）、`constant_branch_max_x_m=-0.488584849`（全 7 支），**均未覆盖**真值。

## 4. 必过样例（复算数值与 sound 覆盖）

`results_v2/must_pass_sample.json`（与 [`13_review_checks.json`](13_review_checks.json) 的 `time_varying_delay_counterexample` **逐字段一致**）：

- `step=60`，`horizon_steps=50`；在 50 拍内**逐拍选取允许延迟**（前 20 拍 `chosen_delay_steps_first_20=[9,9,…,10,11,12,13,14,15,15,…]`，`chosen_delay_steps_unique=[9,10,11,12,13,14,15]`）。
- `switching_x_m=-0.483507547`（真值）；`constant_branch_max_x_m=-0.488584849`（全恒定分支最大 x）。
- `excess_over_all_constant_branches_m=0.005077301` > `low_noise_coverage_tolerance_m=0.003`；`outside_all_constant_branches_even_with_tolerance=true`。
- `scheme_results`：
  | 方案 | `covers` | `first_violation_h` | 末端 x_max |
  |---|---|---|---|
  | `sound_reachable` | **true** | null | −0.482707547 |
  | `constant_all_7` / `constant_endpoints_2` | false | 16 | −0.488584849 |
  | `proto_sliding` / `proto_commit` | false | 6 | −0.499405469 |
- `sound_covers_this_sample=true`：sound 方案**覆盖**该必过样例（本批未观测到违例）；恒定分支与旧 `proto_*` 在样例上**失败**（已复现）。

## 5. 三类规律的违例计数

`results_v2/summary_v2.json: aggregates` 的 `coverage_violation_steps`，两噪声档（`LOW_NOISE`+`N1_NOISE`）求和：

| 规律 | `sound_reachable` | `proto_sliding` | `proto_commit` | `constant_endpoints_2` | `constant_all_7` |
|---|---:|---:|---:|---:|---:|
| `fixed` | 0 | 0 | 0 | 0 | 0 |
| `single_jump` | **0** | 291 | 365 | 9 | 8 |
| `per_step_arbitrary` | **0** | 462 | 771 | 316 | 314 |

- **`fixed`**：各方案**本批未观测到**覆盖违例。
- **`single_jump`/`per_step_arbitrary`**：`sound_reachable` **本批未观测到**违例；`proto_sliding`/`proto_commit` 及恒定端点/全分支方案均出现违例，且 `per_step_arbitrary` 更严重。
- `conclusion`：`sound_reachable_covers_general_time_varying=true`、`old_proto_pruning_unsound_in_general_time_varying=true`、`endpoints_cover_general_time_varying=false`、`retained_all_zero_violation_only_in_original_scenarios=true`。
- 说明：`constant_endpoints_2`/`constant_all_7` 的违例为**覆盖率**（真值落在包络之外）；`proto_*` 另有 `missed_true_branch=true`（真值分支被排除），均记在 `aggregates`。

## 6. 同预算计时与代码路径差异

`results_v2/timing_v2.json`（一次性墙钟，**非确定性**；同场景 `ps_adv_max_x`、同时域 `horizon_steps=50`、同拍区间 `n_steps=210`、`LOW_NOISE`）：

| 方案 | ms/step |
|---|---:|
| `sound_reachable` | 2.5964 |
| `constant_all_7` | 2.4185 |
| `constant_endpoints_2` | 1.1439 |
| `proto_sliding` | 0.3432 |
| `proto_commit` | 0.3482 |

> 上表为**一次运行的单次观测值**（墙钟，**非确定性**，与 `timing_v2.json` 的当前内容一致）；重复运行会有小幅变化，**这些数值不参与任何结论**，仅用于说明路径与量级。

- `code_path_diff`：`history_probe_v2` 中 `sound`=逐拍区间可达集传播；`proto_*`=滑窗残差筛选 ＋ 分支点集展开——**代码路径不同**，**不得据一次计时判定效率**。
- 确定性预算口径见 `summary_v2.json` 的 `budget_branch_rollouts_per_step_mean`（如 `sound_reachable`/`constant_all_7` 为 980、`constant_endpoints_2` 为 280、`proto_commit` LOW 档 140）。
- **另一路径**：`fair_compare_v2`（闭环逐拍重决策 ＋ 全分支扫掠净空证书）与 `history_probe_v2` 代码路径不同，**未在此计时**。

## 7. 撤回/限定旧结论

依复审 §2 F3、§3，**撤回或限定**以下旧结论（`conclusion.superseded_fields`，原字段保留标 `superseded`），并指向本文件与 [`13`](13_acceptance_review.md)：

| 旧结论 | 位置 | 处置 | 理由 |
|---|---|---|---|
| `B2_baseline_covers_within_budget=true` | `history_probe/results/summary.json:conclusion` | **撤回**（限原场景） | 端点 2 支仅在原恒定/单跳场景零违例；逐拍任意时延下会漏（见必过样例） |
| `sound_requires_all_successor_branches(_any)=true` | 同上 | **改述** | 改为逐拍所有后继的区间可达集传播（保守外包），不再以恒定分支包络充当基线 |
| `no_independent_method_contribution`/`verdict` | 同上 | **限定** | 其「可靠备选」前提（端点 2 支覆盖一般时变延迟）不成立，不能在一般时变延迟下由该布尔值证明等价性 |

- 具体限定：**「端点 2 支覆盖一般时变延迟」「全保留 0 违例」「无独立贡献已由测量证明」** 仅**限于原测试场景**（`fixed`/单次跳变的原构造）；`13`/`15` 已给出一般时变延迟反例。
- 旧 `proto_*` 在必过样例上的失败**已被复现并记录**（`must_pass_sample.json`）。

## 8. 局限与诚实声明

- 全部为 **`model-level`**（合成/借用 + N1 辨识延迟区间）；本轮**未做 SITL**。
- 零违例仅写「**本批未观测到**」；结论限于本文列出的规律、场景与噪声档。
- **未引用** `all_survive`/`fair_brake_compare` 作证据。

## 9. 运行与可复算

```
# 在仓库根目录（纯 Python 标准库，无需 ROS）
python3 experiments/paper_v3/history_probe/run_coverage_v2.py
```

- 输出：逐 `(规律×场景×方案×噪声档)` 写 `results_v2/<regime>__<sid>__<scheme>__<noise>.json`；汇总写 `results_v2/summary_v2.json`；必过样例写 `results_v2/must_pass_sample.json`；计时写 `results_v2/timing_v2.json`（含墙钟，非确定）。
- **确定性**：`summary_v2.json` 不含墙钟；重复运行应字节一致（见 `determinism_note`）。
- **改动范围**：仅新增 v2 代码与 `results_v2/`、本文档；**未修改** v1 结果、`baselines/`、`one/`、`src/`、`scripts/`。