# 04 可证伪的公平制动机制对照（阶段 two，N2）

> 日期：2026-09-25　阶段：two　依据：[第二波任务书](publication_tasks.md) §3 **N2**；`spec.md`「N2 可证伪的公平制动机制对照」；`tasks.md` Task 2。
> 交付物：本文档；配套代码 `experiments/paper_v3/fair_compare/`（`common.py`／`scenarios.py`／`run_fair_compare.py`／`results/`）。
> 方法标签一律以 [01_design_corrections.md](01_design_corrections.md) §1 为准：**`Legacy` / `B1` / `B2` / `P`**。
> 口径：[execution_contract_v2.md](../../../../experiments/paper_v3/protocols/execution_contract_v2.md) §3/§4、[prediction_replay_format_v2.md](../../../../experiments/paper_v3/protocols/prediction_replay_format_v2.md) §1/§2/§4/§5、[02_standard_baselines.md](02_standard_baselines.md)。
> 关联：本次对照是对 `P2` 撤回的 [`fair_brake_compare`](../../../../experiments/paper_v3/mechanisms/results/FAIR_BRAKE_COMPARE_RETRACTION.md) 的**可证伪重做**。

---

## 1. 目的与证据级别

- **目的**：在同一可辨识执行系统内，给规则法与标准预测法（`B1`/`B2`）**相同的制动能力与观测信息**，用**同一执行器与同一备份函数**做控制变量对照，回答本阶段唯一研究问题：**`B2` 能解决哪些问题？剩余问题是否真的需要执行历史相关的细化（`P`），而不是旧模型错误造成？**
- **证据级别**：本文全部结论为 **`model-level`**（离线模型级），参数为**合成/借用**值（`baselines/model.py` 的 `UNIT_TABLE`），**非 PX4/SITL/真机实测**。
- **判定由测量导出**：任何 `passed`/`false_safe`/`false_reject` 一律由**评分侧独立回放**（注入真值延迟 `L_true` 与真值扰动 `e_true`）的度量计算，**无硬编码标签**（`summary.json` 的 `all_measured=true`）。
- **与旧汇总的关系**：**不引用** 已撤回的 `all_survive` / `fair_brake_compare` 作证据；本对照独立重算，可复算。

## 2. 统一执行器、共同备份与共享约束

三方法（**规则法 `rule`**、`B1`、`B2`）与两个负对照（`naive_distance`、`naive_instant`）**全部**经过：

- **同一执行器**：`common.execute_and_replay` —— 复用 `baselines/model.py` 的定高二维位置命令闭环（`dt=0.02 s`、一阶滞后 `α`、纯延迟 `n_L=ceil(L/dt)`），逐拍推演已执行命令序列；**真值延迟 `L_true` 与真值扰动 `e_true` 只在执行/评分侧**（`truth_*` 前缀），控制决策不可读。
- **同一备份函数**：复用 `baselines/backup.py` 的共同备份语义 —— 目标 = **触发拍实测水平位置 `p_k`**、此后持续发布保持设定点。`B1`/`B2` 直接调用 `baselines/filters.py` 的 `decide("B1"/"B2", …)`；规则法与负对照在**同一执行器/同一备份目标**下独立实现决策。
- **共享约束**（`SHARED`，显式写出、各方法不得越权）：`dt=0.02 s`；决策时域 `N=140`（2.8 s）；执行/评分拍数 `300`（6.0 s）；速度上限 `1.0 m/s`；单周期位移限幅 `0.25 m`；`margin=0.15 m`；`v_stop=0.05 m/s`；传感权限 = 仅 `p_k`、`v_k`（位置差分）与已发布命令历史。

**参数来源**（`param_source`）：N1 辨识任务（并行）**已产出** `experiments/paper_v3/identification/brake_hold_v3/identified_params.json`，故 `param_source` 记为 **`N1-identified`**；其**速度通道**识别为 `T≈0.30 s`、`L∈[0.18,0.30] s`（`evidence_level=sim-observation`）。
但公平对照要求三方法共用**同一**原型，而 `B1`/`B2` 必须直接调用**冻结**的 `baselines/filters.py`（其声明常量取自 `baselines/model.py`，只读不可改）；把 N1 的速度通道参数替换进冻结基线超出 N2 范围。因此本批**可运行对照使用 N0 声明常量**（`L_hat=0.20 s`、`L_range=[0.15,0.25] s`、`T_lag=0.85 s`、`a_long=0.20 m`、`a_lat=0.05 m`），并在 JSON 的 `params_snapshot.{param_source,n1_identified,applied_in_run}` 中同时标明「来源=N1-identified／本批应用=N0-declared」，供基线版本化后复算。

**三方法的关键差异（诚实声明）**：

| 方法 | 制动/误差处理 | 延迟建模 |
|---|---|---|
| `rule`（规则法） | **速度相关制动外扩** `d_brake(v)=v·T·(1−exp(−L̂/T))`（随速度单调增，**非**固定常量阈值）＋**方向盲标量**膨胀 `a_long` | 点 `L̂` |
| `B1` | 候选显式展开逐点净空 ＋ 共同备份；误差集合退化 `E={0}` | 点 `L̂` |
| `B2` | `B2` 结构（延迟增广 + 方向相关支持函数 `σ_E`）；**候选分支**用延迟区间 `[L_min,L_max]` 逐分支取最坏 | 候选分支用区间；**备份分支沿用点 `L̂`**（见 §5） |

负对照 `naive_distance`（固定距离阈值、速度盲）与 `naive_instant`（瞬时实现、延迟盲）**仅作反例参照**（对应旧 CE1/CE3 的差模型），与三方法共用同一执行器与备份，**不作为公平基线**。

## 3. 三类控制变量场景（实测数值）

每个场景显式记录初态、命令、真值（`truth_*`）与控制侧可见信息；逐 trial JSON 见 `results/<sid>__<method>.json`。真值注入延迟取声明区间上界 `L_true=0.25 s`（C 类另设下界对照）。

### A 同净空不同速度（同一净空 0.30 m、同一障碍位置）

| 场景 | 方法 | 证书/动作 | 最小净空 (m) | 裕量违例 | 净前进 (m) | 判定 |
|---|---|---|---|---|---|---|
| A 低 0.5 m/s | `rule` | `unavailable`/无 | 0.1867 | 0 | 0.000 | 误拒（本批该候选其实安全） |
| A 低 0.5 m/s | `B1` | valid/候选 | 0.1867 | 0 | 0.080 | 通行 pass |
| A 低 0.5 m/s | `B2` | valid/备份 | 0.1867 | 0 | 0.000 | 误拒（保守，本批未观测到违例） |
| A 低 0.5 m/s | `naive_distance` | valid/候选 | 0.1867 | 0 | 0.080 | 通行 pass |
| A 高 1.0 m/s | `rule` | `unavailable`/无 | 0.0734 | 22 | 0.000 | 诚实不可用（未伪造证书） |
| A 高 1.0 m/s | `B1` | **valid/备份** | 0.0734 | **22** | 0.000 | **`false_safe`（伪造安全证书）** |
| A 高 1.0 m/s | `B2` | **valid/备份** | 0.0734 | **22** | 0.000 | **`false_safe`** |
| A 高 1.0 m/s | `naive_distance` | **valid/候选** | 0.0734 | **36** | 0.080 | **`false_safe`** |

- **可证伪点**：速度盲的 `naive_distance` 在 0.5/1.0 m/s 给出**相同决策**（都放行候选），在 1.0 m/s 实测 36 次裕量违例 → 被本场景捕获；速度相关判据在高低速给出**不同决策**。
- **实测结论**：`B1`/`B2` 在 1.0 m/s 选择共同备份，但其**备份证书**在真值延迟上界下出现 22 次裕量违例 → 记为 `false_safe`（详见 §5）；规则法因速度相关制动外扩而**拒绝伪造证书**（`unavailable`，本批仍观测到 22 次违例，属不可行工况的诚实报告）。

### B 同误差范围不同墙面方向（同一注入误差 `e=(0.20, 0)`）

| 场景 | 方法 | σ_E(−∇h) (m) | 证书/动作 | 最小净空 (m) | 裕量违例 | 净前进 (m) | 判定 |
|---|---|---|---|---|---|---|---|
| B 误差朝墙 | `B1` | — | **valid/候选** | 0.1005 | **201** | 0.500 | **`false_safe`** |
| B 误差朝墙 | `B2` | 0.20 | valid/备份 | 0.4001 | 0 | 0.200 | pass（本批未观测到违例） |
| B 误差朝墙 | `rule` | 0.20（标量） | valid/备份 | 0.4001 | 0 | 0.200 | pass |
| B 误差平行墙 | `rule` | 0.20（标量，方向盲） | `unavailable`/无 | 0.3000 | 0 | — | **误拒**（被拒候选本可安全通行） |
| B 误差平行墙 | `B1` | — | valid/候选 | 0.3000 | 0 | 0.300 | pass |
| B 误差平行墙 | `B2` | **0.05（方向相关）** | valid/候选 | 0.3000 | 0 | 0.300 | pass |

- **可证伪点**：同一误差集合、同一 `|e|=0.20 m`，仅改变墙面法向：`σ_E(−∇h)` 从 `a_long=0.20` 变为 `a_lat=0.05`。
  - **朝墙**：`B1`（`E={0}`）忽略方向误差 → 放行候选 → 实测 201 次裕量违例 → **`false_safe`**；`B2`（方向相关 + 区间）拒绝该候选、执行共同备份 → 本批未观测到违例。
  - **平行墙**：方向盲标量（`rule`，膨胀 0.20）**误拒**一个本可安全通行的候选（该被拒候选经**模型级反事实回放**判定 `would_pass`）；`B2` 用 `σ_E=0.05` 正确放行。
- **实测结论**：误差**方向**确实改变判定；**`B2` 的方向相关处理是本批唯一在「朝墙/平行墙」两种情形都正确的标准方法**（`B1` 朝墙 `false_safe`、方向盲标量平行墙误拒）。

### C 同物理位置命令但不同允许延迟（同命令、同初态）

| 场景 | 方法 | 证书/动作 | 最小净空 (m) | 裕量违例 | 判定 |
|---|---|---|---|---|---|
| C `L_true=0.15 s` | `rule` | `unavailable`/无 | 0.2024 | 0 | 误拒（保守） |
| C `L_true=0.15 s` | `B1` | valid/候选 | 0.2024 | 0 | pass |
| C `L_true=0.15 s` | `B2` | valid/备份 | 0.2024 | 0 | 误拒（保守） |
| C `L_true=0.25 s` | `rule` | `unavailable`/无 | 0.1234 | 7 | 诚实不可用（未伪造证书） |
| C `L_true=0.25 s` | `B1` | **valid/候选** | 0.1234 | **10** | **`false_safe`** |
| C `L_true=0.25 s` | `B2` | **valid/备份** | 0.1234 | **7** | **`false_safe`** |
| C `L_true=0.25 s` | `naive_instant` | **valid/候选** | 0.1234 | **10** | **`false_safe`** |

- **可证伪点**：延迟盲的 `naive_instant` 与点延迟的 `B1` 在 `L_true=0.15/0.25 s` 给出**相同决策**（都放行候选），在 `L_true=0.25 s` 实测裕量违例 → 被捕获。
- **实测结论**：`B2` 的**候选分支**用区间取最坏，但在本场景仍因**备份分支**的乐观证书记为 `false_safe`（§5）；规则法的速度/延迟相关扫掠检查未伪造证书。

## 4. 正/负/已解决对照（实测）

| 对照 | 场景 | 实测 |
|---|---|---|
| **双方均安全** | 开阔（净空 ≈100 m） | `rule`/`B1`/`B2`/`naive_*` 均放行候选，最小净空 **99.7002 m**、0 违例、净前进 0.300 m → 三方法均 `pass` |
| **双方均不可行** | 净空 0.10 m ＋ 1.0 m/s 初态 | 三方法与负对照均 `certificate_unavailable`；回放实测 **42 次物理违例、301 次裕量违例**、最小净空 **−0.1266 m** → **不可用 ≠ 安全**，违例如实报告 |
| **标准方法已解决反例 CE1（速度）** | 同 A 高 | 差模型 `naive_distance` `false_safe`（36 违例）；`rule` 未伪造证书；**`B1`/`B2` 仍 `false_safe`** |
| **标准方法已解决反例 CE2（方向）** | 同 B 平行墙 | 方向盲标量 `rule` **误拒**；`B2` 正确放行 |
| **标准方法已解决反例 CE3（延迟）** | 同 C `L_true=0.25` | `naive_instant`/`B1` `false_safe`；`rule` 未伪造证书；**`B2` 仍 `false_safe`**（备份分支） |

- 五个负对照场景（CE1/CE3 反例）的 `expected` 校验：`summary.json` 的 `expected_checks_observed=13/13`（仅校验，不参与判定）。

## 5. 逐方法结论（由测量）

汇总（11 trial/方法，`summary.json` 的 `by_method`）：

| 方法 | `false_safe` | 裕量违例 trial | 物理违例 trial | 通行 trial | `unavailable` trial | 误拒 trial |
|---|---|---|---|---|---|---|
| `rule` | **0** | 5 | 1 | 4 | 9 | **4** |
| `B1` | **5** | 6 | 1 | 5 | 1 | 0 |
| `B2` | **4** | 5 | 1 | 4 | 1 | 2 |
| `naive_distance` | 5 | 6 | 1 | 5 | 1 | 0 |
| `naive_instant` | 5 | 6 | 1 | 5 | 1 | 0 |

- **`rule`（规则法，速度相关制动距离，方向盲标量）**：本批**从未伪造安全证书**（`false_safe=0`），代价是**过度保守**（4 次误拒、9 次 `unavailable`；含对本来安全候选的拒绝）。它能捕获高速/大延迟下备份不可行（诚实报 `unavailable`），但**方向盲**使其在平行墙误拒（CE2 未解决）。
- **`B1`（点延迟、`E={0}`）**：本批 `false_safe=5`（A 高、B 朝墙、C 上界、两个 CE 反例），**既不处理方向也不处理延迟**，是三者中伪造证书最多的标准方法。
- **`B2`（延迟增广 + 方向相关）**：**CE2（方向）唯一在朝墙/平行墙两种情形都正确**（朝墙拒绝→安全备份；平行墙 `σ_E=0.05` 正确放行）。但实测暴露一个**结构性缺口**：`B2` 的**候选分支**用延迟区间取最坏，而**备份分支**仍调用 `backup_reaches_terminal(L_hat)`（**点延迟 + 仅判终端状态**，不检查备份途中的扫掠净空与延迟上界）→ 在 A 高 / C 上界等**备份本身不可行**的工况记 `false_safe=4`。这是 `B1`/`B2` **共用的备份分支证书**的乐观性，可由「改用 `L_max` 且检查扫掠净空」的标准手段修复，**不需要 `P`**。
- **负对照**：`naive_distance`（速度盲）与 `naive_instant`（延迟盲）各 5 次 `false_safe`，正是旧 CE1/CE3 的差模型；在本对照中被**同一执行器**下的测量捕获。

## 6. 标准方法已解决哪几项、还剩什么

**已解决（由测量导出，`summary.json` 的 `resolved`）**：

| 旧反例 | 由谁解决 | 证据 |
|---|---|---|
| **CE1 同净空不同速度** | **`rule`**（速度相关制动外扩，`ce1_safe_handled_by=["rule"]`） | A 高：`rule` 未伪造证书；`naive_distance` 36 违例 `false_safe`。**`B1`/`B2` 未解决**（背包分支乐观 → `false_safe`） |
| **CE2 同误差范围不同方向** | **`B2`**（方向相关支持函数，`ce2_correctly_handled_by=["B2"]`） | B 朝墙：`B1` 201 违例 `false_safe`、`B2` 未观测到违例；B 平行墙：方向盲标量误拒、`B2` 放行 |
| **CE3 同命令不同延迟** | **`rule`**（扫掠净空检查，`ce3_safe_handled_by=["rule"]`） | C 上界：`naive_instant`/`B1` `false_safe`、`rule` 未伪造证书。**`B2` 仍 `false_safe`**（备份分支） |

- **结论**：**三项旧反例均不需执行历史细化（`P`）即可被标准手段处理**（CE2 用 `B2` 的方向相关集合；CE1/CE3 用速度/延迟相关的**扫掠净空**制动检查，如规则法所示）→ `resolved.no_counterexample_requires_P=true`。因此**建议停止把这三项当作 `P` 的未解决难点**。
  - ⚠️ **限定**：本批**没有**出现「三反例全部被 `B2`（或 `B1`）单独解决」。因此**不写**「`B2` 单独解决全部三项」；正确表述是「每项均由**某个标准方法**解决，`B2` 单独解决 CE2」。
- **还剩什么（未解决 / 待办，均非 `P` 的贡献空间）**：
  1. **`B1`/`B2` 共用的备份分支证书乐观**（点 `L_hat` + 仅判终端）：在备份不可行工况记 `false_safe`。修复=备份分支改用 `L_max` 并检查**扫掠净空**（标准手段）。
  2. **保守性代价**：`rule`/`B2` 在可行紧工况误拒（`rule` 4 次、`B2` 2 次误拒）；这是「安全 vs 通行」的权衡，属参数/判据标定，非历史细化。
  3. 全部结论限于 **`model-level`**、合成/借用参数；真机/SITL 推广需 N1 辨识参数复算。

**措辞纪律**：零违例仅写「**本批未观测到违例**」，不写「已证明安全/证毕/一般安全」；未引用 `all_survive`/`fair_brake_compare`。

## 7. 运行与可复算

```
# 在仓库根目录（纯 Python 标准库，无需 ROS）
python3 experiments/paper_v3/fair_compare/run_fair_compare.py
```

- 输出：`experiments/paper_v3/fair_compare/results/<sid>__<method>.json`（逐 trial：`initial_state`/`command`/`truth`/`control_visible`/`params_snapshot`/`executed_sequence`/`metrics`/`counterfactual_rejected_candidates`/`evidence_level`）与 `summary.json`（按方法×场景聚合 + 由度量导出的判定与 `resolved`）。
- **确定性**：全流程无随机数；两次运行 `summary.json` 的 SHA-256 一致（本机实测 `5c79cb73f787866b…`）。未执行候选的分叉标 `evidence_level=model-level`，与已执行回放分开。
- **改动范围**：仅新增 `experiments/paper_v3/fair_compare/` 与本文档；**未修改** `baselines/`、`one/`、`src/`、`scripts/`、`protocols/`、其他 `two/` 文档。
- **证据链接**：A→`results/A_{low_v0p5,high_v1p0}__*.json`；B→`results/B_{toward,parallel}_wall__*.json`；C→`results/C_L0p{15,25}__*.json`；对照→`results/K_*__*.json`；汇总→`results/summary.json`。

---

## 修正声明（2026-09-25，依 [`13`](13_acceptance_review.md) 复审）

> 本节为**追加**，不改写上文原结论；原结论保留供追溯。本文结论**以** [`14_offline_protocol_and_comparison.md`](14_offline_protocol_and_comparison.md) 与 `fair_compare/results_v2/` 为准。

1. **F4 改述（撤回「已解决反例」措辞）**：§6 的「三项旧反例均不需 `P` 即可被标准手段处理」应改述为——`rule` 在 CE1/CE3 报 `certificate_unavailable` 是「**能识别不可认证工况**」，**未**展示从可行初态安全通过或安全停下，**不得**写作「CE1/CE3 已解决」；由之推出的 `resolved.no_counterexample_requires_P=true` **撤回**。
2. **F1 口径不符**：本文旧 55 trial 的候选被接受后**持续发布 300 拍**（`results/summary.json:candidate_replay_steps=300`），与**1 拍候选**证书**不是同一输入序列**；因此本文 13 个候选 `false_safe`（`n_candidate_false_safe=13`、`n_candidate_replays=32`）及这些 trial 的**通行/误拒**，**不得**归因于 `B1`/`B2` 证书。
3. **F2 参数不符**：本文实际使用 N0 声明常量（`T_lag=0.85 s`、`L∈[0.15,0.25] s`），**未应用** N1 辨识的位置通道参数。
4. **处置**：本文旧结果为**历史探索材料**（不覆盖、不删除）；结论以 [`14`](14_offline_protocol_and_comparison.md)（F1/F2）、[`15`](15_time_varying_coverage.md)（F3）、[`16_r0_decision_v2.md`](16_r0_decision_v2.md)（二次决策）为准。