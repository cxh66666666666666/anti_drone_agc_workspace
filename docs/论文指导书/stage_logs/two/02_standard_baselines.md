# 02 标准预测基线 B1/B2 的模型级原型（阶段 two，N0）

> 日期：2026-09-24　阶段：two　依据：[阶段 two 任务书](publication_tasks.md) §3 **N0**；**依赖 P0、P3**（文献精读可提前）。
> 交付物：本文档 `docs/指导书/stage_logs/two/02_standard_baselines.md`；配套代码 `experiments/paper_v3/baselines/`（本文只读引用）。
> 方法标签一律以 [01_design_corrections.md](01_design_corrections.md) §1 为准：**`Legacy` / `B1` / `B2` / `P`**。
> 术语首次出现给英文全称；PDF 出处均给 **PDF 物理页**。

---

## 1. 目的与范围 ＋ 证据级别声明

- **N0 目标**（[第二波任务书](publication_tasks.md) §3 N0）：精读关键原文，并用一个**定高二维、位置命令输入**的低阶闭环模型实现标准 `B1`/`B2`，`B2` 为不引入拟议细化的成熟对照，`P` 只留接口。
- **证据级别**：本文全部结论为 **`model-level`**（模型级）；不构成真机、SITL（software-in-the-loop）或闭环飞行结论。
- **参数性质**：`model.py` 的 `UNIT_TABLE`/`PARAMS` 全为**合成（synthetic）/借用（borrowed）**值——**明确声明：非 PX4 实测、非 SITL 实测**（`model.py:5-6,20-49`）。借用项的原始出处见 §3。
- **改动范围**：本 N0 仅**新增** `experiments/paper_v3/baselines/`（`model.py`/`backup.py`/`filters.py`/`run_baselines.py`/`results/`）；**未修改**旧 `one/` 数据与结果，**未修改** `src/` 生产源码。
- **接口口径**：模型输入对齐 [execution_contract_v2.md](../../../../experiments/paper_v3/protocols/execution_contract_v2.md) 选定的「直接 PX4 位置设定点台架」（`/mavros/setpoint_position/local`）与共同备份语义（契约 §4）；速度量只取契约 §3.2 的合法来源。

## 2. 文献精读要点（均标 PDF 页码）

> 逐方法差异另见 [08_nearest_neighbor_table_v2.md](08_nearest_neighbor_table_v2.md) §2/§2.1/§2.2；此处**不重复整表**，仅固化与 B1/B2 设计直接相关的原文事实。

### 2.1 DDSF（Data-Driven Safety Filter，Bajelani & van Heusden，arXiv:2309.00189v1）
- **输入-输出安全集（I/O safety set）Definition 7**，PDF 第 4 页（§III-A）：输入-输出框架下**只约束 `U × Y`、不考虑状态约束 `X`**，也不需要状态量测/估计；安全初始轨迹与 backup 轨迹须满足 `U × Y` 且**可拼接（patched）**。
- **有限时域 QP 问题 (9)**，PDF 第 4 页（§III-B）：(9a) 最小化对学习输入的修改 `‖ū₀−u_l‖²_R`；(9b) Hankel 等式约束；(9c) 用过去 `T_ini` 步实测 I/O 固定初值；(9d)(9e) 预测段满足 `U`、`Y`；(9f) 终端段落于终端安全集 `S_f`；`ū、ȳ` 依赖系数 α、非独立决策变量（第 5 页）。
- **关键假设**：确定性、离散时间 **LTI（linear time-invariant）**（第 2 页 §II）；**无噪声、持续激励（persistent excitation, PE）**的单条 I/O 轨迹（第 3 页 §II-B）；**A1**（`N_p > T_ini ≥ l(A,C)`）、**A2**（堆叠 Hankel 为 `L` 阶 PE）、**A3**（终端集 `S_f` 控制不变），PDF 第 4 页；递归可行 Theorem 2 第 5 页。
- **作者自陈局限**（§V，PDF 第 7 页）：确定性框架下，真实**噪声/扰动/非线性**会致预测误差与约束违反，通常靠**收紧约束**获鲁棒解**因而引入保守性**；长时域 QP **计算昂贵**。
- 与本项目接口相关的对比事实：DDSF **可处理未知时延**且**不使用延迟显式信息**（第 1、6—7 页）。

### 2.2 Layered Safety（Yamaguchi, Bena, Bahati, Ames，arXiv:2603.00338v2）
- **Eq.19（实时层 ISSf CBF-QP）**，PDF 第 5 页（§III-D）：`μ_s = argmin‖μ−μ_p‖² s.t. ∇h·μ + ∂h/∂t ≥ −αh + (1/ε)‖∇h‖²`；其中 **`(1/ε)‖∇h‖²` 是补偿 ROM（reduced-order model）与全阶系统之间跟踪误差的鲁棒项**——即**跟踪误差补偿项**，**不是方向相关误差集合**，**也不是延迟项**。ISSf = **input-to-state safe（输入-状态安全）**。
- **Theorem 1（Full-Order System Safety）**，PDF 第 5 页：**条件**保证——`h` 为 ROM 的 CBF（control barrier function）、`χ̇_s` 满足 Eq.19（`ε>0`），且存在**跟踪控制 Lyapunov 函数（tracking CLF）`V`** 满足 (21)(22)，则 `λ ≥ α + εμ/(4β)` 时集合 `C_F(t)={B≥0}` 前向不变；**前提是 CLF 与跟踪控制器存在且有效**。
- **Remark 1（Verification 保守性换条件）**，PDF 第 6 页：`V`、`k` 是**验证工具**；当 `V` 未知/黑箱时，可把 Eq.19 的 `α, ε` 取小以**保守满足**。

### 2.3 Wabersich–Zeilinger 2021（arXiv:1812.05506v4，*Automatica* 129:109597）
- **模型假设**，PDF 第 3 页 §3：**非线性**离散时间系统 `x(k+1)=f(x(k),u(k);θ_R)`；数据驱动模型信念 `θ~p(θ)`（第 4 页 Eq.(2)）；**状态/输入相关不确定性**（第 8 页 Definition 4.4）。安全定义为概率式 `Pr(...) ≥ p_S`（Eq.4，第 4 页）。
- **终端集 `S^t`：Assumption 4.2**，PDF 第 6 页（§4.1 末）：`S^t := {x | a_S(x) ≤ 1}`，`a_S` Lipschitz，且存在终端安全滤波器 `π_{S^t}` 使进入 `S^t` 后恒满足 `X`、`U`。
- **递归可行（recursive feasibility）**：名义情形 §4.1（第 5—6 页）用上一最优序列尾部实现缩短时域；不确定性情形 §4.2（第 6—8 页）需 Assumption 4.2、4.3、4.5 与约束收紧递归 (10)；**主结果 Theorem 4.6**（第 8—9 页）在 `L_{E_{pS}} ≤ cε` 下保证概率安全；保守初值 `ρ≈1, ε≈N⁻¹`（第 9 页）；不确定性过大可致初始不可行（Remark 4.7，第 9 页）。

### 2.4 RTD / FaSTrack（仅取与本路线直接重合段落）
- **RTD（Reachability-based Trajectory Design，Vaskov 2019，arXiv:1902.01786）**，PDF 第 6 页：制动回退**假定制动执行无延迟**——原文「assumes no delay in braking actuation; **if there is delay, then T must be increased to include it**」。即 RTD 的延迟处理只是**增大时域**。
- **FaSTrack（Chen 2021，arXiv:2102.07039）**：**TEB（tracking error bound）** 界定**规划模型与跟踪模型之间的失配（mismatch）**，形象为规划模型周围的「safety bubble」（PDF 第 1、3 页）；TEB 在**相对系统误差状态空间**、只依赖相对状态 `r`（第 4 页）；**Proposition 1** 给有限时域内保证（第 7 页）。FaSTrack/RTD 的 TEB/FRS **只针对模型失配与有界扰动，不建模执行延迟**；可作「执行滞后位置误差」的**近似替代**，非延迟专项项。

## 3. 模型与单位表

模型版本 `MODEL_VERSION = two-N0-baselines-v1`（`model.py:31`）。

| name | 物理量 | 单位 | 值 | 来源标签 |
|---|---|---|---|---|
| `dt` | 采样步长 | s | 0.02 | `synthetic` |
| `T_lag` | 位置通道一阶时间常数 | s | 0.85 | `borrowed`（旧 z 轴阶跃时间常数） |
| `L_hat` | 纯延迟点估计 | s | 0.20 | `synthetic` |
| `L_min` | 延迟区间下界 | s | 0.15 | `synthetic` |
| `L_max` | 延迟区间上界 | s | 0.25 | `synthetic` |
| `v_stop` | 停稳速度阈值 | m/s | 0.05 | `borrowed`（制动协议） |
| `margin` | 设计安全裕量 | m | 0.15 | `borrowed`（CE2） |
| `a_long` | 方向误差沿向半轴 | m | 0.20 | `borrowed`（CE2 量级） |
| `a_lat` | 方向误差横向半轴 | m | 0.05 | `borrowed`（CE2 量级） |

- 另有备份限幅 `MAX_OFFSET_CHANGE = 0.25 m`（`borrowed`：契约 §4.3 / `gazebo_env.py`，`backup.py:19`）。
- **状态**：位置 `p∈R²`、速度 `v∈R²`（**由位置差分自洽** `v=(p_k−p_{k−1})/dt`）、已发布命令历史 `cmd_history`（末元素为 `cmd_{k−1}`）及拍号 `k`（`model.py:97-107`）。
- **输入**：位置命令（设定点，m），逐拍发布。
- **动态**（`model.py:119-131`）：纯延迟 `n_L = ceil(L/dt)` 拍 ＋ 一阶滞后 `α = 1 − exp(−dt/T_lag)`；生效命令 `g_k = cmd_{k−n_L}`；`p_{k+1} = p_k + α(g_k − p_k)`；`v_{k+1} = (p_{k+1} − p_k)/dt`。**无未定义状态转移、无隐含真值输入**（对齐契约 §5）。
- **单位纪律**：**无 `m` 当 `m/s`**——任何位置偏移量（m）不得直接充当速度（m/s）；速度只由位置**时间差分**得到（契约 §3.1/§3.2；纠正 `one/03` 的 `max_offset_xy` 当速度之误，见 [01_design_corrections.md](01_design_corrections.md) §2.1 E4）。

## 4. 方法差异：Legacy / B1 / B2 / P

- **`Legacy`**＝旧**恒速动作映射**（`one/` 冻结项，[01_design_corrections.md](01_design_corrections.md) §1）：复用旧 `safety_filter.action_velocity`/`predict_trajectory` 的「当前速度＋定值增量」恒速外推，不建模延迟/滞后。**本 N0 原型未实现 Legacy 代码**，此处仅为标签对齐。
- **`B1`**＝与实际位置命令对应的**标称闭环预测 ＋ 共同备份**（`filters.py:28-55`）：声明**延迟点估计** `L_hat`、误差集合退化为 **`E={0}`**；对候选做**显式展开**（候选一拍 → 共同备份 → 保持）并逐点检查 `clearance ≥ margin`，末端用终端条件判定。
- **`B2`**＝在 B1 之上作**结构性**改动（`filters.py:58-101`）：**延迟增广**（用已发布命令历史，延迟取**区间** `[L_min,L_max]` 而非单点，逐分支传播）＋ **方向相关误差集合**（支持函数 `σ_E(n)=√((a_long·n_x)²+(a_lat·n_y)²)`，`model.py:89-93`）＋ **集合传播**得鲁棒可达集；鲁棒判据 `clearance(cand_cmd) − σ_E(−∇h) ≥ margin`（最坏方向 `−∇h`）；**共用同一备份函数**（`backup.py`）。
- **`B1` ↔ `B2` 是结构性差异**：差异在**延迟建模**（点 vs 区间/增广）与**不确定性处理**（`E={0}` vs 方向相关集合＋传播）**两个层面**——**不得**写成「只差一个 κ 或一个标量集合」（[01_design_corrections.md](01_design_corrections.md) §2.1 E1/E2）。
- **`P`**＝**仅接口**（`filters.py:168-172`）：占位实现，返回 `status=not_implemented`；本阶段**不引入新网络、不做大规模搜索**。

## 5. 候选展开与共同备份

- **显式三段展开**：**候选一拍 → 共同备份 → 保持**（`backup.py:33-47`）。候选一拍用 `cand_cmd`；其后发布**共同备份**（目标为实测水平位置）；再保持。
- **备份目标**＝触发拍**当前实测水平位置** `p_k`（`backup.py:22-25`），**非**冻结旧参考相位（对齐契约 §4.2，区别见契约 §1.3）；在显式展开里备份段自**候选一拍之后**的状态起算、目标取该拍实测位置 `p_{k+1}`（`backup.py:37-45`）。
- **保存上次有效备份**：`LastValidBackup` 记录保持设定点与剩余可执行拍数（`backup.py:57-69`）；本拍不可行时**执行剩余备份**并置 `used_last_valid_backup=true`（`filters.py:134-141`），**不得**输出伪造的通过标签。
- **初态不支持备份**：无候选安全、纯备份也不入 Ω、且无上次有效备份时 → **`certificate_unavailable`**（`filters.py:142-148`）。
- **证书（certificate）三项条件**：
  - **(a) 候选目标可达性** `clearance(cand_cmd) ≥ margin`（`B2` 用鲁棒 `clearance − σ_E(−∇h) ≥ margin`）；
  - **(b) 展开段净空** `min positions[1:] ≥ margin`（`B2` 在 `L_min/L_max` 分支取最坏）；
  - **(c) 展开末态落入 Ω**。
  - （a)(b)(c) 同时成立才 `accepted`（`filters.py:38-41,81-83`）。
- **实现边界（诚实声明）**：① `MAX_OFFSET_CHANGE = 0.25 m` 仅为**接口对齐声明**，本模型级原型**未实际施加**单拍限幅（原型不执行真实发布，`backup.py:7`）；② `LastValidBackup` 的**保存**由场景脚手架（`run_baselines.py` 的 S3 子例）注入，`decide()` 负责**消费**（执行剩余备份）。二者均不改证书语义，也不作为已实现保护功能的证据。

## 6. 可计算终端条件

- **终端集** `Ω = { 净空 ≥ margin 且 |v| ≤ v_stop }`（`model.py:146-152`），由展开序列**数值**判定（末状态）。
- **声明**：短时低速只是**经验停稳观察**，**不等于已证明不变集**；不变性主张仅限**线性模型局部范围**（`model.py:150`）。这与 N0 要求一致（[第二波任务书](publication_tasks.md) §3 N0 第 4 条）。
- 与最近邻对照：WZ-2021 的 `S^t` 需**控制不变**（Assumption 4.2，p6），本原型**未证明**该性质，只作数值判定。

## 7. 可运行入口与基础结果

**入口**（在仓库根目录，纯 Python 标准库，无需 ROS）：

```
python3 experiments/paper_v3/baselines/run_baselines.py
```

结果写入 `experiments/paper_v3/baselines/results/`（`s1_open.json` / `s2_danger.json` / `s3_short_horizon.json` / `s4_margin_disabled.json` / `summary.json`）。

四场景数值（均取自 [summary.json](../../../../experiments/paper_v3/baselines/results/summary.json)；评分侧注入真值延迟 `L_true = 0.25 s`、回放 `GRADE_STEPS = 300`，`run_baselines.py:32-34`）：

| 场景 | 方法 | 最小净空 (m) | 裕量违例 | 物理违例 | 终态速度 (m/s) | 入 Ω | 是否替换候选 | 证书 |
|---|---|---|---|---|---|---|---|---|
| S1 开阔 | B1 | 99.400568 | 0 | 0 | 0.000676 | 是 | 否（→ `cand_fwd`） | `valid` |
| S1 开阔 | B2 | 99.400568 | 0 | 0 | 0.000676 | 是 | 否（→ `cand_fwd`） | `valid` |
| S2 危险 | B1 | 0.486678 | 0 | 0 | 0.000157 | 是 | 是（替换 `cand_into_wall` → 备份） | `valid` |
| S2 危险 | B2 | 0.486678 | 0 | 0 | 0.000157 | 是 | 是（替换 `cand_into_wall` → 备份） | `valid` |
| S3 短时域(N=20) | B1 | 99.886678 | 0 | 0 | 0.000157 | 是 | 是（替换 `cand_fwd` → 无候选/无备份） | **`unavailable`** |
| S3 短时域(N=20) | B2 | 99.886678 | 0 | 0 | 0.000157 | 是 | 同上 | **`unavailable`** |
| S4 裕量置 0 | B1 | **0.05051** | **225** | 0 | 0.000607 | **否** | 否（错误放行 `cand_near_wall`） | `valid` |
| S4 裕量置 0 | B2 | 0.486678 | 0 | 0 | 0.000157 | 是 | 是（替换 `cand_near_wall` → 备份） | `valid` |

补充：
- **S2 保守性探针**：同一边缘候选 `cand_marginal`（`clearance=0.25`）→ **B1 接受、B2 拒绝**（`robust_target_clearance = 0.05 < margin`；`sigma_at_neg_gradh = 0.2`），说明 B2 因延迟区间与方向误差**更保守**（`s2_danger.json` 的 `conservativeness_probe`）。
- **S3 上次有效备份子例**：本拍不可行但存在上次有效备份 → `selected_kind = last_valid_backup`、`used_last_valid_backup = true`、`certificate = valid`（`s3_short_horizon.json` 的 `last_valid_backup_case`）。
- **S3 时域口径澄清**：表中「入 Ω＝是」是**评分侧 300 拍独立回放**（备份最终停稳）的度量；而决策侧时域仅 `N=20`（0.4 s），展开末态**未**在 N 内落入 Ω → `terminal_fail` → `certificate_unavailable`。二者口径不同，不可混说（`run_baselines.py` 的 `GRADE_STEPS=300` vs 决策 `N=20`）。
- **结论由度量推出**：场景判定**不是**检查 `passed`/`safe` 标签，而是对**已执行命令序列**用模型**独立回放**（评分侧、注入真值延迟）计算实测最小净空/违例计数/终态速度/是否入 Ω 再由这些度量推出（`run_baselines.py:37-55、58-66、118-138`）。
- **确定性可复算**：全流程无随机数，参数冻结于 `MODEL_VERSION`，结果可离线复算。

## 8. 与 N0 验收的逐条对应 ＋ 结论

| N0 验收（[第二波任务书](publication_tasks.md) §3 N0） | 对应证据 |
|---|---|
| **开阔空间接受正常候选** | S1：B1/B2 均接受 `cand_fwd`（最小净空 99.400568、0 违例、入 Ω、`certificate=valid`）；`s1_open.json` |
| **危险候选被替换且备份确实执行** | S2：`cand_into_wall` 被替换、`selected_kind=backup`、`executed_is_backup_hold_pk=true`（备份确实执行、保持 `p_k`）；`s2_danger.json` |
| **故意缩短时域或破坏终端条件能暴露失败** | S3（N=20）：三项条件因末态不入 Ω 而 `terminal_fail` → `certificate_unavailable`；S4（决策侧 `margin=0`）：B1 错误放行近墙候选 → **225 裕量违例、不入 Ω**；`s3_short_horizon.json`、`s4_margin_disabled.json` |
| **不能仅检查返回值标签** | 判定由**独立回放的度量**推出（`run_baselines.py` §评分侧 `grade`/`_verdict`），而非标签；S4 的 B1 证书虽为 `valid`，但度量暴露 225 次裕量违例 |

**结论（措辞受限）**：
- 本批四场景的预期均被观测到（`summary.json` 的 `all_expected_observed = true`）；但零违例只写「**本批未观测到违例**」，**不写**「已证明安全/证毕/一般安全」（对齐 `prediction_replay_format_v2.md` §4.5）。
- 参数为**合成/借用**值，**非 PX4 实测、非 SITL 实测**；详见 §1、§3。
- B2 相对 B1 的差异为**结构性**（延迟建模＋不确定性处理两层），非单标量/单一误差集合；本 N0 判据由测量计算、非常量。

## 9. 链接与措辞约束

- **结论链接**：数值 → `experiments/paper_v3/baselines/results/{s1_open,s2_danger,s3_short_horizon,s4_margin_disabled,summary}.json`；文献 → §2 各 PDF 页码。
- **不得引用** `all_survive` / `fair_brake_compare`（其 `all_survive=true` 不构成公平对照证据；[01_design_corrections.md](01_design_corrections.md) §B.4）。
- **不得**写「已证明安全 / 证毕 / 一般安全」等过强表述；零违例只写「本批未观测到」。
- **不得**把合成/借用参数写成实测；所有结论限定在 `model-level`。