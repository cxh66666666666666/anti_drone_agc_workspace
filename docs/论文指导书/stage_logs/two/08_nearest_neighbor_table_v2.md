# 08 最近邻方法差异表 v2（阶段 two，P4）

> 日期：2026-09-24（**第三轮补正**）　阶段：two　依据：[阶段 two 任务书](publication_tasks.md) §2 **P4**、验收 [F5](../one/09_acceptance_review.md)。
> 修订对象：v1 [`../one/05_nearest_neighbor_table_v1.md`](../one/05_nearest_neighbor_table_v1.md)（**原文保持不动，旧文档冻结**）。
> 方法标签一律以 [`01_design_corrections.md`](01_design_corrections.md) §1 为准：**`Legacy` / `B1` / `B2` / `P`**。
> 填写格式：「相比方法 X 的假设 A，本项目处理限制 B；新步骤 C 带来性质或收益 D，代价 E；实验 F 能单独检验它」。
>
> **本轮更新**：用户补入 **Layered Safety（arXiv:2603.00338v2，8 页）** 正文；据此把该行由「已核验摘要＋节选」升级为**已读原文**，补齐 **Eq.19 与 Theorem 1**（见 §1.4、§2、§2.1）。至此**五篇最近邻正文全部读完**。

## 1. v2 的硬性更正

### 1.1 以 **B2** 为最近邻主对照；P 的候选差异不是「比 B1/旧 DQN 好」

- **最近邻主对照 = B2**（延迟增广、方向相关鲁棒预测 ＋ 共同备份）；`Legacy`/`B1` 只作诊断/标准对照（`docs/指导书/stage_logs/two/publication_tasks.md` §3 N0/N2、`publication_plan.md` §7.1）。
- **P = 拟议的执行历史细化**（含分支保留与**计算预算**权衡）；候选差异**不**退回「比 `B1`/旧 DQN 好」，**也**不写成「只差一个误差集合」（`01_design_corrections.md` §2 E3、§3）。

### 1.2 Wabersich–Zeilinger 身份纠正（已获原文）

v1 曾把 WZ-2021 当作综述引文 [78]（实为**两条不同文献**混引，验收 F5）。经原文首页核对：

| 编号 | 准确题名 | 年份/版本 | 发表处 | 标识 / 链接 | 读取深度 |
|---|---|---|---|---|---|
| **WZ-2018** | *Linear model predictive safety certification for learning-based control* | arXiv:1803.08552v6，**7 页** | **CDC 2018, pp. 7130–7135（已确认）** | [arXiv:1803.08552](https://arxiv.org/abs/1803.08552)；DOI `10.1109/CDC.2018.8619829`（据施引文献） | **已读原文** |
| **WZ-2021** | *A predictive safety filter for learning-based control of constrained nonlinear dynamical systems* | arXiv:1812.05506v4，**17 页** | ***Automatica* 129:109597（已确认）** | [arXiv:1812.05506](https://arxiv.org/abs/1812.05506)；DOI [`10.1016/j.automatica.2021.109597`](https://doi.org/10.1016/j.automatica.2021.109597) | **已读原文** |

- 综述 §3.3.3 的施引 **[78] 对应 WZ-2018（线性 MPSC）**，**不是** WZ-2021（非线性 PSF）。

### 1.3 区分 RTD 与 ARMTD（防混引）

- **RTD（主对照）= Vaskov 等 2019（车辆，ACC 2019）** ＋ **RTD 原始移动机器人工作**（Kousik 等；题名/年份**待核验**，见 §5）。
- **ARMTD = 机械臂扩展（Holmes 等 2020，Fetch 7-DOF）**，**相邻参考**，**不得**当作移动机器人 RTD。

### 1.4 归档原文与指纹（本轮新增 Layered Safety）

| 归档文件名 | 文献身份（PDF 首页核对） | 读取深度 | SHA-256 |
|---|---|---|---|
| `docs/Thesis/文献/预测安全过滤\|可达集/DDSF_Bajelani_arXiv2309.00189.pdf` | Bajelani & van Heusden，*Data-Driven Safety Filter: An Input-Output Perspective*，arXiv:2309.00189v1，8 页 | **已读原文** | `9958fdcab238a60f7725cf722e264b15dba7d56dc0a947f3b9429932a08dd126` |
| `docs/Thesis/文献/预测安全过滤\|可达集/WZ2018_Wabersich_arXiv1803.08552.pdf` | Wabersich & Zeilinger，*Linear model predictive safety certification for learning-based control*，arXiv:1803.08552v6，7 页 | **已读原文** | `0acf555d70885f7e3e310c1f5ce3ba6d7d8cf0b8606e5ba85ee3532c32ebbe36` |
| `docs/Thesis/文献/预测安全过滤\|可达集/WZ2021_Wabersich_arXiv1812.05506.pdf` | Wabersich & Zeilinger，*A predictive safety filter for learning-based control of constrained nonlinear dynamical systems*，arXiv:1812.05506v4，17 页 | **已读原文** | `f1604802bf91f0eb672ca04b7772eee536dfccebd4a42072456130ba74fe8f52` |
| `docs/Thesis/文献/预测安全过滤\|可达集/FaSTrack_Chen2021_arXiv2102.07039.pdf` | Chen 等，*FaSTrack…*，arXiv:2102.07039v1，16 页；发表处 *IEEE Trans. Automatic Control* **66(12):5861–5876, 2021**（**用户提供 2026-09-24**） | **已读原文** | `ad2b23bfc5ff46747cddc2dab53a9357e131ac16f68291a005d35f71f3e93718` |
| `docs/Thesis/文献/预测安全过滤\|可达集/RTD_Vaskov2019_arXiv1902.01786.pdf` | Vaskov 等，*Guaranteed Safe Reachability-based Trajectory Design…*，arXiv:1902.01786v2，8 页 | **已读原文** | `a6c3f8f364e9a7238bdd94eb45744b3da1c4583aca634499c0cbf9187aa0ba13` |
| `docs/Thesis/文献/预测安全过滤\|可达集/ARMTD_Holmes2020_arXiv2002.01591.pdf` | Holmes 等，*Reachable Sets for Safe, Real-Time Manipulator Trajectory Design*，arXiv:2002.01591v2，14 页（**相邻参考：机械臂**） | **已读原文** | `fc74fdda25b2b0e25fc3c392beb32c81452ed52c4dd86c066a3b5a0642623313` |
| `docs/Thesis/文献/预测安全过滤\|可达集/LayeredSafety_Yamaguchi2026_arXiv2603.00338.pdf` | Yamaguchi, Bena, Bahati, Ames，*Layered Safety: Enhancing Autonomous Collision Avoidance via Multistage CBF Safety Filters*，arXiv:2603.00338v2，8 页 | **已读原文** | `ee68d1445b7729c9ef4fcefdf2db62626cb963cdfeca0a06c41b59ba9e9c8afa` |

## 2. 差异表 v2

| 方法（标识 / 读取深度） | A 其假设 | B 本项目限制 | C 新步骤（候选差异） | D 收益（候选） | E 代价（候选） | F 检验实验 |
|---|---|---|---|---|---|---|
| **DDSF**（arXiv:2309.00189v1 / **已读原文，8 页**） | **确定性离散时间 LTI**（A,B,C,D 未知，p3）；只需**一条噪声自由、持续激励**的 I/O 轨迹（Hankel/BST，p3）；无需全状态与状态估计；在 **I/O 框架处理未知时延** | 本项目闭环含**饱和/速率受限与执行滞后**；DDSF 的"确定性 + 无噪声 LTI"不覆盖。**但 DDSF 已显式处理 I/O 历史与未知时延**，不可描述为"未处理延迟/历史" | 候选差异**不能**再表述为"引入延迟增广/历史"；应落在**方向相关误差集合 + 可执行制动备份 + 位置接口辨识** | 对轨迹外组合不必建显式模型；但**不**为本项目方向/制动主张背书 | 需重新辨识；集合标定不准则退化；长时域 QP 昂贵（p7） | N2/N3（**未执行**） |
| **Layered Safety**（arXiv:2603.00338v2 / **已读原文，8 页**） | 机器人本体**点云 → 占用栅格 → Poisson 安全函数 PSF → 作 CBF**（p1—p2）；用**降阶模型 ROM** 综合约束；**假设低层控制器准确跟踪 ROM 速度命令**（p1、p4—p5） | 本项目主接口是**位置设定点**（非速度命令），跟踪是**带延迟/滞后/过冲**的位置环；现过滤器仍用真值几何 | 延迟增广 ＋ 方向集合鲁棒收紧（`h_rob = h − σ_E(∇h)`）＋ 可恢复性约束；Layered 的鲁棒项是 **Eq.19 的 `(1/ε)‖∇h‖²`（跟踪误差补偿）**，**非方向集合** | 不依赖"准确跟踪"假设即可给接口一致的鲁棒判据 | 需辨识延迟/误差集合；预测计算重于 CBF-QP | N2/N3（**未执行**） |
| **WZ-2018**（arXiv:1803.08552v6 / **已读原文，7 页**） | **多面体线性 + 加性扰动**（p1）；需**已知/有界不确定模型**；MPSC 验证学习输入、必要时最小修改（p1）；tube-based/鲁棒 MPC 构造**安全备份律 `uB`**（Def. II.1, p2）通向**终端集 `Sf`**（Assumption III.3, p3） | 本项目为**黑盒位置设定点接口、需辨识** | 位置接口辨识 ＋ 方向相关误差集合 ＋ 可执行制动备份；WZ-2018 鲁棒性是"加性扰动 + tube"，非方向集合 | 已知延迟区间 + 方向集合下的可恢复判据 | 需重新辨识；计算量 | N2/N3（**未执行**） |
| **WZ-2021**（arXiv:1812.05506v4 / **已读原文，17 页**） | **非线性**；**数据驱动模型 + 状态/输入相关不确定性**（p2—p3，可 GP p8）；备份计划通向**终端集 `S^t`**（Assumption 4.2, p6）；递归可行（§4.2, p7/p14/p16） | 同 WZ-2018：位置接口 + 需辨识 | 执行历史细化/分支保留 ＋ 计算预算 | 若其不确定界已足够紧，本项目"历史细化"**可能无独立收益**（N3 判） | 计算量；界过保守 | N3（**未执行**） |
| **Kousik RTD**（Vaskov 2019 / **已读原文，8 页**） | 参数化轨迹 + 跟踪控制器；离线预计算**含跟踪误差的 FRS**（p3）；在线 FRS→障碍约束（p2） | 执行滞后 + 位置接口；FRS 离线固定 | 延迟增广 ＋ 方向集合 ＋ 共同备份；FRS 未显式含执行延迟 | 已知延迟区间下的可恢复判据 | 需重新辨识；计算量 | N2（**未执行**） |
| **Chen FaSTrack**（Chen 2021，arXiv:2102.07039v1；*IEEE TAC* 66(12):5861–5876, 2021，DOI `10.1109/TAC.2021.3059838` / **已读原文，16 页**） | 规划模型 + 跟踪模型；离线 **HJ 可达性**预计算 **TEB**（p1—3）；在线障碍按 TEB 膨胀（p2） | "高低保真"对含未知执行延迟；TEB 面向失配/扰动 | 延迟增广 ＋ 方向集合 ＋ 共同备份；TEB 不含执行滞后 | 已知延迟区间下覆盖执行滞后 | 需重新辨识；HJ 预计算成本 | N2/N3（**未执行**） |
| **ARMTD**（Holmes 2020 / **已读原文，14 页，相邻参考：机械臂**） | Fetch 7-DOF 机械臂；离线每关节 **JRS**，含 **fail-safe 制动**（p1—2）；在线组合作业空间 RS ＋ 滚动优化 | 平台/动力学不同 | 仅**相邻参考**，不为主对照提供差异基线 | （相邻参考） | （相邻参考） | （相邻参考） |

### 2.1 关键列一句话（供审查）

| 方法 | 控制输入 | 内环/跟踪假设 | **延迟处理** | 误差/约束集合 | **备份策略** | **终端条件 / 递归可行** | 感知权限 | 对照 | 可复现代码 |
|---|---|---|---|---|---|---|---|---|---|
| **Layered Safety** | 学习/名义速度命令 `µd`；预测层输出 `µp`，实时层输出 `µs` | ROM 上综合；**假设全阶低层控制器准确跟踪 ROM 速度命令**（p1、p4—5） | **原文未显式处理执行延迟**（全文检索 `delay`/`latency`/`actuator dynamics` **无命中**，逐页确认）；鲁棒性只通过 **Eq.19 的 `(1/ε)‖∇h‖²`** 补偿**跟踪误差**（非延迟） | **PSF 0-超水平集**（占用栅格合成，p1—2）＋ **Eq.19 ISSf CBF-QP** 约束 | **无独立 fail-safe 制动备份**；安全靠两层 CBF 使安全集**前向不变**（预测层 DCBF ＋ 实时层 CBF-QP，p4—5） | 预测层时域内逐点 **DCBF `h(ξ_{i+1},t_{i+1}) ≥ ρ h(ξ_i,t_i)`**（p4）＋终端代价 `V(ξ_N)`；**无独立终端安全集** | **仅本体点云**（p1） | 与单级过滤对比 **Pareto 分析**（p6—7） | **原文未见代码链接** |
| DDSF | `uL`；QP 输出最小修改 `ū0` | 无需显式模型/状态估计（BST+Hankel） | **已处理未知时延**（不加显式信息，全来自数据，§IV-B/§V p6—7）；历史经 `Tini`（p4） | I/O 安全集（Def.7, p4）＋ 终端集 `Sf` 控制不变（Assumption 3, p4） | **有**：备份轨迹与初始轨迹**可拼接**，通向 `Sf`（p4） | `Sf` 控制不变；递归可行 Theorem 2（p5） | 不涉及感知 | 线性化 6-DOF 四旋翼 + 时延 ACC（p5） | 未见代码链接 |
| WZ-2018 | `uL`；MPSC 输出最小修改 `u` | 线性 + 加性扰动；tube | **未显式处理延迟**（全文无 delay/latency） | 加性扰动 tube | **有**：安全备份律 `uB`（Def. II.1, p2） | 终端集 `Sf`（Assumption III.3, p3）；§IV-B 递归可行 | 无感知建模 | （原文） | 未见代码链接 |
| WZ-2021 | `uL`；PSF 输出最小修改 `u` | 非线性 + 数据驱动模型 + 状态/输入相关不确定性 | **未显式处理延迟**（全文无 delay/latency） | 概率鲁棒不确定界（可 GP p8） | **有**：安全备份计划（p4） | 终端集 `S^t`（Assumption 4.2, p6）；§4.2 递归可行 | 无感知建模 | NPSF/PSF 对比（p5） | 未见代码链接 |
| Kousik RTD | 轨迹参数 `k` | 跟踪误差有界（g 过近似 p3） | **不显式建模**；仅注明"assume no delay in braking actuation; if delay, T must be increased"（p6） | FRS（含跟踪误差，p3—4） | **有**：FRS 保证"总有可制动轨迹"（p2/p5） | FRS 时域内可刹停：T ≥ τplan + Dstop/vmax（p5—6） | 有限传感器视界 | NMPC、RRT（p1） | 未见代码链接 |
| Chen FaSTrack | 规划模型控制；TEB 控制器 | 失配 + 扰动由 TEB 界定 | **无**（原文未见 delay/latency） | TEB/tvTEB（HJ，p1—3） | **无 fail-safe 制动** | TEB 全落无障碍空间并到目标（Prop.1, p3） | 在线局部感知，按 TEB 膨胀 | 三种规划算法/模型对 | 未见代码链接 |
| ARMTD | 轨迹参数（关节级） | 关节级 JRS（机械臂） | 未见 delay 处理（**待核验**） | JRS → 工作空间 RS（p2—4） | **有**：fail-safe 制动到停（p1—2） | 轨迹含制动段至停 | 机械臂工作空间 | （原文） | **github.com/ramvasudevan/arm planning**（p7） |

### 2.2 Layered Safety 的 Eq.19 与 Theorem 1（贴原文，供核对）

- **Eq.19（§III-D 实时层）**：`∂h/∂t ≥ −αh + (1/ε)‖∇h‖²`——实时层用 **input-to-state safe (ISSf) CBF-QP**（`µs = argmin ‖µ − µp‖²`）过滤预测层命令 `µp`；其中 **`(1/ε)‖∇h‖²` 是"补偿 ROM 与全阶系统之间跟踪误差"的鲁棒项**（p5，引 [5],[33]）。**它是跟踪误差补偿，不是方向相关集合，也不是延迟项。**
- **Theorem 1（Full-Order System Safety，p5）**：**条件**保证——给定全阶系统 (20)，令 `h` 为 ROM (16) 的 CBF、`χ̇s` 为满足 **Eq.19**（某 `ε>0`）的安全速度，并存在**跟踪控制 Lyapunov 函数 (CLF) `V`** 满足 (21)—(22)（某 `β,λ>0`，跟踪控制器 `u=k(q,q̇,χ̇s,t)`），则**在全阶系统轨迹"足够好地跟踪 `χ̇s`"时**安全性得证。**前提是跟踪 CLF 与跟踪控制器存在且有效。**
- **Remark 1（Verification, p6）**：Theorem 1 **假设** `V` 与跟踪控制器存在；当 `V` 未知（低层为黑盒/学习模块）时，仍可通过把 Eq.19 的 `α, ε` 取小以"保守满足"——即**以保守性换取条件成立**。
- **Pareto 分析（§IV-A，p6—7）**：指标为 **`Joptimality`**（实际控制努力 / 已知障碍下的理想最小努力 `Jideal` 之比）与 **`Jrobustness`**（机动中 PSF `h0` 的最小值，即最小安全裕量）；用加权福利函数（`aO=aR=0.5`）求 Pareto 最优点。**结论**：多级（预测＋实时）架构处在 **Pareto 前沿**——优于单级：仅实时层=**低最优/高鲁棒**，仅预测（MPC）层=**高最优/低鲁棒**，多级介于两者之间（p6—7）。

## 3. B2 与 P 的差异如何落到可检验实验

以下实验**在 `docs/指导书/stage_logs/two/publication_tasks.md` §3 中定义，本阶段（P0—P4）尚未执行**：
- **N2**：同净空不同速度 / 同误差不同方向 / 同物理位置命令不同允许延迟；规则法、`B1`、`B2` 用**同一执行器与同一备份函数**；`passed`/`survives` **由测量计算**。
- **N3**：在 **B2 之上**加历史一致性分支筛选原型；同模型/噪声/方向/备份/计算上限，原型**不得读取真实延迟标签**；固定与允许时变延迟分开；**仅范围变小却漏真实分支判为失败**。

## 4. 结论：拟议贡献目前是否可区分（本轮更新）

- **主对照是 B2**；**P 的候选差异**限定为「**执行历史细化/分支保留 ＋ 计算预算**」。
- **五篇最近邻正文现已全部读完**（FaSTrack、RTD/Vaskov、ARMTD(相邻)、WZ-2018、WZ-2021、DDSF、Layered Safety）。综合判断：
  1. **延迟**：WZ-2018/WZ-2021/Layered Safety **均未显式处理执行器延迟/输入滞后**；RTD **不显式建模**（仅注明需增大时域）；FaSTrack **无**；**唯有 DDSF** 在 I/O 框架**已处理未知时延**。
  2. **历史**：**DDSF 已用 I/O 历史（`Tini`）**；本项目**不得**声称"首次处理执行历史/延迟"。
  3. **备份/终端**：WZ-2018/WZ-2021/DDSF/RTD/ARMTD **均含备份（含制动）或终端安全集/递归可行**；Layered Safety/FaSTrack 无独立 fail-safe 制动。本项目**不得**把"备份/终端集"当新增。
- **如实结论：现在仍不能宣称可区分。** 本项目候选差异目前**只落在**「**方向相关误差集合 ＋ 可执行制动备份 ＋ 位置接口辨识**」——其中方向相关集合与接口辨识尚未被最近邻覆盖（Layered 的 Eq.19 是跟踪误差补偿、非方向集合；DDSF 无方向集合、且假设无噪声 LTI），但**是否足以构成独立贡献，须由 N2/N3（未执行）判定**。**在 N2/N3 产出前不锁定创新**；若 N3 显示只是集合辨识/多模型鲁棒控制的等价实现，按 §9.2 明确写"无独立方法贡献"。

## 5. 待核验清单（v2，本轮更新）

1. **发表处**：**WZ-2018 = CDC 2018, pp. 7130–7135（已确认）**；**WZ-2021 = *Automatica* 129:109597（已确认）**；**FaSTrack = *IEEE Trans. Automatic Control* 66(12):5861–5876, 2021**，DOI [`10.1109/TAC.2021.3059838`](https://doi.org/10.1109/TAC.2021.3059838)（**用户提供 2026-09-24**，建议投稿前以出版方页复核）；**ARMTD 与 Layered Safety 发表处：待核验（用户已检索未找到，2026-09-24）**。WZ-2018 的 CDC DOI `10.1109/CDC.2018.8619829` 建议以出版方页确认。
2. **RTD 原始移动机器人工作**（Kousik 等）的准确题名/年份/发表处：**待核验（用户已检索未找到，2026-09-24）**。
3. **ARMTD 的延迟处理**：抽读未见 delay 处理，**待核验**。
4. **公开代码**：除 ARMTD（`github.com/ramvasudevan/arm planning`），其余六篇（DDSF/WZ-2018/WZ-2021/FaSTrack/RTD/Layered Safety）本次原文**均未见代码链接**（不代表不存在）：**待核验（用户已检索未找到，2026-09-24）**。
5. **Layered Safety 终端集**：原文未见独立"终端安全集"描述（仅终端代价与逐点 DCBF），如需递归可行性细节**待核验**。
## 6. N3 结论：历史细化与标准集合辨识/多模型鲁棒控制的关系（2026-09-25）

> 依据：`spec.md`「N3 执行历史细化的独立价值检查」、`tasks.md` Task 3；证据级别 **`model-level`**（合成/借用参数，非 PX4/SITL 实测）。
> 交付文档 [05_history_value_probe.md](05_history_value_probe.md)；代码/数据 `experiments/paper_v3/history_probe/`（`prototype.py`／`probe.py`／`run_probe.py`／`results/`）。
> 允许延迟区间取自 N1 辨识 `[0.18, 0.30] s`（来源 `identification/brake_hold_v3/identified_params.json`）。

**原型（同等约束）**：在 `B2` 之上，把允许延迟区间离散为 `N_b=7` 个分支（`n_L=9…15`），每分支用**同模型族**（`model.py`）、**同方向误差表示**（`σ_E`）、**同共同备份**（`backup.expand_candidate`）、**历史窗口 `W=40` 拍、声明残差界**；控制侧只用**已发布命令历史 + 实测位置**，**不读真实延迟标签**；满时域分支 rollout 数 ≤ `B2` 的 2 支等价预算。两种延迟制度分开：`fixed`（历史剪枝有效）与 `time_varying`（未来可任意跳变，**必须保留所有允许后继分支**）。

**实测（`results/summary.json`，判定由测量导出）**：

| 制度（有利低噪声档） | 保留分支数 | horizon 末端包络宽度 | 覆盖违例 step | 真值分支被排除 step | 安全比例 | 在 B2 预算内 |
|---|---|---|---|---|---|---|
| `fixed`（L=0.18/0.24/0.30）剪枝 | **1.52–1.74**（不剪枝=7.00） | **0.0216–0.0257**（不剪枝=0.0332） | **0** | **0** | 1.00 | 0.83–0.89 |
| `time_varying`（跳变）剪枝 | 1.57–1.60 | 0.0236–0.0257 | **79–83** | **35–39** | **0.60–0.62** | 0.86–0.88 |
| `time_varying` **保留全部分支**（sound） | **7.00** | 0.0332 | 0 | 0 | 1.00 | **0.00** |
| `B2` 基线（区间端点 2 支） | 2.00 | 0.0332 | 0 | —（见注） | 1.00 | **1.00** |

- **固定延迟（正例）**：历史剪枝把保留分支数与包络宽度显著缩小，且**本批未观测到**覆盖违例；——这正是**在线延迟辨识/自适应观测器**，缩小的正是「延迟不确定集」。
- **允许时变延迟（边界负例）**：构造「真实延迟先取区间一端 → 用历史把另一端分支剪掉 → 随后跳变到被剪掉的一端」。即使采用**可重新纳入的滑动窗口**版本，跳变后仍有 **35–39 拍**把真值分支排除、产生 **79–83 拍**覆盖违例（`missed_true_branch=true`）；**commit（确认即锁定）版本**则更差（真值分支排除 35–52 拍、覆盖违例 82–99 拍）。**结论：一般时变延迟下剪枝无效**，sound 处理必须**保留所有允许后继分支**——但此时保留 7 支，**超出 `B2` 的 2 支等价预算**（`within_b2_budget_ratio=0.00`）。
- **`B2` 基线**：延迟区间**端点 2 支**在全部场景**本批未观测到**覆盖违例且始终在预算内（2 支）→ 标准区间鲁棒处理已覆盖同一最坏情形，且比「保留全部分支」省算。
- **现实噪声（N1 量级）敏感性**：同一原型在 N1 观测噪声量级下，固定延迟剪枝几乎**不缩小**（保留分支数 5.5–6.7，`within_b2_budget_ratio` 0.00–0.03），与 N1「延迟只能给区间、不能给点」一致。

**等价性判断（回答 §4 的问题）**：**该「执行历史细化」等价于标准集合辨识 / 多模型鲁棒控制，无独立方法贡献。** 依据（均由测量导出）：（1）`fixed` 下剪枝=`在线延迟辨识`，所得即延迟不确定集，且未优于 `B2` 的区间端点处理（后者同样 0 覆盖违例且在预算内）；（2）`time_varying` 下剪枝**不成立**（`missed_true_branch=true` + 覆盖违例），sound 处理=**保留所有允许后继**（多模型/鲁棒集合），与 `B2` 的区间鲁棒同源，且保留全部分支**超预算**；（3）无任何测量显示存在**既保留全部后继又低于 `B2` 成本**的独立机制。→ `no_independent_method_contribution=true`。

**注**：表中 `B2` 的「真值分支被排除 step」仅在 `fixed_Lmid`（真值 0.24 不在端点 {0.18,0.30}）为非零——`B2` **不主张点识别**，其安全指标看**覆盖违例（0）**，此计数仅为均匀记录的技术量。

**措辞纪律**：零违例仅写「**本批未观测到**」；全部为 `model-level`；**未引用** `all_survive`/`fair_brake_compare` 作证据。

## 7. 修正声明：§6 时变覆盖结论的限定（2026-09-25，依 [`13`](13_acceptance_review.md) 复审）

> 本节为**追加**，不改写 §6 原结论；**§6 表中「`time_varying`（跳变）剪枝」与「`B2` 基线（端点 2 支）」的时变覆盖结论限定至原测试场景**（原单次跳变构造）。

- 依复审 §2 F3：7 支/2 支**恒定**分支包络**不能**未经证明充当逐拍可变时延的安全基线；原「端点 2 支覆盖一般时变延迟」「全保留 0 违例」**仅限原场景**。
- 一般时变延迟下的重算（逐拍所有后继可达集传播、三类转移规律、必过样例）见 [`15_time_varying_coverage.md`](15_time_varying_coverage.md) 与 [`13_review_checks.json`](13_review_checks.json)；`history_probe/results_v2/summary_v2.json:conclusion` 给出 `endpoints_cover_general_time_varying=false`、`retained_all_zero_violation_only_in_original_scenarios=true`、`old_proto_pruning_unsound_in_general_time_varying=true`。
- §6 由该布尔量得到的 `no_independent_method_contribution` 等价性结论**标 `superseded`**，不再作为一般时变延迟下的证明。
