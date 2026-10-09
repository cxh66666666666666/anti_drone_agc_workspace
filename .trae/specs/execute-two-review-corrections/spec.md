# 阶段 two 复审修正规格（F1–F4 修证与二次 R0）

## Why

独立复审 [13_acceptance_review.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/13_acceptance_review.md) 判定 N1 可保留、**N2/N3 证据链不通过、R0 暂不通过**，并给出四处必须纠正的实质缺陷（F1—F4）与下一轮最小工作包。本规格执行该工作包：**先修离线证据与执行协议**，再用辨识后的接口模型重跑对照、重做时变延迟覆盖检查，最后**再做一次 R0 决策**（不进入 R1、不做确认性实验）。

## What Changes

- **F1 修正**：固定唯一执行协议——**每拍重新观测、重新决策**；候选执行与证书基于**同一输入序列**；证书须在**全部允许延迟/误差分支**上验证（不再只判 1 拍候选 + 点延迟 + 仅终端）。
- **F2 修正**：定义并**独立验证**可用于位置命令回放的模型，把 N1 辨识结果**版本化**为参数输入；测试延迟区间**端点与内部值**；原合成参数仅作**敏感性对照**。
- **F3 修正**：明确延迟**允许转移规律**；逐拍可任意变化时改用**逐拍所有后继的可达集传播**（或经证明的连续外包），并以复审反例为**必过样例**。
- **F4 修正**：把「不伪造证书」改述为「**能识别不可认证工况**」，撤回「已解决反例」与由之推出的 `no_counterexample_requires_P` 等过强表述。
- **版本化**：旧 55 trial/探针结果**保留为历史探索材料**（不覆盖）；新结果为版本化目录 + 新文档 `14`/`15`/`16`，并在 `04`/`05`/`06`/`08` 追加**修正/撤回声明**。
- 顺序纪律：**先完成离线修正**，不把 SITL 闭环验证作为第一步；本轮不生成 R1。

## Impact

- Affected specs（只读引用）：[13_acceptance_review.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/13_acceptance_review.md)、[13_review_checks.json](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/13_review_checks.json)、[execution_contract_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/execution_contract_v2.md)、[prediction_replay_format_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/prediction_replay_format_v2.md)、[01_design_corrections.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/01_design_corrections.md) §1、`02`/`03`/`04`/`05`/`06`/`08`。
- Affected code（新增为主，**不改**既有 v1 与 N0/N1 产物）：
  - `experiments/paper_v3/fair_compare/{certificate_v2.py, closed_loop_v2.py, scenarios_v2.py, run_fair_compare_v2.py, results_v2/}`
  - `experiments/paper_v3/history_probe/{coverage_v2.py, run_coverage_v2.py, results_v2/}`
  - 文档：新增 `14_offline_protocol_and_comparison.md`、`15_time_varying_coverage.md`、`16_r0_decision_v2.md`；**追加**修正声明到 `04`/`05`/`06`/`08`。
- 不改：`one/`、`src/`、`scripts/`、`baselines/`、`identification/`、`fair_compare/results/`（v1 历史）、`history_probe/results/`（v1 历史）。

## ADDED Requirements

### Requirement: F1 唯一执行协议与「证书=执行」一致

系统 SHALL 采用每拍重新决策的闭环执行协议，并使证书与执行基于同一输入序列。

#### Scenario: 闭环逐拍重决策
- **WHEN** 执行任一方法的 trial
- **THEN** 每个控制拍 `k`：用**控制侧可得**状态观测 → 重新决策 → **只发布本拍选定命令**（1 拍）→ 经真值闭环（延迟/扰动注入）推进 1 拍，重复 `n_exec_steps` 拍
- **AND** 若本拍判为转备份/不可认证，则**持续发布**共同备份保持命令；**禁止**把「候选一拍 + 备份的预测」与「持续发布候选 300 拍」混用

#### Scenario: 证书与执行同序列、全分支
- **WHEN** 生成候选或备份证书
- **THEN** 证书须在与执行**相同的输入序列**上评估，且覆盖**全部允许延迟分支**与**方向误差集合**（逐分支全时域**扫掠净空** + 终端条件）
- **AND** 点延迟点估计、只判末端（无扫掠净空）的旧证书**不得**再作为 `B1`/`B2` 的通过依据

#### Scenario: 结果分类可区分
- **WHEN** 汇总 trial
- **THEN** 明确区分 `certificate_unavailable`、**裕量违例**、**物理碰撞**、**误拒**、**通行**五类，且每类**由测量计算**
- **AND** 提供有辨别力的样例：可行初态、不可行初态、候选转备份、连续重决策

### Requirement: F2 版本化辨识参数与用其重跑对照

系统 SHALL 用辨识并独立验证的位置命令模型重跑控制变量对照，并保留合成参数作敏感性对照。

#### Scenario: 模型选择与独立验证
- **WHEN** 选定用于位置命令回放的模型
- **THEN** 明确区分**速度通道**与**位置通道**参数（不得互换），给出该模型是否可用的判断；在**整次 run** 的独立验证数据上检查**多步位置/速度误差**
- **AND** 参数**版本化**（含来源与区间），并保留 N1 小样本「非统计边界」声明

#### Scenario: 端点与内部值
- **WHEN** 运行对照
- **THEN** 延迟取辨识区间**下界/中点/上界**（及必要时内部值）；真值延迟上界**不得**低于辨识区间上界；原合成参数（`T_lag=0.85`、`L∈[0.15,0.25]`）作为**敏感性对照**并列报告

#### Scenario: 「已解决」判据更严
- **WHEN** 宣称某标准方法解决了某反例
- **THEN** 必须**同时**满足：不虚报安全（无 `false_safe`）**且**在**可行工况取得任务进展**（通行）
- **AND** 不可行工况只报「不可认证」，**不得**记为「已解决」

### Requirement: F3 时变延迟覆盖按允许转移规律

系统 SHALL 按显式的延迟转移规律做覆盖检查，且逐拍可变时使用可达集传播。

#### Scenario: 显式转移规律与可达集
- **WHEN** 定义延迟不确定性
- **THEN** 显式写出**允许转移规律**；若允许**逐拍任意变化**，则按**逐拍所有可能后继**传播形成可达集合（或给出**经证明**的连续外包）
- **AND** 「恒定延迟分支的包络」**不得**未经证明充当逐拍可变时延的安全基线

#### Scenario: 必过样例与三类测试
- **WHEN** 运行覆盖检查
- **THEN** 以复审反例（第 60 拍起 50 拍内逐拍选取允许延迟、超出全恒定分支 `0.005077 m` > 容差 `0.003 m`）作为**必过样例**
- **AND** 分别测试**对抗性逐拍切换**、**单次跳变**、**固定延迟**；任何称「可靠」的方案须覆盖允许切换与已声明扰动

#### Scenario: 同预算计时
- **WHEN** 报告计算代价
- **THEN** 在**同一任务、同一预测时域、同一计算预算**下计时，并说明代码路径差异；不得据一次不同路径计时判定效率

### Requirement: F4 与 F1–F3 的文档修正可追溯

系统 SHALL 在既有文档中追加修正/撤回声明，并使旧结论与新结论可追溯。

#### Scenario: 撤回与改述
- **WHEN** 更新 `04`/`05`/`06`/`08`
- **THEN** 把 `rule` 在 CE1/CE3 **不可认证**记为「**能识别不可认证工况**」，撤回「已解决反例」措辞及由之推出的 `no_counterexample_requires_P` 结论
- **AND** 撤回或限定 `05`/`08` §6 的「端点 2 支覆盖一般时变延迟」「无独立贡献已由测量证明」至**原测试场景**，并指向 `15`
- **AND** 旧 55 trial/探针结果**保留为历史探索材料**，不覆盖、不删除，新结果写入版本化目录

### Requirement: 二次 R0 决策（离线修正优先）

系统 SHALL 在完成 F1—F3 修正后重做 R0 决策，且不进入 R1。

#### Scenario: 顺序与门槛
- **WHEN** 给出二次决策
- **THEN** 明确「**先完成离线修正**」的顺序；SITL 闭环验证**不作为第一步**，仅在仍有具体机制差异时另行设计
- **AND** 若修正后仍无「标准强基线解决不了、且新机制在同覆盖/同预算下有稳定收益」的实例，**正式终止当前 `P` 的投稿主张**；需要第二创新点时先做一页最近邻差异表与可证伪小算例
- **AND** **不生成 R1**；证据不足时给最小实验与 ≤2 周追加预算

### Requirement: 证据纪律（贯通）

系统 SHALL 保持证据级别与措辞纪律。

#### Scenario: 级别、真值与措辞
- **WHEN** 产出任何结论
- **THEN** 标注 `model-level`/`sim-observation`；真值只进评分侧；零违例只写「本批未观测到」
- **AND** 不引用 `all_survive`/`fair_brake_compare` 作证据；不改 `one/` 冻结数据与 `src/` 生产源码；结果确定性可复算