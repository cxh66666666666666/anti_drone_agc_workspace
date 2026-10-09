# Tasks

> 范围：执行 [13_acceptance_review.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/13_acceptance_review.md) §3 的「下一轮最小工作包与止损点」，并修正其 F1—F4。
> 顺序纪律：**先修离线证据（Task 1—2）→ 重做时变覆盖（Task 3）→ 二次 R0（Task 4）→ 文档修正与核验（Task 5—6）**；不进入 R1、不做确认性实验、不先做 SITL。
> 硬约束：**不覆盖**既有 v1 结果与 N0/N1 产物；不改 `one/`、`src/`、`scripts/`；新结果写版本化目录；纯 Python 标准库、确定性可复算；真值只进评分侧；零违例只写「本批未观测到」。

- [x] Task 1（F1）: 唯一执行协议与「证书=执行」的全分支证书
  - [x] SubTask 1.1: 新建 `experiments/paper_v3/fair_compare/closed_loop_v2.py`：**闭环逐拍重决策**执行器——每拍观测→决策→只发布本拍命令（1 拍）→真值闭环推进 1 拍；转备份/不可认证则持续发布共同备份保持命令；候选每拍重建。
  - [x] SubTask 1.2: 新建 `experiments/paper_v3/fair_compare/certificate_v2.py`：**全分支证书**——在**全部允许延迟分支**与**方向误差集合**上做**全时域扫掠净空** + 终端条件；证书与执行使用**同一输入序列**；不得再用点延迟/仅末端判据。
  - [x] SubTask 1.3: 新建 `experiments/paper_v3/fair_compare/scenarios_v2.py`：加入有辨别力的样例——**可行初态**、**不可行初态**、**候选转备份**、**连续重决策**；三类控制变量场景沿用（同净空不同速度 / 同误差不同墙面方向 / 同命令不同允许延迟）。
  - [x] SubTask 1.4: 新建 `experiments/paper_v3/fair_compare/run_fair_compare_v2.py`：跑全场景×{`rule`,`B1`,`B2`}+负对照，结果写 `fair_compare/results_v2/`；判定**区分五类**（`certificate_unavailable`/裕量违例/物理碰撞/误拒/通行），全部**由测量计算**；被拒候选分叉标 `model-level`。
  - [x] SubTask 1.5: 验证：任一 trial 的**已执行命令序列**与**证书所用输入序列**一致（逐拍），且 `false_safe` 归因不再混入「候选持续发布」差异。

- [x] Task 2（F2）: 版本化辨识参数并用其重跑对照
  - [x] SubTask 2.1: 定义并**独立验证**可用于位置命令回放的模型：区分**速度通道**（`T≈0.30 s`）与**位置通道**（`pos_lag≈1.17 s`）参数，**不得互换**；在 N1 的**整次 run** 验证数据上检查多步位置/速度误差；给出「该模型是否可用」的判断。
  - [x] SubTask 2.2: 参数**版本化**（`fair_compare/results_v2/params_v2.json`）：含来源、区间、验证结论；延迟取辨识区间**下界/中点/上界**，真值延迟上界**不低于**辨识上界（`≥0.30 s`）。
  - [x] SubTask 2.3: 用 v2 参数重跑 Task 1 的对照；**并列**原合成参数（`T_lag=0.85`、`L∈[0.15,0.25]`）作**敏感性对照**。
  - [x] SubTask 2.4: 「已解决」判据收紧：须**不虚报安全且可行工况取得任务进展**；不可行工况只报**不可认证**。据此重算 `resolved` 类结论（不得沿用旧 `no_counterexample_requires_P`）。

- [x] Task 3（F3）: 时变延迟覆盖按允许转移规律
  - [x] SubTask 3.1: 新建 `experiments/paper_v3/history_probe/coverage_v2.py`：显式写出**延迟允许转移规律**；允许逐拍任意变化时按**逐拍所有后继可达集传播**（或给出**经证明**的连续外包）。
  - [x] SubTask 3.2: 以复审反例为**必过样例**（`13_review_checks.json` 的 `time_varying_delay_counterexample`：超出全恒定分支 `0.005077 m` > 容差 `0.003 m`）——sound 方案必须**覆盖**它。
  - [x] SubTask 3.3: 分别测试**对抗性逐拍切换**、**单次跳变**、**固定延迟**；记录覆盖违例、保留分支数/范围宽度、可行/通行比例、**同任务/同时域/同预算**的计时（`fair_compare` v2 与 `history_probe` v2 的代码路径差异须说明）。
  - [x] SubTask 3.4: 新建 `run_coverage_v2.py` 写 `history_probe/results_v2/`；明确**撤回/限定**旧结论（端点 2 支、全保留 0 违例不可外推为一般时变延迟）。
  - [x] SubTask 3.5: 验证：至少一个**必过样例**在 sound 方案下零覆盖违例；旧 `proto_*` 在该样例上的失败被复现并记录。

- [x] Task 4: 二次 R0 决策
  - [x] SubTask 4.1: 汇总 F1—F3 修正后的证据（含失败项与未解决项），结论链接原始证据字段。
  - [x] SubTask 4.2: 写 `docs/指导书/stage_logs/two/16_r0_decision_v2.md`：明确「**先完成离线修正**」顺序；SITL 闭环不作为第一步；给出三选一结论；若仍无稳定收益实例，**正式终止当前 `P` 投稿主张**；**不生成 R1**；证据不足时给最小实验与 ≤2 周预算。
  - [x] SubTask 4.3: 更新 `11_advisor_memo_v2.md`：纳入复审与二次结论、给出要问导师的问题（真实讨论状态，不编造认可）。

- [x] Task 5: 文档修正与可追溯（F1—F4）
  - [x] SubTask 5.1: 新建 `docs/指导书/stage_logs/two/14_offline_protocol_and_comparison.md`（F1/F2：协议、全分支证书、版本化参数、重跑结果与五类判定）。
  - [x] SubTask 5.2: 新建 `docs/指导书/stage_logs/two/15_time_varying_coverage.md`（F3：转移规律、可达集、必过样例与三类测试、同预算计时）。
  - [x] SubTask 5.3: 在 `04`/`05`/`06`/`08` **追加**修正/撤回声明：F4 改述为「能识别不可认证工况」；撤回「已解决反例」与 `no_counterexample_requires_P`；限定 `05`/`08` §6 的时变覆盖结论至原测试场景；旧结果为历史探索材料，指向 `14`/`15`/`16`。
  - [x] SubTask 5.4: 更新根目录 `docs/指导书/stage_logs/two/publication_tasks.md` 的相应状态句（如实反映「R0 复审不通过 → 已完成修正 → 二次决策」，不虚构通过）。

- [x] Task 6: 证据纪律与冻结核验（贯通）
  - [x] SubTask 6.1: 核验 `one/` 冻结 meta/trace 与 `src/`+`scripts/` 哈希未变；v1 结果文件哈希未变（历史保留）。
  - [x] SubTask 6.2: 核验确定性（新脚本重复运行一致，除时间戳字段）、无 ROS/Gazebo 进程；结论措辞与证据级别合规，未引用 `all_survive`/`fair_brake_compare`。

# Task Dependencies

- Task 2 依赖 Task 1（同一 v2 执行器/证书）。
- Task 3 依赖 Task 1 的预算与证书口径（同时域/同预算计时），但**代码可并行**。
- Task 4 依赖 Task 1—3 全部结果。
- Task 5 依赖 Task 1—4（文档汇总）；Task 6 最后统一核验。