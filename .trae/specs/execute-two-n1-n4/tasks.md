# Tasks

> 范围：`docs/指导书/stage_logs/two/publication_tasks.md` §3 的 **N1—N4**。
> 交付目录 `docs/指导书/stage_logs/two/`（N1→`03`、N2→`04`、N3→`05`、N4→`00`+`06`）；代码/数据用 `experiments/paper_v3/` 新目录。
> 硬约束：只读使用既有 `scripts/`、`protocols/`、`baselines/`；**不改** `one/` 冻结数据与 `src/` 生产源码；仿真 headless、只回收本会话进程；证据级别与措辞（零违例=「本批未观测到」）纪律见 `prediction_replay_format_v2.md`。

- [x] Task 1（N1）: 小规模辨识采集与独立验证
  - [x] SubTask 1.1: 先跑 **2 个烟雾样例**（新目录 `experiments/paper_v3/identification/brake_hold_v3/`），核验时钟/`publish_log` 实际发布/完整尾段；异常 run 标独立状态。确认后才批量。
  - [x] SubTask 1.2: 采集**最小矩阵 12 trial**＝纵/横 × 0.5/1.0 m/s × 3，逐 run 预先分配 `fit`/`calib`/`valid` 用途（写进 meta）。
  - [x] SubTask 1.3: 新建辨识脚本 `experiments/paper_v3/identification/identify_v3.py`：只用 `fit`/`calib` 拟合低阶模型与误差范围（时间常数 `T`、纯延迟区间 `[L_min,L_max]`），产出 `identified_params.json` 与可复算汇总；不可识别参数给区间/不可识别。
  - [x] SubTask 1.4: 在 `valid` 集做多步位置/速度预测与制动峰值/回摆检查；报告**命令速度 / 制动实际初速 / 巡航稳态**，解释 1.0 档约 1.10 m/s（不改标签）。
  - [x] SubTask 1.5: 比较 `Legacy`、正确位置接口低阶标称模型、`B2` 所需范围，给出各自参数/误差范围。
  - [x] SubTask 1.6: 写 `docs/指导书/stage_logs/two/03_identification_validation.md`（口径、原始消息/trace、元数据、可复算汇总、局限）。

- [x] Task 2（N2）: 可证伪的公平制动机制对照
  - [x] SubTask 2.1: 新建 `experiments/paper_v3/fair_compare/`：抽取**同一执行器**与**同一备份函数**（复用 `baselines/` 的模型与 `backup` 语义），保证规则法/`B1`/`B2` 的速度上限、命令约束、传感权限、任务时间一致。
  - [x] SubTask 2.2: 实现**规则法**（距离/速度相关制动距离，非错误常量阈值）与 `B1`/`B2` 三种决策。
  - [x] SubTask 2.3: 实现三类控制变量场景：同净空不同速度；同误差范围不同方向/墙面方向；同位置命令不同允许延迟；记录初态/命令/真值/控制侧可见信息。
  - [x] SubTask 2.4: 实现正/负/已解决对照（双方均安全、双方均不可行、标准方法已解决反例）；`passed`/`survives` 全由**测量**计算；未执行候选分叉标 `model-level`。
  - [x] SubTask 2.5: 一键重算脚本 + 逐 trial 结果 JSON（碰撞/裕量违例、通行/停滞、制动最小净空、输入与参数快照）。
  - [x] SubTask 2.6: 写 `docs/指导书/stage_logs/two/04_fair_comparison.md`：回答「标准方法已解决哪几项、还有什么未解决」；若三反例全被 `B2` 解决则明确停止当作 `P` 的难点。

- [x] Task 3（N3）: 执行历史细化的独立价值检查
  - [x] SubTask 3.1: 新建 `experiments/paper_v3/history_probe/`：在 `B2` 上实现**简单历史一致性分支筛选原型**（同模型族/噪声界/方向表示/备份/计算上限；**不读真实延迟标签**；不引入 RL）。
  - [x] SubTask 3.2: 固定延迟与允许时变延迟分开；任意跳变时保留所有允许后继分支；构造「过去观测看似排除、未来仍可能发生」的检查。
  - [x] SubTask 3.3: 记录保留分支数、位置范围宽度、覆盖违例、可行/通行比例、计算耗时；**仅范围变小却漏真实分支判失败**。
  - [x] SubTask 3.4: 等价性判断并更新最近邻表（`two/08` 增补小节）：是否等价于标准集合辨识/多模型鲁棒控制；等价则写「无独立方法贡献」。
  - [x] SubTask 3.5: 写 `docs/指导书/stage_logs/two/05_history_value_probe.md`：至少一个正/负或边界例子；给出继续/否决/证据不足依据。

- [x] Task 4（N4）: R0 复审与下轮决定
  - [x] SubTask 4.1: 汇总完成/失败/未解决项与实际耗时（含仿真占机），结论链接原始证据或原文。
  - [x] SubTask 4.2: 明确三选一结论（继续/改候选方法/停止改题）；证据不足则给最小实验与 ≤2 周追加预算。
  - [x] SubTask 4.3: 写 `docs/指导书/stage_logs/two/06_r0_decision.md` 与 `docs/指导书/stage_logs/two/00_README.md`（阶段索引与审查入口）。
  - [x] SubTask 4.4: 更新 `docs/指导书/stage_logs/two/11_advisor_memo_v2.md`（真实讨论状态，不编造认可；导师回复/学校认定非技术阻塞）。
  - [x] SubTask 4.5: 更新根目录 `docs/指导书/stage_logs/two/publication_tasks.md` §3 的 N0—N4 勾选状态（若本轮完成），并声明 R1 只有满足 R0 条件才发布。

- [x] Task 5: 证据纪律与冻结核验（贯通）
  - [x] SubTask 5.1: 核验 `one/` 冻结 meta/trace 与 `src/` 生产源码未被修改（哈希/差异）。
  - [x] SubTask 5.2: 核验仿真无残留进程；新数据只写新目录；结论措辞与证据级别合规，未引用 `all_survive`。

# Task Dependencies

- Task 1（N1）与 Task 2（N2）的**代码/文档可并行**（N2 模型级优先，参数用 N0 声明范围并标 `param_source`）。
- Task 3（N3）依赖 Task 2（需 N2 的「已解决/未解决」结论）；其原型代码可与 Task 1 并行，但文档结论在 N2 之后。
- Task 4（N4）依赖 Task 1—3 全部结果。
- Task 5 在 Task 1—4 完成后统一核验。