# N1–N4 执行规格（辨识验证、公平对照、历史细化价值、R0 决策）

## Why

阶段 two 的前置补正 P0—P4 与 N0（标准 B1/B2 模型级原型）已完成。本规格执行 §3 剩余新任务 **N1—N4**，回答本阶段唯一研究问题：**在同一可辨识执行系统内，给规则法与标准预测法相同的制动能力与观测信息后，B2 能解决哪些问题？剩余问题是否真的需要执行历史相关的细化，而不是旧模型错误造成？**

## What Changes

- **N1**：在选定接口（直接 PX4 位置设定点台架）上补采小规模辨识数据（2 烟雾 + 12 trial），按**整次运行**预先划分拟合/校准/验证；辨识低阶标称模型与延迟范围；在验证集做多步位置/速度与制动峰值检查；比较 `Legacy`、正确位置接口的低阶标称模型与 `B2` 所需范围。
- **N2**：重做**可证伪**的公平制动机制对照。规则法、`B1`、`B2` 使用**同一执行器与同一备份函数**，同样速度上限/命令约束/传感权限/任务时间；三类控制变量场景 + 正/负/已解决对照；`passed`/`survives` 一律由**测量**计算。
- **N3**：在 `B2` 之上做一个**简单历史一致性分支筛选原型**（不引入 RL），固定延迟与允许时变延迟分开，专门构造「过去看似排除、未来仍可能发生」的检查；记录分支数/范围宽度/覆盖违例/通行比例/计算耗时；**判断该细化是否等价于标准集合辨识/多模型鲁棒控制**。
- **N4**：汇总证据，给出**继续 / 改候选方法 / 停止该机制**三选一结论（证据不足则给最小实验与 ≤2 周追加预算），并更新导师备忘录。
- 全部交付写入 `docs/指导书/stage_logs/two/`（N1→`03`、N2→`04`、N3→`05`、N4→`00`+`06`）；代码/数据用 `experiments/paper_v3/` 新目录，**不改** `one/` 冻结数据与 `src/` 生产源码。

## Impact

- Affected specs（只读引用，不修改）：
  - 接口契约与共同备份 [execution_contract_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/execution_contract_v2.md)
  - 采集/指标口径 v2 [brake_hold_protocol_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/brake_hold_protocol_v2.md)、[07_measurement_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/07_measurement_v2.md)
  - 预测/回放/评分口径 [prediction_replay_format_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/prediction_replay_format_v2.md)
  - 方法标签 [01_design_corrections.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/01_design_corrections.md) §1；最近邻表 [08_nearest_neighbor_table_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/08_nearest_neighbor_table_v2.md)
  - N0 原型 [02_standard_baselines.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/02_standard_baselines.md) 与 `experiments/paper_v3/baselines/`
- Affected code（新增为主）：
  - `experiments/paper_v3/identification/brake_hold_v3/`（新采集 run + 辨识脚本 + 汇总）
  - `experiments/paper_v3/fair_compare/`（N2 对照代码与逐 trial 结果）
  - `experiments/paper_v3/history_probe/`（N3 原型与结果）
  - `scripts/start_sim.sh`、`brake_hold_test_v2.py` 等**既有文件只读使用，不修改**（如需新行为，新增脚本/参数，不改旧文件语义）。

## ADDED Requirements

### Requirement: N1 小规模辨识与独立验证

系统 SHALL 在选定接口上补采小规模辨识数据，并按整次运行划分拟合/校准/验证，产出可复算的辨识与验证结论。

#### Scenario: 先烟雾后批量
- **WHEN** 开始 N1 采集
- **THEN** 先完成 **2 个烟雾样例**，验证时钟（`/clock`、`use_sim_time`）、实际发布（`publish_log.csv` 序号/值/时刻）与**完整尾段**；确认后再批量采集
- **AND** 不以失效数据继续调模型；异常 run 标独立状态（`clock_jump`/`clock_stall`/`drop_frame`/超时）

#### Scenario: 最小矩阵与用途预分配
- **WHEN** 批量采集
- **THEN** 采集 **纵/横 2 方向 × 0.5/1.0 m/s 2 档 × 3 次＝12 个新 trial**，三次按**整次运行**预先分配到 `fit`/`calib`/`valid` 用途（不按时间行随机分割同一次运行）
- **AND** 命令过程同时提供**斜坡**（巡航参考匀速前进）与**制动激励**（设定点回退到实测位置）

#### Scenario: 报告命令速度、制动初速与巡航稳态
- **WHEN** 汇总 N1 结果
- **THEN** 同时报告**命令速度**、**制动实际初速**与**巡航稳态情况**；对原 1.0 m/s 档约 1.10 m/s 的现象给出解释（尚未稳态/参考时序/响应超调），**不得**改标签写成「1.0 m/s 实测」

#### Scenario: 只在拟合/校准集定模型
- **WHEN** 辨识模型与误差范围
- **THEN** 仅在 `fit`/`calib` 集拟合；`valid` 集只用于**多步位置/速度预测**与**制动峰值/回摆**检查
- **AND** 无法识别的延迟参数给**区间**或**不可识别**结论，**不硬拟合一个数**

#### Scenario: 与 Legacy / B2 范围对比
- **WHEN** 比较方法所需模型
- **THEN** 对照 `Legacy`（恒速动作映射）、正确位置接口的**低阶标称模型**与 `B2` 所需范围，给出各自参数与误差范围

### Requirement: N2 可证伪的公平制动机制对照

系统 SHALL 用同一执行器与同一备份函数对规则法、`B1`、`B2` 做控制变量对照，结论由测量计算。

#### Scenario: 三类控制变量场景
- **WHEN** 构造 N2 场景
- **THEN** 至少含：**同净空不同速度**、**同误差范围不同方向/墙面方向**、**同物理位置命令但不同允许延迟**；每个场景记录初态、命令、真值与**控制侧可见信息**

#### Scenario: 同一执行器与同一备份
- **WHEN** 运行任一方法
- **THEN** 规则法、`B1`、`B2` 使用**同一执行器、同一备份函数**、同样的速度上限/命令约束/传感权限/任务时间
- **AND** 规则法**允许使用速度相关的制动距离**，不得被限定为错误常量阈值

#### Scenario: 度量而非标签
- **WHEN** 汇报结果
- **THEN** 输出碰撞/裕量违例、是否通行/停滞、制动最小净空；任何 `passed`/`survives` **必须由测量计算**而来
- **AND** 对未执行动作的分叉试验明确标 `evidence_level=model-level`，不冒充实际飞行观测

#### Scenario: 正负与已解决对照
- **WHEN** 汇总结论
- **THEN** 含**双方均安全**、**双方均不可行**、**标准方法已解决反例**的对照；能回答「标准方法已解决哪几项、还有什么未解决」
- **AND** 若不要求三反例全部存活；若全部被 `B2` 解决，明确停止把这三项当 `P` 的未解决难点

### Requirement: N3 执行历史细化的独立价值检查

系统 SHALL 在 `B2` 上实现简单历史一致性分支筛选原型，并判定其是否有独立于标准集合辨识的价值。

#### Scenario: 原型受同等约束
- **WHEN** 实现原型
- **THEN** 与 `B2` **同模型族、噪声界、方向表示、备份与计算上限**；**不得读取真实延迟标签**；不引入 RL

#### Scenario: 固定与允许时变延迟分开
- **WHEN** 测试延迟处理
- **THEN** 固定延迟与允许时变延迟**分开**报告；未来可任意跳变时必须保留**所有允许后继分支**
- **AND** 专门构造一次「过去观测看似排除、未来仍可能发生」的延迟变化检查

#### Scenario: 指标与失败判据
- **WHEN** 记录结果
- **THEN** 记录保留分支数、位置范围宽度、覆盖违例、可行/通行比例与**计算耗时**
- **AND** **仅范围变小却漏掉真实分支**判为**失败**

#### Scenario: 等价性判断
- **WHEN** 给出结论
- **THEN** 更新最近邻表：判定该细化**是否等价于标准集合辨识/多模型鲁棒控制**；若等价，明确写**无独立方法贡献**；若有差异，列具体步骤、可证明性质与待检验代价
- **AND** 给出**继续 / 否决 / 证据不足**的依据；若有可靠否决证据，直接提交否决分析，不强行造新场景

### Requirement: N4 R0 复审与下轮决定

系统 SHALL 汇总证据并给出明确的 R0 决策。

#### Scenario: 证据汇总
- **WHEN** 撰写 N4
- **THEN** 汇总完成项、失败项、未解决问题与实际耗时；所有结论链接到**原始证据或原文**

#### Scenario: 三选一结论
- **WHEN** 给出决策
- **THEN** 在「继续当前具体机制 / 问题成立但改候选方法 / 停止该机制并讨论改题」中**明确选一**；证据不足时给出一个能改变决策的**最小实验**与最多 **≤2 周/42—70 h** 追加预算
- **AND** 只有「继续」且 R0 条件满足时才生成 R1（扩大辨识/局部感知隔离）任务；**不因目录齐全自动过关**

#### Scenario: 交付物
- **WHEN** 完成 N4
- **THEN** 交付 `two/00_README.md`（阶段索引）与 `two/06_r0_decision.md`，并更新导师备忘录（保留「未讨论/已讨论日期与反馈」真实状态，不编造认可）

### Requirement: 证据纪律（贯通 N1–N4）

系统 SHALL 对全部结论保持证据等级与措辞纪律。

#### Scenario: 证据级别与真值权限
- **WHEN** 任何结论产出
- **THEN** 标注证据级别（`model-level` / `sim-observation`）；真值只进评分侧；零违例只写「本批未观测到」，不写「已证明安全/一般安全」
- **AND** 不引用 `all_survive`/`fair_brake_compare` 作证据；仿真全程 headless、只回收本会话进程

#### Scenario: 冻结与可复算
- **WHEN** 运行任何实验
- **THEN** 不改 `one/` 冻结的 meta/trace 与 `src/` 生产源码；新数据写入新目录；保留命令/代码哈希/参数/trial 身份/终止原因，且结果可复算