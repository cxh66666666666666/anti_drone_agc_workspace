# N0 标准预测基线模型级原型 Spec

## Why

阶段 two 需要在**统一执行接口**上给出标准预测基线 **B1/B2** 的**模型级可运行原型**，作为后续 N2（公平对照）与 N3（执行历史细化是否独立有价值）的对照与判定依据。当前只有设计文档（`one/03_b1_b2_design.md`）与旧的共用恒速预测（含「偏移上限当速度」的单位错误），既缺少与位置设定点接口一致的闭环模型，也缺少可证伪的运行入口与基础结果。本规格把 N0 落成**纯 Python、无 ROS、可离线复算**的模型级原型，明确标注合成参数，不作为 PX4 实测。

## What Changes

- 新增**低阶定高二维闭环模型**（位置命令输入；状态含位置/速度/命令历史；合成参数明确标注）。
- 实现**共同备份函数**（所有方法共用，语义对齐 `execution_contract_v2.md` §4）与三个方法：**B1**（标称点预测）、**B2**（延迟增广＋方向相关鲁棒预测）、**P**（仅接口，不引入新网络或大规模搜索）。
- 每个候选显式展开「**候选一拍 → 共同备份 → 保持**」；保存上次有效备份，超时/不可行时执行剩余备份；初态不支持备份时报告**证书不可用**。
- 给出**可计算的终端条件**（线性模型下的简单局部保持集 + 数值检查）；声明短时低速只是经验停稳观察，**不等于**已证明不变集。
- 新增**可运行入口**与四个验收场景（开阔接受、危险替换且备份确实执行、缩短时域暴露失败、破坏终端条件暴露失败）；结论按**实际回放轨迹度量**给出，**不得仅检查返回值标签**。
- 交付 `docs/指导书/stage_logs/two/02_standard_baselines.md`：模型/单位表、文献精读要点、方法差异、可运行入口与基础结果。

## Impact

- Affected specs（只读引用，不修改）：
  - 方法标签 `Legacy` / `B1` / `B2` / `P`：[01_design_corrections.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/01_design_corrections.md) §1
  - 共同备份与接管规则：[execution_contract_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/execution_contract_v2.md) §3—§5
  - 预测/回放/评分口径：[prediction_replay_format_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/protocols/prediction_replay_format_v2.md)
  - 最近邻差异表（文献已读结论）：[08_nearest_neighbor_table_v2.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/two/08_nearest_neighbor_table_v2.md)
- Affected code（新增，不改动既有）：
  - 新增 `experiments/paper_v3/baselines/`（模型、过滤器、共同备份、运行入口、结果 JSON）。
  - **不改** `src/rl_obstacle_avoidance/scripts/safety_filter.py` 等生产源码；**不改** `one/` 冻结数据；**不新增** PX4 仿真运行。
- 证据级别：全部为 `model-level`；参数为**合成/借用值并明确标注**，不得表述为 PX4 实测。

## ADDED Requirements

### Requirement: 低阶定高二维闭环模型

系统 SHALL 提供一个定高二维、位置命令输入的低阶闭环模型，用于 B1/B2 的预测与评分的共同被控对象描述。

#### Scenario: 状态与单位完备
- **WHEN** 读取模型定义
- **THEN** 状态至少含位置 `p_k∈R²` 与速度 `v_k∈R²`，输入为位置命令 `cmd_k∈R²`（map/ENU，m）；模型为「纯延迟 `n_L` 拍 + 一阶滞后 `α=1−exp(−dt/T)`」，速度由位置差分自洽给出
- **AND** 每个量、参数均给出名称、物理量、单位、来源（合成/借用/待辨识），无单位混用，无未定义状态转移，无隐含真值输入

#### Scenario: 合成参数明确标注
- **WHEN** 文档或结果中出现模型参数
- **THEN** 明确标注为**合成/借用值**（例如 `T`、`L`、方向误差半轴），并声明**不是** PX4 实测；不得把位置偏移（m）当作速度（m/s）

### Requirement: 共同备份与候选展开

系统 SHALL 令 Legacy/B1/B2/P **共用同一备份函数**，并把每个候选显式展开为「候选一拍 → 共同备份 → 保持」。

#### Scenario: 备份触发与目标
- **WHEN** 某候选在预测时域内不满足净空约束，或全部候选不安全
- **THEN** 触发共同备份：参考置为**当前实测水平位置** `p_k`，持续发布该保持设定点（对齐 `execution_contract_v2.md` §4.2）
- **AND** 展开序列为：候选一拍 `k→k+1`，其后 `k+1→N` 走共同备份，末段保持

#### Scenario: 备份保存与超时降级
- **WHEN** 本拍求解成功
- **THEN** 保存本拍有效备份（命令、目标、触发拍索引）
- **AND** 若本拍超时或不可行，则执行**上次有效备份的剩余部分**，并记录 `used_last_valid_backup=true`

#### Scenario: 初态不支持备份
- **WHEN** 即使从 `p_k` 立即施加共同备份，在时域 `N` 内也到不了终端集
- **THEN** 报告 `certificate_unavailable`（证书不可用），不伪造「已放行/已安全」标签

### Requirement: B1 标称基线

系统 SHALL 实现 B1：与实际位置命令对应的**标称闭环点预测 + 共同备份**，误差集合退化为 `E={0}`，使用声明的延迟点估计 `L_hat`（不增广）。

#### Scenario: 单点名义预测
- **WHEN** 给定候选 `a`、初态与 `L_hat`
- **THEN** 生成单条名义位置序列 `P_nom(k,a)`，逐点检查净空，末端用终端条件判定；备份与 B2 **完全相同**

### Requirement: B2 延迟增广、方向相关鲁棒基线

系统 SHALL 实现 B2：在 B1 之上作**结构性**改动——**延迟增广状态 + 延迟区间 `[L_min,L_max]` + 方向相关误差集合及其集合传播**，备份函数与 B1 相同。

#### Scenario: 延迟增广与集合传播
- **WHEN** 构造 B2 预测
- **THEN** 状态含已发布命令历史，延迟取区间（不取单点），误差集合以支持函数 `σ_E(n)` 表示并沿时域传播得可达集 `R(a)`
- **AND** 鲁棒判据为 `h(p) − σ_E(−∇h) ≥ margin`（方向相关，非各向同性标量膨胀；最坏方向为 `−∇h`，见 `prediction_replay_format_v2.md` §5.2）；`σ_E(−∇h)` 与 `σ_E(∇h)` 方向核对，不默认相等

#### Scenario: 与 B1 的差异为结构性
- **WHEN** 对比 B1 与 B2
- **THEN** 差异**不**表述为「只差一个 κ 或一个标量集合」；中心模型本身即为「延迟增广 + 方向相关鲁棒」，且备份共用

### Requirement: P 仅保留接口

系统 SHALL 只保留 P 的接口（函数签名与占位实现），本阶段**不**引入新网络、不做大规模搜索。

#### Scenario: 接口占位
- **WHEN** 调用 P 接口
- **THEN** 返回明确标记的占位结果或抛出未实现，且在文档中标明 `P = 仅接口`，不冒充已实现方法

### Requirement: 可计算终端条件

系统 SHALL 给出实际可计算的终端条件，并用数值检查给出是否满足，而不主张已证明不变集。

#### Scenario: 终端集与数值检查
- **WHEN** 评估终端条件
- **THEN** 使用线性模型下的**简单局部保持集** `Ω = {净空 ≥ margin 且 |v| ≤ v_stop}`，并由备份展开序列**数值**判定是否在时域内落入 `Ω`
- **AND** 文档声明：短时低速只是经验停稳观察，**不等于**已证明不变集；不变性主张仅限线性模型下的局部范围

### Requirement: 可运行入口与验收场景

系统 SHALL 提供纯 Python（无 ROS、无第三方依赖）的可运行入口，执行四个验收场景，并**按实际回放轨迹度量**给出结论。

#### Scenario: 开阔与危险场景
- **WHEN** 运行开阔场景
- **THEN** B1/B2 均**接受**正常候选（未被替换）
- **WHEN** 运行危险场景（候选方向近障碍）
- **THEN** 危险候选被**替换**为共同备份，且**备份确实被执行**（回放的已执行命令序列等于备份保持设定点，轨迹进入 `Ω`）

#### Scenario: 故意破坏暴露失败
- **WHEN** 缩短预测时域（`N` 过小）或破坏终端条件（禁用 `Ω` 检查）
- **THEN** 能暴露失败：缩短时域时原可接受场景报 `certificate_unavailable`；破坏终端条件时出现**实际净空/裕量违例**或无法停稳的候选被错误放行
- **AND** 结论来自对回放轨迹的**独立度量**（实际净空、违例计数、终态速度），**不得仅检查 `passed`/`safe` 等返回值标签**

#### Scenario: 结果可复算
- **WHEN** 重复运行入口
- **THEN** 输出确定一致的结果 JSON（含初态、命令、模型版本/参数、已执行命令序列、四类度量、`evidence_level=model-level`）

### Requirement: 交付文档

系统 SHALL 交付 `docs/指导书/stage_logs/two/02_standard_baselines.md`，含模型/单位表、文献精读要点、方法差异、可运行入口与基础结果。

#### Scenario: 文档内容完备
- **WHEN** 阅读交付文档
- **THEN** 至少含：模型/单位表（含量来源与「合成/借用」标注）；文献要点（DDSF 安全集/优化问题/假设/局限、Layered Safety Eq.19/Theorem 1、WZ-2021 模型/终端/递归可行，RTD/FaSTrack 仅与本路线重合段落）；Legacy/B1/B2/P 差异；候选→备份→保持与备份保存/超时/证书不可用；可计算终端条件；运行命令与基础结果；证据级别声明