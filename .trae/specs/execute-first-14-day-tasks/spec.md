# 执行指导书「前 14 天具体任务」Spec

## Why

[publication_plan.md](../../../docs/指导书/publication_plan.md) §9.3 定义了 R0 阶段的前 14 天具体任务，但当前仓库只交付了指导书与重审报告，**这 14 天的实际工作尚未执行**（见 [publication_progress.md](../../../docs/指导书/publication_progress.md)：R0 未验收）。这 14 天要回答一个具体问题：

> 旧的距离/瞬时动作模型在哪种可重复条件下失败，标准方法能解决多少，剩下的具体问题是什么。

本 spec 把这 14 天拆成可验证的工作项并产出证据，供 R0 决策与后续六周竞争性预研使用。**不重写旧结果、不删除任何目录、不修改旧清单**。

## What Changes

- 新增冻结基线产物：旧系统关键源码、checkpoint 配置、实验索引的内容哈希清单（机器可复核）。
- 新增 days 1–3 审计日志：33/38 维与 masked/raw 差异、唯一真实执行链路图（物理量+延迟）、`_safety_obstacles()` 真值权限复查、`cross_validation_ok` 拆分。
- 新增制动/保持命令规范 + 无障碍低速纵横向短段实测（新测量，替代旧 `a_brake` p10）。
- 新增 B1/B2 最小模型级实现设计（基于安全过滤综述与 ≥2 篇最近邻原文）。
- 新增三个机制反例实验（同位置不同速度 / 同误差不同方向 / 同路径不同延迟），并给规则法与标准预测法同等制动能力。
- 新增未来预测保存、输入回放、扫掠净空评估格式定义。
- 新增第一次研究决策交付：最近邻差异表 v1、接口选择依据、前 3 个月工作量预算、投稿要求核验记录、导师讨论备忘录。
- **BREAKING**：无。本 spec 不修改现有控制/训练代码，不改变旧实验语义。

## Impact

- 影响能力（文档化能力，非新算法）：R0 基线审计、制动辨识、机制验证、研究决策。
- 影响代码：
  - 新增（不改旧文件）：`experiments/paper_v3/` 下的协议、脚本与运行数据。
  - 新增：`docs/指导书/stage_logs/one/` 下的日志与清单。
  - 只读引用（不改写）：`src/rl_obstacle_avoidance/scripts/gazebo_env.py`、`safety_policy.py`、`safety_filter.py`、`experiments/stage3/run_stage3_batch.py`、`experiments/stage3_task5_cross_validate.py`、`experiments/stage3/build_audit_index.py`、`scripts/start_sim.sh`、`src/lqr_trajectory_tracker/src/lqr_tracker_node.cpp`、`src/drone_launch_controller/src/drone_launch_controller_node.cpp`。

## ADDED Requirements

### Requirement: 冻结基线清单

系统 SHALL 生成一份机器可复核的冻结清单，记录旧系统关键源码内容、checkpoint 配置与模型、实验索引、场景清单的 SHA-256，以及当前 git revision。

#### Scenario: 生成冻结清单
- **WHEN** 运行冻结脚本
- **THEN** 写出 `docs/指导书/stage_logs/one/frozen_manifest.json`，含每个文件的相对路径、SHA-256、字节数，以及 `git_revision`、生成时间戳（UTC）

#### Scenario: 可复现
- **WHEN** 在工作区内容未变时重复运行冻结脚本
- **THEN** 所有 SHA-256 与首次一致（幂等），且脚本对缺失文件记录 `null` 而非臆造哈希

### Requirement: 33/38 维与 masked/raw 差异说明

系统 SHALL 在 days 1–3 日志中列出状态维度历史（33 维 vs 38 维）与动作接口档（masked vs raw）的差异，并给出来源行号与对应 checkpoint 配置，说明旧 33 维 checkpoint 不能默认加载到当前 38 维环境。

#### Scenario: 维度差异可追溯
- **WHEN** 阅读差距说明
- **THEN** 能定位到 `observation_space_dim` 与 `PATH_FEATURES`/`num_sectors` 的源码行，以及 `training_runs/*/config.json` 中的 `observation_dim` 字段

#### Scenario: 接口档差异可追溯
- **WHEN** 阅读接口档说明
- **THEN** 能定位 `action_interface` 分支（raw 档距离掩码退出裁决、只保留侧向迟滞），并列出各模式（b2/b2m/a1/p）的接口映射

### Requirement: 唯一真实执行链路图

系统 SHALL 交付一张（且仅一张）真实执行链路文档，逐层标明物理量、话题/频率与延迟，且结论必须来自代码而非旧文档。

#### Scenario: 链路可核对
- **WHEN** 阅读链路文档
- **THEN** 每一层（任务路径 → 候选参考 → 控制器 → 位置设定点 → PX4 内环 → 机体运动）都给出物理量、发布频率、源码/配置文件位置与实测或已冻结的延迟数值

### Requirement: `_safety_obstacles()` 真值权限复查

系统 SHALL 复查 `_safety_obstacles()` 是否、以及在何种条件下使用场景真值几何，并明确其可否支持「纯 LiDAR、未知环境」主张。

#### Scenario: 真值权限结论明确
- **WHEN** 阅读复查结论
- **THEN** 明确指出该函数读取 `scenario_obstacles_typed`/障碍中心与尺寸，说明其属于「真值辅助几何诊断配置」，并给出可作为新主实验观测隔离要求的结论

### Requirement: `cross_validation_ok` 拆分

系统 SHALL 把现有 `experiments/stage3_task5_cross_validate.py` 的判据拆分为（a）实现一致性检查与（b）尚缺的未来预测验证，且不重写旧结果文件。

#### Scenario: 判据归类
- **WHEN** 阅读拆分说明
- **THEN** `sound`/`false_safe`/同量残差双侧界被归为「实现一致性检查」，并具体说明缺失的「对齐预测发出时刻、实际执行输入与未来观测」验证是什么、为何现有判据不能替代它

#### Scenario: 旧结果不被改写
- **WHEN** 完成拆分
- **THEN** 旧 `cross_validation.json`、`audit_index.*` 与既有分析产物内容保持不变（仅新增说明文档）

### Requirement: 制动/保持命令规范与短段实测

系统 SHALL 定义基于当前接口的制动/保持命令规范，并在无障碍、低速条件下做纵横向短段实测；必须持续记录到物理停止，不用旧 `a_brake` p10 顶替新测量。

#### Scenario: 规范可执行
- **WHEN** 阅读制动/保持规范
- **THEN** 明确制动触发条件、制动的参考/命令定义、保持策略、终止判据（速度阈值与稳定窗口）与尾段观察时长

#### Scenario: 实测产物完整
- **WHEN** 完成短段实测
- **THEN** 产出逐次运行的原始日志（时间戳、参考、实际位姿/速度、MAVROS 与 Gazebo 两路）与汇总报告，报告给出减速过程、最大漂移、命令生效延迟、失败形态，并标注样本量与适用范围

#### Scenario: 仿真不可用时的诚实处理
- **WHEN** 仿真在给定尝试次数内无法启动或无法产出有效数据
- **THEN** 在日志中如实记录阻塞原因、命令行与原始报错，标记该项「未完成」，不伪造数据、不以旧 p10 代替

### Requirement: B1/B2 最小模型级实现设计

系统 SHALL 阅读安全过滤综述与至少两篇最近邻原文，产出 B1（标准标称预测安全过滤）与 B2（延迟增广、方向相关鲁棒预测安全过滤）的最小模型级实现设计。

#### Scenario: 设计可实施
- **WHEN** 阅读设计文档
- **THEN** B1/B2 各自给出状态/输入定义、预测模型、误差集合表示、约束判据、备份与终端的接口约定，且明确两者只差「误差集合表示/明确的新增步骤」这一项

#### Scenario: 来源可核验
- **WHEN** 阅读设计文档
- **THEN** 关键外部结论标注原始 DOI/arXiv 版本，未获取原文的结论标「待核验」

### Requirement: 三个机制反例

系统 SHALL 完成三个机制反例（同位置不同速度或命令历史 / 同误差模长不同方向 / 同路径不同执行滞后），并给规则法与标准预测法同等制动能力，确认问题仍存在。

#### Scenario: 反例可复现
- **WHEN** 阅读机制反例报告
- **THEN** 每个反例给出初态、控制/误差设置、运行命令、原始产物路径与可复现结论

#### Scenario: 物理事实与差模型分离
- **WHEN** 阅读结论
- **THEN** 明确区分「哪些是物理事实」与「哪些只是某个差模型造成的」，并说明基线是否被赋予了制动能力

### Requirement: 预测保存/回放/扫掠净空评估格式

系统 SHALL 定义未来预测保存、实际输入回放与扫掠净空评估的统一格式，使时刻 k 的预测可在「已知且相同的实际输入序列」下被检验。

#### Scenario: 格式可落地
- **WHEN** 阅读格式定义
- **THEN** 明确需保存字段（模型版本、实际初态、候选/备份输入序列、预测范围、真实未来状态）、回放对齐规则与扫掠净空下界的判定方法

### Requirement: 第一次研究决策交付

系统 SHALL 交付第一次研究决策所需材料：最近邻差异表 v1、接口选择依据、前 3 个月工作量预算，并核验投稿要求来源与日期。

#### Scenario: 差异表可用
- **WHEN** 阅读最近邻差异表 v1
- **THEN** 至少覆盖 DDSF（arXiv:2309.00189）、Layered Safety（arXiv:2603.00338）、Wabersich–Zeilinger 2021、Kousik RTD、Chen FaSTrack，并按「相比方法 X 的假设 A，本项目处理限制 B；新步骤 C 带来性质或收益 D，代价 E；实验 F 能单独检验它」的形式填写或标「填不出」

#### Scenario: 投稿要求核验有记录
- **WHEN** 阅读核验记录
- **THEN** 记录访问目标 URL、日期、结果（含 HTTP 403 等失败）与本地格式文件作为退路，不含未经验证的现行效力断言

#### Scenario: 两周结论可展示
- **WHEN** 阅读收尾日志
- **THEN** 能展示两周要求的结论：旧模型在哪种可重复条件下失败、标准方法能解决多少、剩下的具体问题是什么；未完成项如实标注

### Requirement: 导师讨论备忘录

系统 SHALL 产出给导师的一页备忘录，含「问题、证据、当前结论、下次能否证伪」四项。

#### Scenario: 备忘录结构完整
- **WHEN** 阅读备忘录
- **THEN** 问题、证据、当前结论、下次能否证伪四项齐备，且证据引用本 spec 产出的具体文件