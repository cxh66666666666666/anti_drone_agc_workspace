# 阶段 two 前置补正（P0—P4）Spec

## Why

`docs/指导书/stage_logs/one/09_acceptance_review.md` 对阶段 one 的验收结论为**部分通过、尚未通过 R0**，并提出 F1—F5 五项必须前置补正。根目录任务书 [publication_tasks.md](../../../docs/指导书/stage_logs/two/publication_tasks.md) §2 已把这些补正落成 P0—P4。**在这些补正的必要部分完成前，不得开始 §3 的 N0—N4 新任务。**

本 spec 只覆盖 P0—P4；N0—N4 不在本次范围。

## What Changes

- **P0** 新增 `experiments/paper_v3/protocols/execution_contract_v2.md`（下一阶段唯一主接口契约）与 `docs/指导书/stage_logs/two/01_design_corrections.md`（方法标签与接口结论固化）。
- **P1** 新增制动采集器 v2 与协议 v2：统一仿真时钟、区分源/接收时间戳、指标全部由保存日志重算、修正 `max_drift_m`/`tail_drift_m` 语义、只管理本会话进程；并补 **1 个小样例** 走通全链路。
- **P2** 在 `two/01_design_corrections.md` 记录 CE1/CE2/CE3 的替代解释，并把 `fair_brake_compare.json` 的 `all_survive=true` 撤回为**不构成公平对照证据**。
- **P3** 新增 `experiments/paper_v3/protocols/prediction_replay_format_v2.md`（不覆盖 v1），并给出三个离线核对样例。
- **P4** 修正方法标签与研究决策、核对 Wabersich–Zeilinger 文献身份、归档官网来源、更新导师备忘录。
- **BREAKING**：无。旧 one 的原始 data/meta/trace 冻结不改；按任务书 §4，允许在**带版本指纹**的前提下新增 v2 版本文件，不覆盖 v1。

## Impact

- 新增目录：`docs/指导书/stage_logs/two/`、`experiments/paper_v3/identification/brake_hold_v2/`、`experiments/paper_v3/protocols/*_v2.md`。
- 新增/修订代码：`experiments/paper_v3/scripts/brake_hold_test_v2.py`、重算脚本与分析脚本（新文件，保留 v1）。
- 只读引用：`docs/指导书/stage_logs/one/*`（除按 P4 需新增 two/ 下的修订版，不改 one 原文）、`docs/指导书/stage_logs/two/publication_tasks.md`、`docs/指导书/publication_plan.md`、`src/rl_obstacle_avoidance/scripts/*`、`scripts/start_sim.sh`。
- 受影响能力：执行接口契约、制动辨识采集与指标、机制反例证据等级、未来预测回放协议、研究决策与来源材料。
- 命名避让：任务书 §3 的 N0—N4 已占用 `docs/指导书/stage_logs/two/00_README.md` 与 `02`—`06`；本轮 P 交付物使用 `01` 与 `07`—`12`，避免与下一阶段文件名冲突。

## ADDED Requirements

### Requirement: 统一执行接口契约（P0）

系统 SHALL 交付一份下一阶段的唯一主接口契约，明确选定接口，并列出该接口的全部量与单位；不得混用单位或隐含真值输入。

#### Scenario: 接口契约完整
- **WHEN** 阅读 `execution_contract_v2.md`
- **THEN** 逐项给出：任务参考、实际发布的位置命令、速度反馈、历史命令、坐标系、单位、发布者、发布频率；并声明选定的研究接口，以及它是否替换了原 LQR 外环

#### Scenario: 不宣称已验证原组合
- **WHEN** 阅读接口结论
- **THEN** 表述为「本阶段选用的研究接口」；若选用直接位置设定点台架，须明确它**替换**了原 LQR 外环且需重新辨识，**不得**宣称新台架已验证原 LQR＋PX4 组合系统

#### Scenario: 单位不混用
- **WHEN** 检查契约中的速度类量
- **THEN** 不得把 `max_offset_xy=1.4 m` 当作 `1.4 m/s`；每个量标注物理单位与来源

### Requirement: 共同备份与接管规则（P0）

系统 SHALL 定义所有比较方法共用的备份语义，使「候选一拍 → 共同备份 → 保持」可从一个实际发布命令递推。

#### Scenario: 备份语义完整
- **WHEN** 阅读契约的备份定义
- **THEN** 明确：何时把参考改到什么值、如何限幅、如何持续发布（满足 OFFBOARD 频率）、何时宣布任务失败、物理观察何时结束、多发布者的接管/释放规则

#### Scenario: 可递推
- **WHEN** 给定一个实际发布的位置命令
- **THEN** 能据此推导下一步预测与对应备份，且不存在未定义的状态转移或隐含真值输入

### Requirement: 方法标签固化（P0）

系统 SHALL 在 `docs/指导书/stage_logs/two/01_design_corrections.md` 固化方法标签，纠正旧文档的 B1/B2 关系描述。

#### Scenario: 标签定义明确
- **WHEN** 阅读标签定义
- **THEN** `Legacy` = 旧恒速动作映射；`B1` = 与实际位置命令对应的标称闭环预测＋共同备份；`B2` = 延迟增广、方向相关鲁棒预测＋同一备份；`P` = 拟议的执行历史细化

#### Scenario: 不再称只差一项
- **WHEN** 比较 B1 与 B2
- **THEN** 文档允许二者在延迟建模与不确定性处理上不同，**不得**再把它们描述为「只差一个 κ 或一个标量集合」

### Requirement: 制动采集与指标口径修正（P1）

系统 SHALL 交付制动采集器 v2 与协议 v2，修正时基、时间戳、指标定义与进程管理，且不改写旧 one 的原始 meta/trace。

#### Scenario: 时钟与时基
- **WHEN** 运行采集器 v2
- **THEN** 统一 `/use_sim_time=true` 并记录 `/clock`，另记单调墙钟用于 watchdog 与计算耗时；启动失败、时钟跳变或停滞有**独立状态**，且不得把墙钟秒当动力学秒

#### Scenario: 时间戳与发布序号
- **WHEN** 检查保存日志
- **THEN** 含实际 setpoint 的发布序号、值与时间；MAVROS 位姿/速度的**源时间戳与接收时间分别记录**；Gazebo 无 header 时记录接收时刻及对应仿真时刻并声明同步精度

#### Scenario: 指标由日志重算
- **WHEN** 由保存日志重算指标
- **THEN** 能分别给出沿向最大外扩、水平最大位移、终态净偏移、停稳确认时刻、尾段最大偏移与最大速度；`max_drift_m`/`tail_drift_m` 的错名或单点计算已修复，状态分类使用修正后的定义

#### Scenario: 延迟命名与辨识分离
- **WHEN** 阅读指标定义
- **THEN** 「速度首次下降 5% 的时间」命名为**阈值响应时间**；纯延迟通过单独模型辨识估计，不直接用该时间替代

#### Scenario: 真值仅用于评分
- **WHEN** 检查控制/备份状态转换
- **THEN** 其只使用可获得的观测与已声明阈值；若台架仍用真值决定记录窗口，须明确标注为**评分配置**，不得挪用为部署控制

#### Scenario: 进程管理与可追溯
- **WHEN** 采集器启动/结束
- **THEN** 只管理本次会话进程（不使用全局 `pkill -9 -f` 清场），并保存采集脚本内容哈希、实际参数、协议版本、发布者与运行终止原因

#### Scenario: 新增小样例可复算
- **WHEN** 用 v2 采集 1 个小样例
- **THEN** 能由原始日志重算全部关键指标；触发/发布/停止确认/尾段时刻可区分；并有一个丢帧或超时样例被正确标记

### Requirement: 机制反例证据等级更正（P2）

系统 SHALL 在 `two/01_design_corrections.md` 更正三个反例与公平比较的证据等级。

#### Scenario: 替代解释逐项列出
- **WHEN** 阅读反例更正
- **THEN** CE1 被说明为「实际无障碍轨迹＋假设净空评分」；CE2 的已知误差例子与椭圆集合例子**分开**表述；CE3 被说明为合成一阶速度模型，不是实际位置指令回放

#### Scenario: 撤回公平对照结论
- **WHEN** 检查 `fair_brake_compare.json` 的使用
- **THEN** `all_survive=true` 被标为**不构成公平对照证据**；历史文件保留可追溯，新索引与新摘要不再引用其为实验通过

#### Scenario: 碰撞与裕量区分
- **WHEN** 阅读 CE2 结论
- **THEN** 区分物理碰撞（几何包络相交）与未满足设计裕量；CE2 的 0.10 m 净空**不得**写成碰撞

### Requirement: 未来预测/回放协议 v2（P3）

系统 SHALL 新建 `experiments/paper_v3/protocols/prediction_replay_format_v2.md`（不覆盖 v1），修正 v1 的五类实质问题。

#### Scenario: 三种任务分离
- **WHEN** 阅读协议
- **THEN** 明确区分「同初态/同输入的候选复算」「实际未来发布输入驱动的离线预测」「未执行候选的模型分叉」；仅当前两者输入与初态相同时才要求预测数值一致

#### Scenario: 预测只用可得信息
- **WHEN** 定义预测侧输入
- **THEN** 只使用可得状态与估计参数/范围；真实注入延迟、扰动与障碍几何放独立评分侧；不得要求预测参数与真值参数强行一致

#### Scenario: 日志不混淆生效时刻
- **WHEN** 检查日志字段
- **THEN** 「已发布命令历史」与「估计的可能执行时刻/分支」分开存储；不得把 `t_pub+L_hat` 写成已知真实生效时间

#### Scenario: 判据独立定义
- **WHEN** 阅读指标定义
- **THEN** 位置集合覆盖、逐时域误差、净空下界违例、输入复算误差各自独立定义；零违例只称「本批未观测到」，不作为一般安全证明

#### Scenario: 几何检查有前提
- **WHEN** 定义采样间/几何检查
- **THEN** 平面墙/凸多边形优先用显式分离约束或保守相交测试；用支持函数时给出方向与适用前提；用离散采样时给出全段速度/加速度界或曲线外包，不得用起点速度替代全段上界

#### Scenario: 三个离线核对样例
- **WHEN** 阅读协议附带的样例
- **THEN** 至少给出「中途改命令」「采样间曲线接近障碍」「未知延迟落在允许范围边缘」三个核对样例，并各自说明该比较什么、不能推出什么

### Requirement: 研究决策与来源材料校正（P4）

系统 SHALL 在 `docs/指导书/stage_logs/two/` 下产出修订版差异表与导师备忘录，并校正文献身份与来源归档。

#### Scenario: 最近邻主对照明确
- **WHEN** 阅读修订版差异表与备忘录
- **THEN** 以 **B2** 为最近邻主对照，P 的候选差异限定为执行历史细化/分支保留与计算预算；不得退回「比 B1 或旧 DQN 好即可」

#### Scenario: 决策节奏复位
- **WHEN** 阅读研究决策
- **THEN** R0 决策仍在**六周预研预算结束时**进行，不顺延到第三个月；本轮不做确认性测试；预算与每周 21—35 h（四周 84—140 h）一致

#### Scenario: 文献身份与链接
- **WHEN** 检查 Wabersich–Zeilinger 引用
- **THEN** 核对 2018 线性安全认证与 2021 非线性预测过滤的身份，纠正混引，并修复原文相对链接

#### Scenario: 来源只归档已获取内容
- **WHEN** 阅读官网/政策来源记录
- **THEN** 只存档已获取内容并标日期，尚缺项列为待办；若沿用对 AIGC 政策的转述，须核对原句与适用范围并标注「交导师评估」，不得只凭摘要代替官方条文

#### Scenario: 导师状态真实
- **WHEN** 阅读导师备忘录
- **THEN** 保留「未讨论/已讨论日期与反馈」的真实状态；不发送任何代用户邮件、不编造导师认可；导师回复与学校认定不作为技术阻塞项