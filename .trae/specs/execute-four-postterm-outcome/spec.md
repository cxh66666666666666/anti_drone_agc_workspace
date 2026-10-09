# 终止后物理结局的可信评测（第四轮）规格

## Why

第三轮把研究题目改到「四旋翼避障仿真中终止后物理结局的可信评测」，并交付了尾段/接触探针报告 [`three/09`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/09_%E5%B0%BE%E6%AE%B5%E6%8E%A5%E8%A7%A6%E6%8E%A2%E9%92%88%E6%8A%A5%E5%91%8A.md)。但现有证据仍有明确边界：真实闭环 5 个回合全为 `timeout`，**没有** `shield_infeasible` / `sensor_invalid` / `perception_timeout` 尾段结果；contact 来自新增 bumper 碰撞盒，**不能称为原版机体全部接触的独立真值**；[`three/09`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/09_%E5%B0%BE%E6%AE%B5%E6%8E%A5%E8%A7%A6%E6%8E%A2%E9%92%88%E6%8A%A5%E5%91%8A.md) 自身存在待复核数值（空场景 contact 消息数报告写 2812，`recompute_all.json` 写 2814），且 `contact_tail` 的解释需按 `gazebo_env.py` 的「连续两帧」判据校正。

本轮（第四轮）要在 10 个工作日、主动投入 ≤70 h 内：复核并校正既有证据、标定接触源与派生模型影响、打通**真实提前终止原因**的尾段链路、补核最近邻并冻结评测协议、跑一次**小规模配对预试**，最后由预定判据裁决是否扩大为正式论文批次。**本轮不开发新安全过滤算法**，不把「区分终止/截断」「分列成功与碰撞」「使用 contact 传感器」当作创新，也不依据定向搜索场景推断真实飞行碰撞概率。

## What Changes

- **T1 现有报告与测量定义校正（第 1 天）**：用原始 `trace.csv`/`contact.csv`/`meta.json` 与 [`recompute.py`](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/tail_contact_probe/recompute.py) 复核 [`three/09`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/09_%E5%B0%BE%E6%AE%B5%E6%8E%A5%E8%A7%A6%E6%8E%A2%E9%92%88%E6%8A%A5%E5%91%8A.md) 全部关键数值；校正 `contact_tail` 解释；写明三类不同事件。
- **T2 接触源与派生模型标定（第 1—4 天）**：对照原版/派生 SDF；在空场景、可控障碍接近、地面起降核验接触源；比较原版与派生机体运动与首次相互作用；检查消息丢失与时钟对齐。
- **T3 打通真实提前终止尾段（第 2—5 天）**：定位 `shield_infeasible`/`sensor_invalid`/`perception_timeout` 触发条件；修正探针失效窗口未激活问题；每类实际可触发原因取得 ≥2 次独立闭环回合。
- **T4 补核最近邻并冻结评测协议（第 4—6 天）**：补 [`three/08`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/08_%E6%9C%80%E8%BF%91%E9%82%BB%E5%8E%9F%E6%96%87%E7%9F%A9%E9%98%B5.md) 缺口；核查目标期刊要求；在看预试结果前冻结协议。
- **T5 小规模配对预试与阶段裁决（第 6—10 天）**：仅在 T2/T3 门槛与 T4 冻结通过后跑 27 回合配对预试并复算，写三选一裁决。
- **版本与纪律**：新代码、场景、数据放入 `experiments/paper_v3/` 下**独立版本目录**（本轮取 `experiments/paper_v3/postterm_outcome/`）；旧 `three/09` 原始结果只读保留，不静默改写。

## Impact

- Affected specs（只读引用）：[`three/07`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/07_%E6%94%B9%E9%A2%98%E5%86%B3%E5%AE%9A_%E7%BB%88%E6%AD%A2%E5%90%8E%E7%89%A9%E7%90%86%E7%BB%93%E5%B1%80%E8%AF%84%E6%B5%8B.md)、[`three/08`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/08_%E6%9C%80%E8%BF%91%E9%82%BB%E5%8E%9F%E6%96%87%E7%9F%A9%E9%98%B5.md)、[`three/09`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/09_%E5%B0%BE%E6%AE%B5%E6%8E%A5%E8%A7%A6%E6%8E%A2%E9%92%88%E6%8A%A5%E5%91%8A.md)、[`第四轮任务.md`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/four/%E7%AC%AC%E5%9B%9B%E8%BD%AE%E4%BB%BB%E5%8A%A1.md)。
- Affected code（新增为主）：
  - 新增独立版本目录 `experiments/paper_v3/postterm_outcome/`（新代码、场景、原始日志、一键复算脚本）。
  - 探针必要修正：`experiments/paper_v3/tail_contact_probe/{tail_contact_probe.py, episode_tail_probe.py, recompute.py, make_contact_model.py}` 及 `scripts/start_sim.sh` 的 `SITL_SDF` 向后兼容接口；**改动须版本化、保留原版可复算入口**。
  - 只读复用（不修改）：`src/rl_obstacle_avoidance/scripts/gazebo_env.py`（运行时控制器与真值评分器保持隔离）、`experiments/stage3/scenarios/`（旧诊断场景只读引用）。
- 新增文档：`docs/指导书/stage_logs/four/{01_现有证据复核.md, 02_接触源标定.md, 03_真实终止链路.md, 04_冻结评测协议.md, 05_配对预试结果.md, 06_阶段裁决.md}`；必要时按修订流程更新 `three/09` 并注明修订。
- 不改：`src/` 运行时控制逻辑、`one/`、`two/` 全部原始结果与文档、`experiments/paper_v3/tail_contact_probe/results/` 既有原始数据（只读）。

## ADDED Requirements

### Requirement: T1 现有报告与测量定义校正

系统 SHALL 用原始日志复核既有报告全部关键数值，并明确三类事件的对象与证据等级。

#### Scenario: 关键数值可复算
- **WHEN** 复核 [`three/09`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/09_%E5%B0%BE%E6%AE%B5%E6%8E%A5%E8%A7%A6%E6%8E%A2%E9%92%88%E6%8A%A5%E5%91%8A.md)
- **THEN** 用原始 `trace.csv`/`contact.csv`/`meta.json` 与 [`recompute.py`](file:///home/ubuntu/anti_drone_agc_workspace/experiments/paper_v3/tail_contact_probe/recompute.py) 复核全部关键数值，**特别核对空场景 contact 消息数（报告 2812 vs `recompute_all.json` 2814）与终止时实际净空**
- **AND** 保留原始数据、不静默改写；差异以**差异表**列出（报告值、复算值、来源、处置）

#### Scenario: `contact_tail` 解释校正
- **WHEN** 解释 `contact_tail` 的净空负值
- **THEN** 说明 [`gazebo_env.py`](file:///home/ubuntu/anti_drone_agc_workspace/src/rl_obstacle_avoidance/scripts/gazebo_env.py#L1878-L1891) 的 `is_done()` 要求**连续两次**净空达阈值才记碰撞；逐拍估计净空曾为负**只能证明代理量出现负值**
- **AND** 若声称「旧判据会记为碰撞」，须按**实际控制周期重放**判定并检查特征新鲜度，否则降格措辞

#### Scenario: 三类事件对象区分
- **WHEN** 描述结局事件
- **THEN** 明确区分：① 估计位姿几何代理越界；② Gazebo 真值几何相交；③ 指定碰撞体的 Gazebo contact
- **AND** 写明接触消息条数**不是**独立碰撞回合数；新增 bumper 参与物理求解，当前接触证据的对象是**派生机体的 bumper**

#### Scenario: 交付与通过
- **WHEN** 交付 T1
- **THEN** 产出 `four/01_现有证据复核.md`，含复算命令与差异表；必要时修订 `three/09` 并注明修订
- **AND** 报告数值与可复算日志一致，所有结论的事件对象和证据等级清楚

### Requirement: T2 接触源与派生模型标定

系统 SHALL 标定接触源可测的碰撞体、误检/漏检边界与派生模型影响。

#### Scenario: 原版/派生 SDF 对照
- **WHEN** 对照模型
- **THEN** 列出原有机体碰撞体与新增 bumper 的尺寸、质量、位置和接触覆盖范围
- **AND** 查明是否可从 Gazebo 原生碰撞事件或原有碰撞体读取 contact；若采用新传感器，说明它**只覆盖哪些碰撞体**

#### Scenario: 事件源核验
- **WHEN** 在空场景、可控障碍接近、地面起降核验
- **THEN** 确认接触源持续有消息、空 `states` 语义正确、地面与障碍接触可区分
- **AND** 记录首次接触时间、碰撞体名称、接触点/法向或可获得的力信息，并与同拍姿态、真值几何比较

#### Scenario: 派生模型影响界定
- **WHEN** 用相同初态和命令比较原版与派生机体
- **THEN** 量化 bumper 是否改变接触时刻、轨迹或尾段外扩
- **AND** 若影响无法界定，则将结果限定为「派生模型」，不外推到原版机体

#### Scenario: 丢失与时钟对齐
- **WHEN** 检查消息丢失与时钟对齐
- **THEN** 把局部 contact 话题失联、时钟停滞、复位前残留消息分别标记为**缺失/基础设施失败**，不能写成无接触
- **AND** 记录检测规则及一例故障样例

#### Scenario: 交付与通过
- **WHEN** 交付 T2
- **THEN** 产出 `four/02_接触源标定.md`，附版本化代码、原始日志和复算入口
- **AND** 能明确说出 contact 测到的碰撞体、误检/漏检边界与派生模型影响；**若达不到，接触结局不得进入正式主指标**

### Requirement: T3 真实提前终止尾段链路打通

系统 SHALL 在真实闭环中取得目标提前终止原因的可追溯尾段。

#### Scenario: 触发条件定位
- **WHEN** 定位终止原因
- **THEN** 按 [`three/07`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/07_%E6%94%B9%E9%A2%98%E5%86%B3%E5%AE%9A_%E7%BB%88%E6%AD%A2%E5%90%8E%E7%89%A9%E7%90%86%E7%BB%93%E5%B1%80%E8%AF%84%E6%B5%8B.md) 事件清单，分别定位 `shield_infeasible`、`sensor_invalid`、`perception_timeout` 的代码触发条件
- **AND** 核对 `sensor_invalid` 与 `perception_timeout` 的标签来源，避免把**注入故障标签**当作实际 `done_reason`

#### Scenario: 探针失效窗口修正
- **WHEN** 修正探针
- **THEN** 修正失效窗口未激活的问题，记录 `trajectory_phase_s`、窗口起止、特征新鲜度、过滤器候选集和最终 `done_reason`
- **AND** **只对探针/测试配置做必要改动**；保留运行时控制器与真值评分器隔离

#### Scenario: 独立闭环回合
- **WHEN** 采集尾段
- **THEN** 对每类实际可触发的提前终止原因，至少取得 **2 次独立闭环回合**
- **AND** 每次从 `done` 前持续记录至统一尾段上限或预定停稳条件，期间**不先复位**；保存接触有/无、真值/估计净空、命令、速度和故障标记；碰撞与无碰撞**均保留**

#### Scenario: 无法触发时如实记录
- **WHEN** 某类原因在限定工时内仍无法触发
- **THEN** 记录尝试参数、运行日志和失败原因
- **AND** **不通过**重命名 `timeout` 或人为净空截断补齐；新场景与种子另建清单，**旧诊断批次不进入配对结果**

#### Scenario: 交付与通过
- **WHEN** 交付 T3
- **THEN** 产出 `four/03_真实终止链路.md`，附原始回合及可复算摘要
- **AND** 至少**两类**目标提前终止原因各有 2 次独立真实闭环尾段，覆盖**过滤器失效**与**传感/感知失效**两种机制；`done`、尾段开始、复位时刻可追溯；三类覆盖情况逐类如实报告
- **AND** 未达到时只保留链路测试，**不进入 T5 的配对预试**

### Requirement: T4 最近邻补核与评测协议冻结

系统 SHALL 补核文献缺口并在看预试结果前冻结协议。

#### Scenario: 补核最近邻缺口
- **WHEN** 补核 [`three/08`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/three/08_%E6%9C%80%E8%BF%91%E9%82%BB%E5%8E%9F%E6%96%87%E7%9F%A9%E9%98%B5.md) 明列缺口
- **THEN** 补核四旋翼几何碰撞终止的近邻原文、机器人/自动驾驶「终止后继续推进仿真」的工作、删失与配对评测统计方法
- **AND** 注明检索范围、原文/摘要深度、版本和未取得项目；**不得由「本轮未检索到」推出研究空白**

#### Scenario: 期刊要求核查
- **WHEN** 核查目标期刊/会议
- **THEN** 核查至少一个拟投期刊或会议对**评测/实证论文**的当前范围与要求
- **AND** 无法访问原文政策时标**未核实**，不以旧转述代替；导师反馈只在实际取得时记录

#### Scenario: 协议冻结
- **WHEN** 在看配对预试结果前冻结协议版本
- **THEN** 固定场景家族与种子、B2/A1/P 或其他可比且事先固定的方法、任务期限、终止后命令、固定观察上限、停稳定义、复位规则、失联规则、模型/控制器版本及所有排除条件
- **AND** 旧口径须按其**真实控制周期与两次确认逻辑**重算；完整口径分别报**真值几何相交**和**已验证的 contact**

#### Scenario: 试验单位与判据预定
- **WHEN** 定义统计口径
- **THEN** 试验单位为**整回合**，配对差值以同一回合的「立即结算」与「继续观察」两个读数计算；算法比较按相同场景/种子配对
- **AND** 按场景家族聚类给不确定性区间，分别列任务完成、软件提前终止、尾段接触、尾段超时、传感缺失和复位失败；**时间步不作独立样本**
- **AND** 预定主要判据与最小有意义差异，**不能见结果后更换主指标**

#### Scenario: 交付与通过
- **WHEN** 交付 T4
- **THEN** 产出 `four/04_冻结评测协议.md`、新场景清单与版本指纹
- **AND** 另一人能按协议独立计算每个结局及分母
- **AND** 接触源有效性和真实终止链路未通过时，**只冻结草案**，不启动正式批次

### Requirement: T5 小规模配对预试与阶段裁决

系统 SHALL 仅在门槛通过后运行配对预试，并按预定判据给出三选一裁决。

#### Scenario: 预试规模与前置
- **WHEN** 运行预试
- **THEN** **仅在 T2/T3 最低门槛通过、T4 协议已冻结后**运行；建议初始规模 3 个代表场景家族 × 3 个方法 × 每家族 3 个配对种子 = **27 个回合**，实际以冻结协议为准
- **AND** 先做每方法 1 个独立烟雾回合确认时钟、接触与复位；**烟雾回合不计入 27 个预试回合**

#### Scenario: 逐回合复算与配对
- **WHEN** 从原始日志复算
- **THEN** 逐回合重算原结算口径、含尾段真值几何结局、含尾段 contact 结局及首次事件时间
- **AND** 显示两口径的**配对差值、分母、缺失与不一致矩阵**；对尾段观察上限做**预定敏感性分析**；观察窗结束仍在运动的回合标**未停稳/右删失**，不视为安全

#### Scenario: 可比性与基础设施失败
- **WHEN** 检查可比性
- **THEN** 检查各方法在同场景/种子下初态和故障注入是否可比；把明显的环境复位失败或配置漂移列为**基础设施失败**并保留原始记录
- **AND** 预试样本只估计**可测性、差异方向和后续样本规模**，不用 27 个回合直接宣称普遍发生率或稳定排序翻转

#### Scenario: 三选一裁决
- **WHEN** 写裁决
- **THEN** 三选一：① 测量可信且真实尾段差异值得扩大 → 制定正式批次及样本量依据；② 机制成立但当前证据/统计精度不足 → 指定一个有上限的补充验证；③ 差异不稳定、只来自 bumper 改模/实现错误，或近邻已覆盖同等结论 → 停止论文主张并作为平台质量改进归档

#### Scenario: 交付与通过
- **WHEN** 交付 T5
- **THEN** 产出 `four/05_配对预试结果.md`、`four/06_阶段裁决.md`、原始日志及一键复算脚本
- **AND** 每个数字有**回合级来源**；负结果、缺失、未停稳和不确定性完整呈现；是否进入正式实验由**预定判据**决定，不由任务勾选数决定

### Requirement: 证据纪律、版本隔离与预算（贯通）

系统 SHALL 保持证据级别、版本隔离与预算纪律。

#### Scenario: 版本隔离与只读保留
- **WHEN** 产生任何新代码/场景/数据
- **THEN** 放入 `experiments/paper_v3/postterm_outcome/` 的独立版本目录；旧 `three/09` 原始结果**只读保留**，不静默改写
- **AND** 修订既有报告时保留原文并**注明修订**

#### Scenario: 运行可追溯
- **WHEN** 每次运行
- **THEN** 保存代码哈希、模型 SDF 哈希、实际命令与参数、场景/种子、时钟、控制侧信息权限、终止和复位原因
- **AND** 运行摘要由原始日志重算，不以手写表格为唯一证据

#### Scenario: 预算与运行纪律
- **WHEN** 统计投入
- **THEN** 主动投入上限：T1 6 h、T2 14 h、T3 14 h、T4 12 h、T5 18 h、复核余量 6 h，**总上限 70 h / 10 个工作日**；仿真占机时间另记
- **AND** 全程**无头**运行，不留 ROS/Gazebo GUI 残留进程

#### Scenario: 措辞纪律
- **WHEN** 产出结论
- **THEN** 只写「**本批未观测到**」；不虚报安全；未达成即如实记负结果
- **AND** 论文题目与投稿价值在 `four/06` 根据本轮结果**重新裁决**，本规格不预设贡献成立
