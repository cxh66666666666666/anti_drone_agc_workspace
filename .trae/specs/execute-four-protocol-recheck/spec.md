# 终止后测量协议修复与固定清单小复核 Spec

> 依据 [`four/07_独立复审与下一步.md`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/four/07_%E7%8B%AC%E7%AB%8B%E5%A4%8D%E5%AE%A1%E4%B8%8E%E4%B8%8B%E4%B8%80%E6%AD%A5.md) §三 的 1—5 步。第四轮 T1—T5 已由既有 spec `execute-four-postterm-outcome` 完成；本 spec 是复审后的**协议修复 + 最小复核 + 范围决定**。

## Why

T5 的 **5/21** 是在「当前 bumper 模型、终止后无共同控制接管」下得到的探索值，混入了终止处理差异，不能当作 [`four/04`](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E6%8C%87%E5%AF%BC%E4%B9%A6/stage_logs/four/04_%E5%86%BB%E7%BB%93%E8%AF%84%E6%B5%8B%E5%8D%8F%E8%AE%AE.md) 冻结协议 v4.0 的主结果。复审要求**先修好测量协议**（共同 hold、原版接触源、复算口径），再决定是否追加窄走廊批次。本 spec 落实该执行顺序。

## What Changes

- **唯一尾段控制者（探针 v3）**：在每种 `done_reason` 之后，由探针发布**同一个预定 hold 设定点**；记录命令时间/值/发布者身份，并订阅落盘实际 `/mavros/setpoint_position/local` 以核实「确实保持」与「无发布者竞争」。控制只用终止拍 **MAVROS 估计位姿**，不读 Gazebo 评分真值。
- **主接触源改为原版碰撞体**：使用 T2 已建的 `iris_3d_lidar_origcontact` 派生模型（**不新增 bumper 几何**）；**分列**原版 `base_link_inertia_collision`/`rotor_*_collision` 接触与真值几何相交（`geom_clearance_gz<=0`）。bumper 结果降为包络传感器的探索对照。
- **接触心跳落盘**：`/drone_contact` 的**空消息**也落盘（消息时间/序号），使每个尾段窗口都能判断信号是否在场（区分「窗口内无接触」与「局部失联」）。
- **复算口径修复（analyz v2）**：分析 v2 对**同一有效回合集合**计算主值与 `{2,4,8} s` 敏感性；按 `/clock` 判定**连续 2.0 s 低速**；记录每个终止后观察窗的**实际仿真时长**；按原版接触/真值相交**分列**结局。**不覆盖** T5 原始日志、冻结协议 v4.0 与原 `t5_analyze.py` 结果，只新增版本与修订注记。
- **v4.1 协议补充说明**：在看本轮结果前冻结，含 hold 定义、主接触源、心跳规则、复算规则、**固定清单**、**技术门槛**、新增窄走廊族参数与 ≤24 回合设计、研究范围判据。
- **固定清单小复核**：一个已翻转近障场景、一个未翻转近障场景、一个黑障场景，各自覆盖相关终止类型。
- **条件性窄走廊批次**：**仅当**「共同 hold、原版 contact、heartbeat、停稳复算」四项同时通过，才追加 ≤24 个窄走廊回合。
- 无 **BREAKING** 更改：不修改 `gazebo_env.py` 运行时控制器与真值评分器隔离，不修改冻结协议 v4.0，旧数据（`three/09`、T5 原始日志）只读保留。

## Impact

- Affected specs：终止后物理结局测量协议（four/04 v4.0）、T5 预试口径（four/05）、阶段裁决（four/06）
- Affected code：
  - 新增 `experiments/paper_v3/postterm_outcome/t6_recheck/`（探针 v3、接触心跳、分析 v2、运行脚本、新增场景）
  - 复用 `experiments/paper_v3/postterm_outcome/t2_contact/models/iris_3d_lidar_origcontact/iris_3d_lidar_origcontact.sdf`（`3d8ec420…8dfc5`）
  - 只读参考 `t3_chain/episode_tail_probe_v2.py`、`t3_chain/t3_recompute.py`、`t5_pretest/{t5_analyze.py,run_t5.sh}`
  - 新增文档 `docs/指导书/stage_logs/four/08_v4.1协议补充说明.md`、`09_修复后小复核.md`、`10_研究范围决定.md`

## ADDED Requirements

### Requirement: 唯一尾段控制者与共同 hold

探针 SHALL 在每种 `done_reason` 之后，作为**唯一控制者**发布与终止原因无关的**同一预定 hold 命令**，并 SHALL 记录每条 hold 命令的时间（`t_sim`/`t_wall`/`t_mono`）、值、发布者身份，以及实际 `/mavros/setpoint_position/local` 观测值。hold 目标 SHALL 取自**终止拍 MAVROS 估计位姿**，MUST NOT 使用 Gazebo 真值参与控制。

#### Scenario: 共同 hold 生效且无竞争
- **WHEN** 探针在 `shield_infeasible`、`sensor_invalid`/`perception_timeout`、`collision` 任一原因后进入尾段
- **THEN** 每个原因都发布同一形式的 hold 命令（冻结 LQR 参考相位 + 保持 offset），`/mavros/setpoint_position/local` 在整个尾段窗口内的漂移在预定容差内保持，且无第二个发布者与 hold 竞争

#### Scenario: 控制不使用评分真值
- **WHEN** 生成 hold 命令
- **THEN** 命令只依赖 `/mavros/local_position/pose` 与 LQR 跟踪器上报的参考量，不读取 `/gazebo/model_states` 真值

### Requirement: 主接触源为原版碰撞体且分列报告

接触结局 SHALL 以 T2 的 `iris_3d_lidar_origcontact` 模型（原版碰撞体挂接触传感器，不加 bumper 几何）为主接触源；SHALL 分列报告原版 `base_link_inertia_collision`/`rotor_*_collision` 接触与真值几何相交；bumper 接触 SHALL 仅作探索对照。

#### Scenario: 分列接触与相交
- **WHEN** 复算某一回合的尾段结局
- **THEN** 输出中「原版机体接触」「bumper 接触」「真值几何相交」三个字段相互独立、各自可追溯到 `contact.csv` 的 `collision1` 与 `trace.csv` 的 `geom_clearance_gz`

### Requirement: 接触信号心跳落盘

`/drone_contact` 的空消息 SHALL 落盘为心跳记录（含消息时间或序号），使每个尾段窗口都能判断信号是否在场。

#### Scenario: 区分无接触与局部失联
- **WHEN** 某尾段窗口内 `contact.csv` 无物体接触行
- **THEN** 可依据心跳记录判定该窗口是「有信号但无接触」还是「窗口内信号停止（局部失联）」，后者记基础设施失败，MUST NOT 记为无接触

### Requirement: 复算口径修复并保留旧版本

分析 v2 SHALL 使用**同一有效回合集合**计算主值与 `{2,4,8} s` 敏感性；SHALL 按 `/clock` 判定连续 `2.0 s`、`speed<0.05 m/s` 的停稳；SHALL 记录每个终止后观察窗的实际仿真时长。分析 v2 MUST NOT 覆盖 T5 原始日志、冻结协议 v4.0 或原分析结果。

#### Scenario: 主值与敏感性分母一致
- **WHEN** 计算主判据与 2/4/8 s 敏感性
- **THEN** 三者分母为同一有效回合集合（排除复位失败与接触信号缺失/局部失联），不再出现 T5 中「主表 21、敏感性 22」的不一致

#### Scenario: 停稳按整回合判据
- **WHEN** 判定某回合尾段是否停稳
- **THEN** 依据 `/clock` 连续 `2.0 s` 的低速区间，而非仅尾段末行速度

### Requirement: v4.1 协议补充说明在结果前冻结

在看本轮结果前 SHALL 产出 v4.1 协议补充说明，含 hold 定义、主接触源、心跳规则、复算规则、固定清单、技术门槛、新增窄走廊族参数与 ≤24 回合设计、研究范围判据。

#### Scenario: 预先定义避免事后更换口径
- **WHEN** 后续运行产生结果
- **THEN** 所有判据、门槛与新场景族参数均可回溯到 v4.1 文档，未在见结果后更改

### Requirement: 固定清单小复核与门槛

SHALL 约在一个已翻转的近障场景、一个未翻转的近障场景、一个黑障场景上执行小复核，每个至少覆盖其相关终止类型；SHALL 判定四项技术门槛是否同时通过。

#### Scenario: 门槛未过则不追加
- **WHEN** 「共同 hold」「原版 contact」「heartbeat」「停稳复算」任一项未通过
- **THEN** MUST NOT 追加 24 个窄走廊回合，只保留测量修复结论

### Requirement: 条件性窄走廊批次与研究范围决定

仅当四项技术门槛同时通过时，SHALL 按 v4.1 预定义的场景总体生成并运行 ≤24 个窄走廊回合，并**按家族分层**报告翻转率；随后 SHALL 给出研究范围决定：稳定且可解释 → 设计新批次；仅在挑选的近障家族出现 → 改题并另设独立家族验证；差异主要来自 bumper 或尾段控制差异 → 停止论文主张、保留为平台测量修复。

#### Scenario: 分层报告不合并
- **WHEN** 报窄走廊批次的翻转率
- **THEN** 按家族分别给出，MUST NOT 合并成单一数，且沿用 v4.0 的 ≥1/3 主判据不得事后放宽