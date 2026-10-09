# 阶段3 D：评估科学化 Spec

## Why
阶段3要求 D（评估科学化）先行，作为 A/B 正式评估的闸门。当前栈的评估能力存在三类缺口：
1. **协议未冻结**：成功/失败分类、真值判据、时间窗与数据字典散落在各任务文档，没有单一可哈希的 `protocol_v1`，也没有场景/种子 manifest 与统计输入格式。
2. **场景不可随机化、批跑不可续**：AGC 链路只有固定场景 YAML（`config/scenario_*.yaml`），`scenario_reset`/`intruder_driver` 不消费 seed；感知参数来自单一 `perception_sensors.yaml`；`start_agc_collab.sh` 无断点续跑、重复种子检测、失败重试记录。
3. **统计不足**：`summarize_runs.py` 只做计数与 M0 恒等式，无 Wilson 95% 区间、失败模式占比区间、配对效应量、泛化 gap；N5 地面 DQN 报告 val=4/test=6 回合全碰撞、成功率 0%，是小样本边界，需独立大样本复评。

## What Changes
- 新增机器可读的冻结评估协议 `eval_protocol_v1.yaml` 与场景/种子清单 schema `eval_manifest.schema.json`，并提供冻结/校验脚本与哈希留档。
- 编排层新增逐场景 **sensors_file** 与 **seed** 透传（`start_agc_collab.sh` + `agc_collab.launch`），使噪声/视场/遮挡/丢包/故障参数可按场景随机化。
- 新增确定性的随机化场景生成器与 manifest 构建器（物理可行性约束、故意不可达与边界组、按布局/种子隔离的 train/val/test）。
- 新增可断点续跑的批跑器：三类分母核对、重复种子检测、基础设施失败重试并保留记录（只追加不覆盖）。
- 新增统计报告管线：Wilson 95% 成功率区间、失败模式占比与区间、配对效应量与置信区间、泛化 gap、消融（固定场景/随机化、去掉关键跟踪机制），每图附原始 run 清单。
- 新增地面 DQN 未见布局独立复评：既重训（更大预算）又扩大独立评估样本量，两者对照。
- 新增统一核验脚本与 `D0.md`/`D1.md`/`D2.md` 验收记录，并更新阶段3任务清单状态。

## Impact
- Affected specs: 阶段3 D（本 spec 新建）；沿用 `agc-m0-v1.2` 单世界/`world` 坐标/仿真时间/唯一地面执行口/LQR 唯一空中 setpoint 发布者；阶段 A 终态仍只称 `TRACKING_SUCCESS`，禁用 `INTERCEPT_CONFIRMED`。
- Affected code:
  - `scripts/start_agc_collab.sh`（新增 `--sensors-file`、`--scene-seed`）
  - `src/air_ground_core/launch/agc_collab.launch`（新增 `perception_sensors_file`、`run_seed` 参数）
  - `src/air_ground_core/scripts/`（新增 d0/d1/d2/verify 脚本）
  - `src/air_ground_core/config/`（新增 `eval_protocol_v1.yaml`、`eval_manifest.schema.json`）
  - `src/rl_ground_avoidance/scripts/`（复用 `n5_ground_scenarios.py`）
  - `experiments/d/`（协议哈希、manifest、场景/感知文件、批跑索引、统计报告）
  - `docs/项目任务日志/阶段3/D/`（D0/D1/D2 验收文档）、`docs/项目任务日志/阶段3/任务清单.md`（状态更新）
- 契约影响：不新增/修改协同话题，**不构成 M0 语义变更**，契约保持 `agc-m0-v1.2`。

## ADDED Requirements

### Requirement: 冻结评估协议与基线（D0）
系统 SHALL 提供机器可读、可哈希的 `eval_protocol_v1`，统一成功/失败分类、真值判据、时间窗、数据字典、三类分母规则、基线与预注册对照，并给出场景/种子 manifest schema 与统计脚本输入格式。

#### Scenario: 协议冻结可追溯
- **WHEN** 运行冻结校验脚本
- **THEN** 输出协议/schema/参数/场景模板的哈希到 `experiments/d/protocol_v1_hashes.json`，并给出 PASS/FAIL；改动任一被冻结文件后重跑会 FAIL（冻结前后版本可追溯）

#### Scenario: 历史样例映射到新分类
- **WHEN** 查阅 `D0.md`
- **THEN** 给出 N5 地面未见布局 0%、N1 恢复但整回合失败、N3 新旧不可达语义到新分类（`tracking_success`/`mission_failure`/`aborted`/`infra_failure`）的逐条映射，并标明历史样本不计入阶段3正式测试分母

### Requirement: 逐场景感知配置与种子注入
编排脚本与 launch SHALL 支持逐场景传入感知配置文件与运行种子，且不改变现有话题语义。

#### Scenario: 参数透传
- **WHEN** 以 `--sensors-file <file> --scene-seed <int>` 启动一次 run
- **THEN** `/agc/perception/sensors_file` 指向该文件、`/agc/log/seed` 记录该种子；run 的 `meta.json.seed` 与 `meta.json.scenario_hash`/`agc_params_hash` 非空且与现场一致；默认不带参数时行为与改动前一致

### Requirement: 随机化场景生成与清单（D1）
生成器 SHALL 从单一主种子确定性地产出场景 YAML、逐场景感知配置与 manifest，覆盖目标初始位姿、静止/匀速/转向/机动轨迹、障碍布局、起始位姿、视场/噪声/遮挡/故障参数；满足物理可行性约束并保留故意不可达与边界组；按布局与命名空间隔离 train/val/test。

#### Scenario: manifest 可重建且无泄漏
- **WHEN** 用同一主种子重新生成 manifest
- **THEN** 生成结果逐字节一致；不同 split 之间无场景或种子重复；每个场景的文件哈希写入 manifest

### Requirement: 可续跑批跑器（D1）
批跑器 SHALL 支持断点续跑、复位隔离、重复种子检测与失败重试记录，并逐条核对三类分母；基础设施失败不得被静默重跑后抹去。

#### Scenario: 三类分母可核对
- **WHEN** 对某个已完成批跑运行汇总
- **THEN** `attempted = valid_task + infra_failure + aborted`、`valid_task = tracking_success + mission_failure` 两条恒等式 PASS；每个 run 的 `run_id`、配置/代码哈希、种子、注入时序可追溯；重试 run 保留原记录只追加

### Requirement: 独立测试与统计报告（D2）
统计管线 SHALL 在冻结测试集上计算 Wilson 95% 成功率区间、失败模式占比与区间、配对效应量与置信区间、泛化 gap 与基础设施失败率，并按预注册的最小有意义差异判断证据是否充分；消融至少包含固定场景/随机化与去掉关键跟踪机制两类；每图附原始 run 清单。

#### Scenario: 统计输出完整
- **WHEN** 运行统计报告脚本
- **THEN** 报告同时给出点估计、95% 区间、`n`、泛化 gap、失败分类与基础设施故障；样本量按期望区间宽度/最小有意义差异预注册，不使用“两组区间是否重叠”判定

#### Scenario: 地面 DQN 未见布局复评
- **WHEN** 执行 DQN 复评
- **THEN** 同时给出“复用既有 best_model.pt 扩大独立评估”与“重训后评估”两套结果，含 Wilson 区间与泛化 gap，并保留逐布局分解与失败分母

### Requirement: 阶段3 D 核验与验收
系统 SHALL 提供统一核验脚本，对 D0/D1/D2 的关键判据给出 PASS/FAIL，并产出 `D0.md`/`D1.md`/`D2.md` 验收记录。

#### Scenario: 核验通过
- **WHEN** 运行核验脚本
- **THEN** 协议哈希、manifest 确定性、split 隔离、三类分母恒等式、Wilson 计算正确性、报告完整性逐项 PASS，且任务清单 D0/D1/D2 状态与证据一致

## MODIFIED Requirements
### Requirement: 评估汇总入口
现有 `summarize_runs.py` 保持既有计数与 M0 恒等式行为不变；其输出 `aggregate.json` 作为 D2 统计管线的输入之一，统计增强由新增 `d2_stat_report.py` 承担，不修改既有口径。

## REMOVED Requirements
无。