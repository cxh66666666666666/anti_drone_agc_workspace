# Tasks（change-id: establish-a0-observation-model）

> 对应 [阶段3任务清单](../../../docs/项目任务日志/阶段3/任务清单.md) 第四节 A0。承接 N1 几何门控与 D0 `eval_protocol_v1`（不修改该冻结协议）。
> 所有批跑遵守任务清单第二节统一规则（先锁协议后跑数据、记录尝试分母、配对可复现、指标可审计、契约检查点）。
> 契约边界：A0 只做**配置字段级 + 评估日志级**改动，不新增/修改协同话题；真值关联 ID 只写评估日志；虚警/多目标在线接线延后到 A2。

- [x] Task 1: 观测模型配置 schema 扩展与默认参数
  - [x] SubTask 1.1: `src/air_ground_core/config/perception_sensors.yaml` 通道新增字段：`corr_xy`（通道内 x/y 相关）、`corr_group`/`corr_rho`（通道间公共模态）、`noise_tau_s`（时间相关 AR(1)）、`bias_m`（确定性偏置 `[x,y,z]`）、`drop_burst_prob`/`drop_burst_mean_s`/`drop_burst_mean_gap_s`、`drop_group`、`false_alarm_rate_hz`；新增顶层 `perception_model` 段（遮挡开关 `occlusion.enabled`/`occlusion.pad_m`、全局开关）。
  - [x] SubTask 1.2: `src/air_ground_core/config/agc_params.yaml` 的 `perception` 段补默认项（遮挡默认关、成簇/虚警默认 0，保证默认行为与 N1 逐位等价）。
  - [x] SubTask 1.3: 在文件头注释写明各字段含义、单位、默认值，以及「真值派生的感知仿真」诚实口径与「相关噪声不共享随机流但误差相关」的实现约定。

- [x] Task 2: `air_perception` 相关噪声、偏置与成簇丢失实现
  - [x] SubTask 2.1: 每通道每随机源独立 RNG 命名空间（噪声/丢包/虚警/成簇），基线种子由 `/agc/log/seed` 与通道索引确定性派生；替换现有单一 `rngs_` 流。
  - [x] SubTask 2.2: 相关噪声：通道内 x/y 相关写入 `pose.covariance` 非对角项；`corr_group` 通道共享公共模态噪声（保持独立随机流、误差相关）；`noise_tau_s` 实现 AR(1) 时间相关。
  - [x] SubTask 2.3: 确定性偏置 `bias_m`（与随机噪声分离，可被离线核验区分）。
  - [x] SubTask 2.4: 成簇丢失（两态 Markov：正常态/突发态），支持 `drop_group` 跨通道同向丢失；与既有 `drop_prob`（逐拍独立）共存且有明确优先级。

- [x] Task 3: 遮挡门控
  - [x] SubTask 3.1: 在 `air_perception` 实现视线遮挡判定：无人机位置→目标/障碍质心的线段与真值障碍 OBB（`agc::Obstacle`，含 `occlusion.pad_m` 外扩）求交。
  - [x] SubTask 3.2: 被遮挡通道不产生对应观测，`gate_reason=occluded`；与 N1 门控次序固定且可判定；解除后恢复且不刷新旧观测时间戳。
  - [x] SubTask 3.3: 默认 `occlusion.enabled=false` 时行为与 N1 逐位等价（回归）。

- [x] Task 4: 虚警生成（评估日志级）
  - [x] SubTask 4.1: 按 `false_alarm_rate_hz` 生成幻影目标观测；记录 `is_false_alarm=true`、`truth_id=null`，位置与真值不相邻。
  - [x] SubTask 4.2: 确认虚警**不进入** `/agc/air/obs`/`/agc/targets` 的消息语义（在线仍单 `target_id`）；虚警仅落评估日志。

- [x] Task 5: 真值集日志扩展与真值 ID 隔离
  - [x] SubTask 5.1: `perception_log.jsonl` 逐样本新增：生成时刻（复用 `t_sim`，等价 `t_obs`）、送达时刻 `t_send_s`、源（`sensor_id`）、真值关联 ID `truth_id`、`gate_reason`（含 `occluded`）、`is_false_alarm`、以及噪声/丢包子流诊断量；保留 N1 既有字段（`gate_ok`/`target_range_m`/`target_az_deg`/`target_el_deg`）。
  - [x] SubTask 5.2: 核验 `TargetObservation`/`TargetTrack`/`SensorObservation` 等消息定义与其发布内容均不含真值关联 ID 字段。

- [x] Task 6: 目标机动扩展
  - [x] SubTask 6.1: `agc_common.cpp` 的 `loadScenario` 解析 `target_motion` 新增机动模式与参数（在 `static`/`linear` 之外，如 `sinusoid`/`circle`）。
  - [x] SubTask 6.2: `intruder_driver.cpp` 按仿真时钟确定性生成机动轨迹；位置/速度连续、速度仍低于车上限；`target_trajectory.jsonl` 落盘；复位相位归零行为保持。
  - [x] SubTask 6.3: `d1_scenario_gen.py` 的 `sample_motion` 支持机动模式（供 A1/A3 抽取机动场景）。

- [x] Task 7: 感知随机化集成与 A0 冻结
  - [x] SubTask 7.1: `d1_scenario_gen.py` 的 `make_sensors` 在 `perception_fault`/`nominal` 族中随机化 A0 新字段（相关/偏置/成簇/虚警），保持 v1/v2 标定语义并复现既有哈希口径。
  - [x] SubTask 7.2: 新增 `src/air_ground_core/scripts/a0_freeze.py`：对 `perception_sensors.yaml`、`agc_params.yaml`(perception 段)、场景/感知模板、核验脚本取 sha256，写出 `experiments/a0/protocol_a0_hashes.json`（含 `combined_hash`），支持 `--check` 复核（改动即 FAIL）。
  - [x] SubTask 7.3: 重放一致性：相同种子与配置重复运行，`perception_log.jsonl` 观测序列逐条一致（差异仅墙钟字段）。

- [x] Task 8: A0 核验脚本与验收文档
  - [x] SubTask 8.1: 新增 `src/air_ground_core/scripts/verify_a0_perception.py`：FOV/量程门控一致性、遮挡原因合法性与时序、延迟分布、漏检率/虚警率与配置相符、相关噪声（通道内/通道间/时间）与配置相符、偏置均值相符、`truth_id` 不泄漏、重放一致性；输出 `experiments/a0/verify_a0_perception.json`。
  - [x] SubTask 8.2: 执行小规模集成冒烟（含遮挡、成簇丢失、虚警、机动）与 N1/M4 回归，保留 run 索引与命令。
  - [x] SubTask 8.3: 撰写 `docs/项目任务日志/阶段3/A/A0.md`（命令、目录、版本哈希、样本分母、核验器 PASS/FAIL、负例与未决边界），并更新 `docs/项目任务日志/阶段3/任务清单.md` A0 状态为「已验收」并链接交付物；未跑完不得标已验收。

- [x] Task 9: 修正 a0_occlusion 场景使遮挡可恢复，并重验核验项 2b
  - [x] SubTask 9.1: 修改 `d1_scenario_gen.py` 的 A0 `a0_occlusion` 场景：目标改为**瞬态穿越遮挡**（如 `linear` 从 (5,-3) 到 (5,3) 往返，速度 ~0.5 m/s），障碍保持在无人机悬停点与目标初始连线上的 (2.5,0)——使视线在 |y| 小时被遮挡、|y| 大时解除（遮挡窗口约 2 s）。
  - [x] SubTask 9.2: 重新生成 `experiments/a0/manifest_a0.json` 与场景/感知文件；默认（未开 `--a0-model`）输出保持逐字节不变、`experiments/d` 不变。
  - [x] SubTask 9.3: 重跑 `a0_occlusion` 冒烟与 `verify_a0_perception.py`，确认核验项 2b（遮挡段连续 + 之后恢复 `has_target=true`）PASS；重跑 `a0_freeze.py` 并 `--check` PASS；更新 `docs/项目任务日志/阶段3/A/A0.md`（核验表、run 目录、哈希、未决边界同步）。

- [x] Task 10: 补 M4 融合名义回归（Task 8.2 要求 N1/M4 回归，此前仅完成 N1）
  - [x] SubTask 10.1: 用**默认** `perception_sensors.yaml`（A0 字段全 0/关）跑一次名义批跑（`config/scenario_flat.yaml`，`--target-source air_fusion --obstacle-source air_fusion`）。
  - [x] SubTask 10.2: 对批跑目录运行 `verify_fusion.py --mode nominal`（必要时补 `--mode obstacle`），期望 PASS；把命令、批跑/run 目录、PASS/FAIL 计数与门控/融合关键指标写入 `experiments/a0/m4_regression.json`。
  - [x] SubTask 10.3: 在 `docs/项目任务日志/阶段3/A/A0.md` §五 补 M4 回归一段（含 run 目录与核验结论）。

# Task Dependencies
- Task 1 是 Task 2/3/4/5 配置前置；Task 2、3、4 相互独立，可并行。
- Task 5 依赖 Task 2/3/4（字段来源）；Task 6 独立于感知链路，可并行。
- Task 7 依赖 Task 1–6；Task 8 依赖 Task 7。
- Task 1 是所有后续任务的前置（先锁配置口径）。
- 契约检查点：Task 4 只做评估日志级虚警；任何把虚警/多目标送入在线消费者的工作**不在本 spec**，须待 A2 的 M0 v1.3 契约变更表。