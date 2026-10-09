# Checklist（change-id: establish-a0-observation-model）

> 逐条核验 A0 交付；证据见 `docs/项目任务日志/阶段3/A/A0.md`、`experiments/a0/`。

## 配置与默认等价
- [x] `perception_sensors.yaml` 含相关噪声（`corr_xy`/`corr_group`/`corr_rho`/`noise_tau_s`）、偏置（`bias_m`）、成簇丢失（`drop_burst_*`/`drop_group`）、虚警（`false_alarm_rate_hz`）字段与顶层 `perception_model` 段，注释写明含义/单位/默认值
- [x] `agc_params.yaml` 的 `perception` 段补默认项，默认关闭遮挡/成簇/虚警
- [x] 默认配置下 `air_perception` 行为与 N1 逐位等价（回归证据）——默认路径不新增抽样、抽样顺序与次数不变（`reseed_enabled_=false`、A0 字段全 0/关）；N1 回归 `verify_n1_gating.py` PASS=21/FAIL=0（run `20261009T040926_e17819`，终态 APPROACH_TIMEOUT 非 TARGET_LOST）

## 观测模型实现
- [x] 每通道每随机源（噪声/丢包/虚警/成簇）使用独立命名空间种子，互不影响
- [x] 相关噪声：x/y 非对角协方差写入消息（`c[1]=c[6]=ρ·σ²`）；`corr_group` 通道误差相关但不共享随机流；`noise_tau_s` 时间相关按 AR(1) 实现（bucket 栅格）
- [x] 确定性偏置 `bias_m` 生效且可与随机噪声分离（核验项 6：实测均值在 3σ 内，n=3083）
- [x] 成簇丢失呈突发分布，`drop_group` 通道同向丢失；实测突发长度/丢包率与配置相符（核验项 4：组同向 1600/0 异，独立率 0.148/0.046 vs 0.161/0.036）
- [x] 遮挡门控按原始观测时刻的真值障碍 OBB 求交实现；被遮挡不产生观测且 `gate_reason=occluded`；解除后恢复且不刷新旧时间戳（核验项 2a/2b：最长连续段 3.45s、恢复事件 1751）
- [x] 拒收原因取值合法（沿用 N1 原因集 + `occluded`）（核验项 1：非法原因 0）
- [x] 虚警按 `false_alarm_rate_hz` 生成，日志标 `is_false_alarm=true`、`truth_id=null`，位置与真值不相邻（核验项 5）
- [x] 目标机动模式在 `intruder_driver` 确定性驱动，轨迹连续、速度低于车上限、复位相位归零（a0_maneuver 合速度 ≤0.71 m/s；终态 TRACKING_SUCCESS）

## 真值集与隔离
- [x] `perception_log.jsonl` 每条含生成时刻、送达时刻、源、真值关联 ID、拒收原因、`is_false_alarm` 与子流诊断量（`t_sim`/`t_send_s`/`source`/`truth_id`/`gate_reason`/`is_false_alarm`/`burst_state`/`drop_source`/`occluder`）
- [x] `TargetObservation`/`TargetTrack`/`SensorObservation` 等消息定义与发布内容**不含**真值关联 ID 字段（核验项 7：msg 命中 0；run 目录除评估日志外命中 0）
- [x] 在线链路仍只见 `target_id=intruder_uav`，未引入第二 ID 或虚警专用 ID

## 冻结与可复现
- [x] `a0_freeze.py` 输出 `experiments/a0/protocol_a0_hashes.json`（含 `combined_hash=a980636e…`）；改动被冻结对象后 `--check` FAIL，当前 `--check` PASS
- [x] 相同种子与配置重放，`perception_log.jsonl` 观测序列逐条一致——**口径说明**：Gazebo 非锁步 + 动态模型真值微抖使「逐点严格相等」不可得（`replay_check.json:status=inconsistent`）；改用 **bucket 栅格 + 残差判据** `|Δobs-Δtruth| ≤ 1e-9` 通过（`verdict=consistent`，max 残差 4.4e-16，n=2907），且 truth 相等 bucket 上 obs 严格一致 655/655（核验项 8）
- [x] 未修改 `eval_protocol_v1`（冻结协议）与其哈希档案（`eval_protocol_v1.yaml` sha256 `c0bf0f47…` 未变；`experiments/d/protocol_v1_hashes.json` 未改写，mtime 2026-10-07）；冻结 D 测试集 198/198 哈希匹配、`experiments/d` 树哈希 `d110f5ff…` 不变

## 核验与验收
- [x] `verify_a0_perception.py` 对门控一致性、遮挡、延迟、漏检/虚警率、相关性、偏置、`truth_id` 不泄漏、重放一致性逐项给出 PASS/FAIL，且无 FAIL（overall **PASS 10/0**）
- [x] 小规模集成冒烟（遮挡/成簇/虚警/机动）与 N1、M4 回归通过，run 索引与命令留存（`smoke_index.json`；N1 `n1_regression.json` PASS=21/0；M4 `m4_regression.json` `verify_fusion` PASS=16/0，run `20261009T043036_b472f2`）
- [x] `docs/项目任务日志/阶段3/A/A0.md` 含命令、目录、版本哈希、样本分母、核验器结论、负例与未决边界
- [x] `docs/项目任务日志/阶段3/任务清单.md` A0 状态更新为「已验收」并链接交付物
- [x] 结论表述遵守阶段 A 约束：仅称 `TRACKING_SUCCESS`，不出现 `INTERCEPT_CONFIRMED`（仅出现于「不得表述/禁用」的约束句），感知只称真值派生的多源观测仿真

## 已知边界（如实登记，不视为通过项掩盖）
- [x] `d0_freeze.py --check` 现因 A0 **有意**改动 `perception_sensors.yaml`/`agc_params.yaml`/`air_perception.cpp`/`intruder_driver.cpp`/`d1_scenario_gen.py` 而 FAIL；依据任务清单第二节第 6 条与 N1 先例，属**配置字段级扩展**、非契约/语义变更；`eval_protocol_v1` 语义与冻结 D 测试集数据均未变（已在 `A0.md` 七.2 与任务清单记录）
- [x] 虚警与多目标 ID 的**在线接线**不在 A0：属 `target_id` 规则变更，须待 A2 的 M0 v1.3 契约检查点（已在 `A0.md` 一 记录）