# Tasks（change-id: establish-d-eval-science）

> 对应 [阶段3任务清单](../../../docs/项目任务日志/阶段3/任务清单.md) 第三节 D0/D1/D2。按 D0 → D1 → D2 顺序推进；
> 所有批跑遵守任务清单第二节统一规则（先锁协议后跑数据、记录尝试分母、配对可复现、指标可审计、契约检查点）。

- [x] Task 1: D0 冻结评估协议与基线
  - [x] SubTask 1.1: 新增 `src/air_ground_core/config/eval_protocol_v1.yaml`：成功/失败分类（复用 M0 state 4/5/6/8 → tracking_success/mission_failure/aborted/infra_failure）、真值判据（安全净空/碰撞以真值为准，估计量仅作控制输入与性能指标）、时间窗（mission/approach/search/hold/freshness）、数据字典（`meta.json`/`summary.json`/`state_events.jsonl`/`records.jsonl`/`batch.json` 字段）、三类分母恒等式、基线（gt_adapter 真值直供、air_fusion 多源融合、无注入现有栈）、预注册配对对照与主要指标、样本量规划（按 Wilson 期望区间宽度与最小有意义差异）、消融清单（固定场景/随机化、去掉关键跟踪机制）。
  - [x] SubTask 1.2: 新增 `src/air_ground_core/config/eval_manifest.schema.json`：manifest 条目字段（scenario_id/split/family/seed/scenario_file/scenario_hash/sensors_file/sensors_hash/params/created_from）。
  - [x] SubTask 1.3: 新增 `src/air_ground_core/scripts/d0_freeze.py`：计算并写出 `experiments/d/protocol_v1_hashes.json`（协议/schema/agc_params/perception_sensors/场景模板），支持 `--check` 复核；冻结后改动被冻结文件须 FAIL。
  - [x] SubTask 1.4: 撰写 `docs/项目任务日志/阶段3/D/D0.md`：协议正文、基线定义、预注册对照、样本量依据、**历史样例映射表**（N5 地面未见布局 0%、N1 短时恢复但整回合 APPROACH_TIMEOUT、N3 旧不可达贴障已达 vs 新 `scenario_truly_blocked.yaml`）、冻结前后版本与哈希、未决边界。

- [x] Task 2: D1 编排层与 launch 支持逐场景感知配置与种子
  - [x] SubTask 2.1: `src/air_ground_core/launch/agc_collab.launch` 新增参数 `perception_sensors_file`（默认包内 `config/perception_sensors.yaml`）与 `run_seed`（默认空），分别设置 `/agc/perception/sensors_file` 与 `/agc/log/seed`。
  - [x] SubTask 2.2: `scripts/start_agc_collab.sh` 新增 `--sensors-file <file>` 与 `--scene-seed <int>`，透传到 launch，并写入批跑 header：`records.jsonl` 的 `batch` 记录新增 `sensors_file`/`scene_seed` 字段。
  - [x] SubTask 2.3: 回归：不带新参数的批跑行为与改动前一致（默认 `sensors_file` 指向原文件、`seed=unseeded`），`meta.json` 的 `scenario_hash`/`agc_params_hash`/`seed` 字段符合预期。

- [x] Task 3: D1 随机化场景生成器与 manifest 构建
  - [x] SubTask 3.1: 新增 `src/air_ground_core/scripts/d1_scenario_gen.py`：定义场景族与 split（如 `nominal_static`/`nominal_moving`/`obstacle_layout`/`unreachable_closed`/`boundary`/`perception_fault`），按 (split, family, index) 派生独立种子，确定性地产出逐场景 `scenario_*.yaml` 与逐场景 `perception_*.yaml`。
  - [x] SubTask 3.2: 可行性约束与保留组：spawn 净空 ≥2.4m、目标速度 < 车上限、非故意不可达场景保证可达；保留故意不可达（封闭障碍，复用 N3 `scenario_truly_blocked` 语义）与边界组；生成时校验并记录被拒场景。
  - [x] SubTask 3.3: manifest 输出 `experiments/d/manifest_*.json`（含每场景/感知文件 sha256），提供 `--rebuild-check` 证明同主种子逐字节一致；提供 split 隔离断言（无布局/种子跨 split 重复）。

- [x] Task 4: D1 可续跑批跑器
  - [x] SubTask 4.1: 新增 `src/air_ground_core/scripts/d1_batch_runner.py`：读 manifest，按 split/family 过滤，逐场景调 `start_agc_collab.sh`（`--scenario`/`--sensors-file`/`--scene-seed`/`--batch-prefix`）。
  - [x] SubTask 4.2: 断点续跑与失败处理：状态文件记录已完成 run（按 scenario_id+seed）；重复种子检测；基础设施失败按策略重试并记录原失败（append-only，不覆盖历史）。
  - [x] SubTask 4.3: 三类分母汇总：从 run `summary.json` 读取分类，核对 `attempted = valid_task + infra + aborted`、`valid_task = tracking_success + mission_failure`，输出逐场景/逐族结果与批跑索引 `experiments/d/batches.json`。

- [x] Task 5: D1 冻结正式测试集并执行批跑
  - [x] SubTask 5.1: 按 `protocol_v1` 样本量规划生成并冻结 `experiments/d/manifest_test_v1.json`（含 train/val/test 及消融所需 manifest），写出冻结核验（哈希 + 确定性重建 PASS）。
  - [x] SubTask 5.2: 用批跑器执行冻结测试集（可断点续跑，长时后台运行），逐条保留 `run_id`/配置与代码哈希/种子/注入时序；基础设施失败与人工终止单列。
  - [x] SubTask 5.3: 汇总冻结测试集分母并核对两条恒等式 PASS；输出 `experiments/d/batches.json` 与原始 run 索引。

- [x] Task 6: D2 独立测试与统计报告管线
  - [x] SubTask 6.1: 新增 `src/air_ground_core/scripts/d2_stat_report.py`：Wilson 95% 成功率区间、失败模式占比 + Wilson 区间、按预注册对照的配对效应量（风险差/优势比）+ bootstrap 置信区间、泛化 gap、基础设施失败率与有效分母。
  - [x] SubTask 6.2: 消融统计：固定场景 vs 随机化、去掉关键跟踪机制（以 `target_source=gt_adapter` 真值直供对照 `air_fusion` 全链路，及成功判据敏感性）两类，输出对照表与原始 run 清单。
  - [x] SubTask 6.3: 报告落盘 `experiments/d/d2_report.json` + `docs/项目任务日志/阶段3/D/D2.md`：同时列点估计、95% 区间、`n`、泛化 gap、失败分类与基础设施故障；按预注册最小有意义差异判断证据是否充分（不以区间是否重叠判定）；每图附原始 run 清单。

- [x] Task 7: D2 地面 DQN 未见布局独立复评（重训 + 扩大评估）
  - [x] SubTask 7.1: 复用 `src/rl_ground_avoidance/scripts/n5_ground_scenarios.py` 的布局隔离，扩大 val/test 独立回合数（按 Wilson 规划）。
  - [x] SubTask 7.2: 新增/扩展脚本（两臂均完成：臂A复用扩大评估、臂B重训800回合后评估）：既评估现有 `best_model.pt`（扩大样本），又以更大预算重训后评估，落盘 `experiments/d/d2_ground_reeval.json`（含逐布局分解、失败分母、Wilson 区间、泛化 gap）。
  - [x] SubTask 7.3: 在 `D2.md` 写明“未见布局 0%”的结论边界：区分小样本边界与真实策略缺陷，保留原始 run/轨迹索引。

- [x] Task 8: 核验、验收文档与任务清单状态更新
  - [x] SubTask 8.1: 新增 `src/air_ground_core/scripts/verify_d_eval.py`：协议哈希冻结 PASS/FAIL、manifest 确定性、split 隔离、批跑三类分母恒等式、Wilson 计算正确性（对拍已知样例）、D2 报告完整性，输出 `experiments/d/verify_d_eval.json`。
  - [x] SubTask 8.2: 撰写 `docs/项目任务日志/阶段3/D/D1.md` 与补全 `D2.md` 的验收段：命令、目录、版本哈希、样本分母、核验器 PASS/FAIL、负例与未决边界。
  - [ ] SubTask 8.3: 更新 `docs/项目任务日志/阶段3/任务清单.md`：D0/D1/D2 状态改为「已验收」并链接交付物；未跑完不得标已验收。

- [x] Task 9: 执行架构改造（Gazebo 常驻热切换，效率）
  - [x] SubTask 9.1: `scenario_reset` 支持 `/agc/scenario/dynamic_obstacles`（动态删/增 Gazebo 障碍 + bump `/agc/scenario/epoch`）；默认 false 行为逐位等价
  - [x] SubTask 9.2: 新世界 `agc_world_dynamic.world`（无固定障碍）；`map_adapter`/`task_manager`/`intruder_driver`/`air_perception` 场景或感知配置变更即重载；`agc_run_logger` 在 attempt 起点重建 meta
  - [x] SubTask 9.3: `start_agc_collab.sh --serve/--dynamic-obstacles`；`d1_batch_runner.py --hot-swap`（按 target_source/obstacle_source 分组，一组一次启动）
  - [x] SubTask 9.4: pilot 验证：116s→~69s/回合；障碍物理存在审计 `scenario_obstacles_ok=True`；seed/哈希一致、`hash_mismatch=0`

# Task Dependencies
- Task 2、Task 3 相互独立，可并行；Task 4 依赖 Task 2、Task 3；Task 5 依赖 Task 4。
- Task 6 依赖 Task 5（冻结测试集 run 日志）；Task 7 依赖 Task 3（场景/布局）但可独立于 Task 5 推进。
- Task 8 依赖 Task 1、Task 4、Task 5、Task 6、Task 7。
- Task 1 是所有后续任务的前置（先锁协议）。