# Checklist（change-id: establish-d-eval-science）

## D0：冻结评估协议与基线
- [x] `src/air_ground_core/config/eval_protocol_v1.yaml` 含成功/失败分类、真值判据、时间窗、数据字典、三类分母恒等式、基线、预注册配对对照与主要指标、样本量规划、消融清单。
- [x] `src/air_ground_core/config/eval_manifest.schema.json` 定义 manifest 条目字段并可校验生成的 manifest。
- [x] `src/air_ground_core/scripts/d0_freeze.py` 写出 `experiments/d/protocol_v1_hashes.json`，冻结后改动被冻结文件时 `--check` 返回 FAIL（冻结前后可追溯）。
- [x] `docs/项目任务日志/阶段3/D/D0.md` 含历史样例映射表（N5 地面未见布局 0%、N1 短时恢复但整回合 APPROACH_TIMEOUT、N3 旧贴障不可达 vs 新 `scenario_truly_blocked.yaml`），并声明历史样本不计入阶段3正式测试分母。
- [x] 协议明确沿用 `agc-m0-v1.2`，未新增/修改协同话题，未使用 `INTERCEPT_CONFIRMED`。

## D1：编排/launch 与随机化生成、批跑
- [x] `agc_collab.launch` 新增 `perception_sensors_file`、`run_seed` 参数并正确设置 `/agc/perception/sensors_file` 与 `/agc/log/seed`。
- [x] `start_agc_collab.sh` 新增 `--sensors-file`、`--scene-seed`，透传 launch 并写入 `records.jsonl` 的 batch 记录；不带参数时行为与改动前一致。
- [x] `d1_scenario_gen.py` 由同一主种子确定性产出场景 YAML、逐场景感知配置与 manifest；`--rebuild-check` 证明逐字节一致。
- [x] 生成器满足 spawn 净空 ≥2.4m、目标速度 < 车上限、非故意不可达场景可达；保留故意不可达（封闭障碍）与边界组；被拒场景有记录。
- [x] train/val/test 按布局与种子命名空间隔离，无场景或种子跨 split 重复（隔离断言 PASS）。
- [x] `d1_batch_runner.py` 支持断点续跑、重复种子检测、失败重试记录（append-only），并从 run 日志逐条核对三类分母。
- [x] `experiments/d/manifest_test_v1.json` 已按 `protocol_v1` 样本量冻结，冻结哈希与确定性重建 PASS。
- [x] 冻结测试集已执行（可断点续跑），逐 run 保留 `run_id`、配置/代码哈希、种子与注入时序；基础设施失败与人工终止单列。
- [x] 冻结测试集汇总两条分母恒等式 PASS。

## D2：独立测试与统计报告
- [x] `d2_stat_report.py` 输出 Wilson 95% 成功率区间、失败模式占比与区间、配对效应量与置信区间、泛化 gap、基础设施失败率与有效分母。
- [x] 消融含固定场景/随机化、去掉关键跟踪机制两类，并给对照表与原始 run 清单。
- [x] `experiments/d/d2_report.json` 与 `docs/项目任务日志/阶段3/D/D2.md` 同时列点估计、95% 区间、`n`、泛化 gap、失败分类与基础设施故障；按预注册最小有意义差异判断证据充分性，不以区间重叠判定。
- [x] 地面 DQN 复评同时给出“复用既有 best_model.pt 扩大评估”与“重训后评估”两套结果，含 Wilson 区间、泛化 gap、逐布局分解与失败分母。
- [x] `D2.md` 对“未见布局 0%”区分小样本边界与真实策略缺陷，并保留原始 run/轨迹索引。

## 核验与验收
- [x] `verify_d_eval.py` 对协议哈希、manifest 确定性、split 隔离、三类分母恒等式、Wilson 计算、报告完整性逐项输出 PASS/FAIL，落盘 `experiments/d/verify_d_eval.json`。
- [x] `docs/项目任务日志/阶段3/D/D1.md`、`D2.md` 给出命令、目录、版本哈希、样本分母、核验器 PASS/FAIL、负例与未决边界。
- [x] `docs/项目任务日志/阶段3/任务清单.md` 中 D0/D1/D2 状态与证据一致（未跑完不得标已验收）。
- [x] 未引入未解释的发布者冲突、历史数据覆盖、测试集泄漏或 `INTERCEPT_CONFIRMED`。