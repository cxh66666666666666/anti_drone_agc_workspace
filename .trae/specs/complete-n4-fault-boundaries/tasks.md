# Tasks（change-id: complete-n4-fault-boundaries）

- [x] Task 1: 融合冷启动冲突规则（`air_fusion`）
  - [x] SubTask 1.1: 在 `air_fusion.cpp` 的 `fuseTargets()` 中，无历史参考时按 `gate_m` 单链聚类取最大簇作参考；候选 ≥2 且最大簇 < `cold_start_min_agree` 时发布空数组并记 `result=cold_start_ambiguous`；有历史路径保持不变
  - [x] SubTask 1.2: 融合日志新增 `cold_start`、`ref_source`（`history`|`cluster`|`single`|`ambiguous`）、`cluster_sizes` 字段
  - [x] SubTask 1.3: `config/agc_params.yaml` 新增 `fusion/cold_start_min_agree: 2`（run 前冻结，哈希入 run 日志）
  - [x] SubTask 1.4: 单通道失效降级确认——仅 1 个候选时正常输出（不误判冲突）

- [x] Task 2: 控制侧故障边界与注入能力
  - [x] SubTask 2.1: `drone_launch_controller`：`releaseControlCallback` 中 `local_pos_pub_.shutdown()`；`takeoffCallback`/`armMotorsCallback` 入口确保重新 advertise（保证批跑多 attempt 可用）
  - [x] SubTask 2.2: `agc_collab.launch` 新增 `mission_timeout_s` arg 覆盖 `limits/mission_timeout_s`（默认取 agc_params 值）
  - [x] SubTask 2.3: `scripts/start_agc_collab.sh` 新增 `--timeout-inject <sec>` 与 `--handover-inject <sec>`（交接后按秒 `rosnode kill` LQR），并写入 batch.json 的 `inject` 说明

- [x] Task 3: 离线核验扩展
  - [x] SubTask 3.1: `verify_fusion.py` 增 `outlier` 模式（校验冷启动歧义、目标不被偏置、终态为超时类）
  - [x] SubTask 3.2: `verify_fusion.py` 增 `single_channel` 模式（单通道失效后仍输出、无误拒）
  - [x] SubTask 3.3: `verify_control_authority.py` 增 `timeout` 模式（`MISSION_TIMEOUT` 终态 + 终态后停车/hold + 唯一控制源）
  - [x] SubTask 3.4: `verify_control_authority.py` 增 `handover` 模式（交接后 advertiser 不残留 + 控制空窗被记录）

- [x] Task 4: 批跑取证与验收文档
  - [x] SubTask 4.1: 运行名义组、冷启动高置信离群组、单通道失效组、超时组、交接异常组（`--gui`，非无头），锁定场景/参数/哈希
  - [x] SubTask 4.2: 回归 M4 nominal（`verify_fusion nominal`）、M5 nominal/conflict（`verify_control_authority`）、M6 nominal（`verify_m6_scenarios`），确认无新增误报
  - [x] SubTask 4.3: 撰写 `docs/项目任务日志/阶段2/N4.md`（口径、实现、批次、逐项 PASS/FAIL、交付物核对、未决）

# Task Dependencies

- Task 3 depends on Task 1、Task 2（核验模式依赖新日志字段与注入选项）
- Task 4 depends on Task 1、Task 2、Task 3
- Task 1 与 Task 2 相互独立，可并行