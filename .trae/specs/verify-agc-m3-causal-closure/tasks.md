# Tasks

> 依据 [spec.md](file:///home/ubuntu/anti_drone_agc_workspace/.trae/specs/verify-agc-m3-causal-closure/spec.md) 与 [M0_接口契约_v1.1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E4%BB%BB%E5%8A%A1%E6%97%A5%E5%BF%97/M0/M0_%E6%8E%A5%E5%8F%A3%E5%A5%91%E7%BA%A6_v1.1.md) 第三、六、七节。新代码落在 `src/air_ground_core/`；全程无头运行，进程按本会话启动差集回收；实时链路 C++ 节点、`scenario_flat.yaml`、`start_agc_collab.sh` 只读保留。

- [x] Task 1：受控对比场景（仅改变空中目标输入）
  - [x] SubTask 1.1：新增 `config/scenario_linear_target.yaml`：`scenario_id=flat_linear_target`、`locked=true`；`car_start`/`target_start`/`obstacles`/`map`/`thresholds` 与 `scenario_flat.yaml` 逐字段一致，仅 `target_motion` 改为 `mode: linear`、`speed_mps: 0.6`、`waypoints: [[6.5,2.5],[3.5,2.5]]`
  - [x] SubTask 1.2：可达性说明：目标速度 0.6 m/s < `commander.max_linear_speed_mps`（1.0 m/s），且目标航段 `y=2.5` 避让 `pole_1`/`pole_2`，逐段做净空/可达性核对
  - [x] SubTask 1.3：脚本化核对两场景「除 `target_motion` 外逐字段一致」（解析 YAML 逐键比较），作为 Task 2 核验的一部分
  - [x] SubTask 1.4：`catkin build air_ground_core` 通过；`roslaunch --files`/`rosparam load` 能加载新场景（用 python yaml 解析自检亦可）

- [x] Task 2：离线因果闭环核验工具 `scripts/verify_causal_closure.py`
  - [x] SubTask 2.1：CLI：`--control-run`、`--treatment-run`、`--noinfo-run`、`--control-scenario`、`--treatment-scenario`、`--out`；打印 PASS/FAIL 表，写出 JSON，任一 FAIL 退出码 1
  - [x] SubTask 2.2：场景一致性核查：两场景除 `target_motion` 外逐字段一致；处理组 `mode=linear` 且 `speed_mps ≤ 1.0`；三 run 的 `meta.json` 可复算哈希且对照/处理组 `agc_params_hash` 相同
  - [x] SubTask 2.3：输入核查：处理组 `topics/_agc_targets.jsonl`（或 `target_trajectory.jsonl`）目标地面位置变化范围 ≥ 1.0 m；对照组 ≤ 0.1 m
  - [x] SubTask 2.4：规划核查：对齐 `/agc/targets` 与 `/agc/ground/goal`，处理组 `goal` 与目标投影中位距离 ≤ 0.5 m、最大 ≤ 1.5 m 且 `goal` 变化范围 ≥ 1.0 m；对照组 `goal` 变化范围 ≤ 0.2 m；`planner_log.jsonl` 规划记录字段齐全，处理组含 ≥1 次 `trigger="displacement"`，对照组无 `displacement` 且 trigger 仅取 `first`/`period`/`infeasible`/`invalid`
  - [x] SubTask 2.5：执行核查：处理组 `/agc/ground/odom` 最大位移 ≥ 3.0 m；`GROUND_APPROACH` 期间存在非零 `cmd_vel.linear.x`；车位移方向与「车→goal」方向的平均同向性为正
  - [x] SubTask 2.6：无信息核查：`/agc/targets` 转空后 ≤1.0 s 内 `cmd_vel` 归零；自最后有效目标样本起车位移 ≤ 0.10 m；终态为 `MISSION_FAILED`/`TARGET_LOST`
  - [x] SubTask 2.7：`python3 -c "import ast; ast.parse(open(...).read())"` 语法自检通过；用现有历史 run（静态成功 run）做一次 dry-run 确认对照组断言可过

- [x] Task 3：M3 无头批跑与证据采集
  - [x] SubTask 3.1：对照组批跑 `./scripts/start_agc_collab.sh --cycles 1 --repeat 2`（默认 `scenario_flat.yaml`），产出 `_m3_batch_<ts>_control/`
  - [x] SubTask 3.2：处理组批跑 `./scripts/start_agc_collab.sh --cycles 1 --repeat 2 --scenario <pkg>/config/scenario_linear_target.yaml`，产出 `_m3_batch_<ts>_treatment/`
  - [x] SubTask 3.3：无信息样例 `./scripts/start_agc_collab.sh --auto-run mission_failed`，产出 `_m3_batch_<ts>_noinfo/`
  - [x] SubTask 3.4：登记三组 run 目录与 `batch.json`/`records.jsonl` 路径；确认每 run 干净退出（`any_leftover=false`）
  - [x] SubTask 3.5：如批跑脚本生成的目录前缀为 `_m1_batch_`，按 M2 先例重命名为 `_m3_batch_*`（不改动目录内容）

- [x] Task 4：M3 验收文档与核验结论
  - [x] SubTask 4.1：以三组 run 运行 `verify_causal_closure.py`，产出 `experiments/agc_runs/m3_causal_closure.json`（全 PASS）
  - [x] SubTask 4.2：写 `docs/项目任务日志/M3/M3_空地最小闭环验收_v1.md`：任务口径、批跑设置、受控对比证明（仅目标不同）、因果链逐项结果（goal/path/cmd/实际轨迹/重规划原因）、无信息安全停车、结果口径与措辞（`TRACKING_SUCCESS` ≠ 拦截完成）、未决事项
  - [x] SubTask 4.3：核对 M3 证据索引与 `spec.md`/`checklist.md` 一致

# Task Dependencies

- Task 1 与 Task 2 相互独立，可并行（Task 2 的 2.2 需要 Task 1 的场景文件，故 2.2 待 1.1 完成后联调）。
- Task 3 依赖 Task 1（新场景）与 Task 2（核验脚本）。
- Task 4 依赖 Task 3（需要实跑证据）。
- 各 Task 产物互不覆盖；实时链路节点、`scenario_flat.yaml`、`start_agc_collab.sh` 只读保留。
