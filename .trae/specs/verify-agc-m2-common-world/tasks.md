# Tasks

> 依据 [spec.md](file:///home/ubuntu/anti_drone_agc_workspace/.trae/specs/verify-agc-m2-common-world/spec.md) 与 [M0_接口契约_v1.1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/%E9%A1%B9%E7%9B%AE%E4%BB%BB%E5%8A%A1%E6%97%A5%E5%BF%97/M0/M0_%E6%8E%A5%E5%8F%A3%E5%A5%91%E7%BA%A6_v1.1.md) 第二、五节。新代码落在 `src/air_ground_core/`；全程无头运行，进程按本会话启动差集回收；`start_sim.sh`、无人机单体与论文实验资产只读保留。

- [x] Task 1：地面车保护距离标定与碰撞口径对齐
  - [x] SubTask 1.1：`task_manager.cpp` 把接触判据改为仅由 `clearance ≤ car_radius_m` 触发 `GROUND_COLLISION`，移除 `collision_m_` 成员与读取；保留 `CLEARANCE_VIOLATION`/`EMERGENCY_TRIGGER` 为不改结局的保护带事件，并补净空度量（车中心到障碍 OBB 表面有符号距离）注释
  - [x] SubTask 1.2：`agc_common.cpp` 移除未被消费的 `Scenario` 净空字段（`prot`/`collision`/`emergency`）解析与定义
  - [x] SubTask 1.3：`config/agc_params.yaml` 的 `clearance` 段写入标定值与推导注释（`car_radius_m=0.30`、`emergency_m=0.35`、`protection_m=0.80`），删除 `collision_m`
  - [x] SubTask 1.4：`config/scenario_flat.yaml` 删除重复且从未生效的 `thresholds.clearance_*`
  - [x] SubTask 1.5：`catkin build air_ground_core` 通过，全仓无 `collision_m` / `clearance_collision_m` 残留引用

- [x] Task 2：M2 专用核验扩展（`scripts/audit_interfaces.py`）
  - [x] SubTask 2.1：新增「空中 odom 与 Gazebo `iris` 一致」核查（位置差 ≤ 0.20 m）
  - [x] SubTask 2.2：新增「同拍模型齐全」核查（单条 `model_states` 同时含 `iris`/`agc_car`/`intruder_uav`/`pole_1`/`pole_2`）
  - [x] SubTask 2.3：新增「同拍新鲜度」核查（同一窗口内空中 odom、地面 odom、目标观测均收到样本且年龄不超阈值）
  - [x] SubTask 2.4：新增「时间戳属仿真时钟」核查（采样 `header.stamp` 与 `/clock` 差 ≤ 1.0 s）
  - [x] SubTask 2.5：新增「地图与场景一致」核查（`/agc/map` 原点/分辨率/尺寸与 `scenario_flat.yaml` 的 `map` 段一致；`pole_1`/`pole_2` 中心所在栅格为占用；占用格数 > 0；`/agc/map/version_hash` 非空）
  - [x] SubTask 2.6：`python3 -c "import ast; ast.parse(open(...).read())"` 语法自检通过，核查项命名与输出格式与既有 PASS/FAIL 表一致

- [x] Task 3：M2 无头实跑与证据采集
  - [x] SubTask 3.1：运行 `./scripts/start_agc_collab.sh --cycles 1 --repeat 5`（无头），产出 `experiments/agc_runs/_m2_batch_<ts>/`
  - [x] SubTask 3.2：逐 run 核对 `air_calibration.json` 的 `residual_m` 非空且 ≤ 0.05 m；`meta.json` 的 `params.clearance` 含标定值且无 `collision_m`
  - [x] SubTask 3.3：核对执行轨迹最小净空（`tracking_frames.jsonl` 的 `clearance_m`，或 `planner_log.jsonl`）不低于 `protection_m`，且无 `GROUND_COLLISION`/`CLEARANCE_VIOLATION` 结算
  - [x] SubTask 3.4：收集 `audit_interfaces_cycle*.json`（全 PASS）、批跑 `batch.json`（干净退出、无残留），登记为 M2 证据索引

- [x] Task 4：M2 验收文档
  - [x] SubTask 4.1：写 `docs/项目任务日志/M2/M2_共同世界验收_v1.md`：任务口径、批跑设置、逐项结果（世界/时钟/TF/模型命名/同拍位姿/坐标变换/时间戳/地图接口/单入口与退出）、保护距离标定推导与核对、交付物核对、未决事项

# Task Dependencies

- Task 1 与 Task 2 相互独立，可并行。
- Task 3 依赖 Task 1、Task 2（核验脚本与阈值改动都要先落地）。
- Task 4 依赖 Task 3（需要实跑证据）。
- 各 Task 产物互不覆盖；`start_sim.sh`、无人机单体实验与论文实验资产只读保留。