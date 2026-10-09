# Tasks

- [x] Task 1: 修补项目任务日志与指导书的引用与陈旧内容（纯文档，独立可并行）
  - [x] SubTask 1.1: 修正 [M1_单体验收_v1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目任务日志/M1/M1_单体验收_v1.md) 两处 `agc_params.yaml` 链接路径为 `src/air_ground_core/config/agc_params.yaml`
  - [x] SubTask 1.2: 修正 [M0_接口契约_v1.1.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目任务日志/M0/M0_接口契约_v1.1.md) 与 [M0_接口契约_v1.2.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目任务日志/M0/M0_接口契约_v1.2.md) 中对缺失 `M0_接口契约_v1.0.md` 的链接，改为“v1.0 已由 v1.1 取代、未随仓库保留”的文字说明
  - [x] SubTask 1.3: 更新 [项目总结_系统与进展.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目指导书/项目总结_系统与进展.md)：契约 `agc-m0-v1.0`→`agc-m0-v1.2`；4.2 节 `car_commander` 骨架表述按现状改写；第五节“尚缺条件”按 M3–M6 已验收事实更新；文档索引路径改指 `docs/项目任务日志/M0/M0_接口契约_v1.2.md`
  - [x] SubTask 1.4: 更新 [下一阶段任务清单.md](file:///home/ubuntu/anti_drone_agc_workspace/docs/项目指导书/下一阶段任务清单.md) M5 行为【已验收】，附 `M5_控制权交接验收_v1.md` 与权威批次 `_m5_nominal_20261003T133039` 等
  - [x] SubTask 1.5: 复核修改后全部相对链接可达（脚本化检查，0 断裂）

- [x] Task 2: 修复 ground_drive_adapter 复位独立性缺陷（实时链路 C++）
  - [x] SubTask 2.1: 订阅 `/agc/task/state`，按 `!isTerminal(state) && state != ST_IDLE` 计算 `task_running_`（镜像 `air_control_bridge` 既有语义）
  - [x] SubTask 2.2: `tick()` 中非运行期不积分、不写位姿，并把积分器重同步到 Gazebo 实际位姿；运行期保持原语义（积分 + watchdog + 写入权核查 + `write()`）
  - [x] SubTask 2.3: 保持 `/agc/drive/pose_authority_ok`、`/agc/drive/external_write_count`、`drive_log.jsonl` 日志字段不变（可加 `task_running` 字段便于复盘）
  - [x] SubTask 2.4: `catkin build air_ground_core` 编译通过

- [x] Task 3: 修复后回归验收并补记证据（依赖 Task 2）
  - [x] SubTask 3.1: 非无头（GUI）运行单栈多 attempt 批次（≥3 attempt），核验每 attempt 车实际起点 = `car_start (-4.000,-3.000)±1e-2`，终态与恒等式成立、`any_leftover=false`
  - [x] SubTask 3.2: 非无头复跑 M5 名义组与写入权冲突注入组，核验 `verify_control_authority.py` 全 PASS、无 `CONTROL_CONFLICT` 误报、终态零速停稳、冲突仍被检出
  - [x] SubTask 3.3: 在 M1/M5/M6 验收文档补记修复后证据与批次路径，并把 M6 §八.1 由“建议修复”更新为“已修复 + 证据”

# Task Dependencies
- Task 3 依赖 Task 2（须先完成代码修复与编译）
- Task 1 与 Task 2 相互独立，可并行