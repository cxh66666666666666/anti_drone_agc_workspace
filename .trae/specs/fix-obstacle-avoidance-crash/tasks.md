# Tasks

- [x] Task 1: 修复 rl_planner_node.py 的 plan_once() 方法
  - 将 `self.choose_action()` 和 `self.action_to_offset(action)` 替换为 `self.compute_escape_offset()`
  - 为 `features` 添加 None 检查，防止首次触发时崩溃
  - 移除对 `action` 变量的依赖，将 `Int32(data=action)` 改为 `Int32(data=0)`（或删除该发布，视需要）
  - 验证：节点启动后不再报 AttributeError，`/rl_planner/local_offset` 正常发布

- [x] Task 2: 端到端仿真验证
  - 启动仿真: `./scripts/start_sim.sh lqr --with-perception --obstacle "3,0,2.5"`
  - 确认 `perception_node` 正常发布 `/obstacle_features`
  - 确认 `rl_planner_node` 正常发布 `/rl_planner/local_offset`（障碍物存在时偏移量非零）
  - 确认 LQR 跟踪器收到偏移量并修正轨迹，无人机避开障碍物

# Task Dependencies
- Task 2 依赖 Task 1
