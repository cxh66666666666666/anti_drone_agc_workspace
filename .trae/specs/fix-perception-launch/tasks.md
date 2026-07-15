# Tasks

- [x] Task 1: 在 `start_sim.sh` 的 `run_lqr()` 中添加 `rl_planner_node` 启动
  - [x] SubTask 1.1: 在 `with_perception` 块中，`perception.launch` 之后，增加 `roslaunch rl_obstacle_avoidance rl_planner_node.launch &` 并记录 PID
  - [x] SubTask 1.2: 在 `cleanup_procs()` 和 `cleanup_and_exit()` 中增加 `pkill -9 -f rl_planner_node` 清理

- [x] Task 2: 从 `start_sim.sh` 移除 `--with-obstacles` 选项
  - [x] SubTask 2.1: 移除 `run_lqr()` 中的障碍物加载逻辑（`if [ "$WITH_OBSTACLES" = "true" ]` 块）
  - [x] SubTask 2.2: 移除参数解析中的 `--with-obstacles` case 分支
  - [x] SubTask 2.3: 移除 `WITH_OBSTACLES` 默认变量及相关引用
  - [x] SubTask 2.4: 更新帮助信息，删除 `--with-obstacles` 描述和相关示例

- [x] Task 3: 删除 `scripts/spawn_obstacles.py`

# Task Dependencies
- Task 2 和 Task 3 相互独立，可并行处理
- Task 1 和 Task 2 均修改 `start_sim.sh`，建议合并为一个原子编辑操作
