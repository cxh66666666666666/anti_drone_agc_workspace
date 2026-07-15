# Checklist

- [x] `start_sim.sh` 中 `run_lqr()` 函数在 `--with-perception` 时启动 `rl_planner_node.launch`
- [x] `start_sim.sh` 中 `cleanup_procs()` 包含 `rl_planner_node` 的清理
- [x] `start_sim.sh` 中 `cleanup_and_exit()` 包含 `rl_planner_node` 的清理
- [x] `start_sim.sh` 中不再包含 `--with-obstacles` 选项解析
- [x] `start_sim.sh` 中 `WITH_OBSTACLES` 变量已移除
- [x] `start_sim.sh` 中 `run_lqr()` 不再包含障碍物加载代码
- [x] `start_sim.sh` 帮助信息不再提及 `--with-obstacles`
- [x] `scripts/spawn_obstacles.py` 已删除
