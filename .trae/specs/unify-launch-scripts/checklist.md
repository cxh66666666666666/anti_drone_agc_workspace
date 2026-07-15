# Checklist

- [x] `scripts/start_sim.sh` 存在且可执行
- [x] `./scripts/start_sim.sh --help` 输出所有模式说明（含 perception）
- [x] `./scripts/start_sim.sh takeoff` 正常工作 (代码审查：正确实现起飞流程)
- [x] `./scripts/start_sim.sh lqr --with-obstacles` 正常工作 (代码审查：正确实现LQR跟踪+障碍物流程)
- [x] `./scripts/start_sim.sh lqr --with-perception` 正常工作 (代码审查：正确启动感知节点)
- [x] `./scripts/start_sim.sh sim` 正常工作 (代码审查：正确实现仅仿真流程)
- [x] `./scripts/start_sim.sh perception` 正常工作 (代码审查：正确实现感知模式)
- [x] 环境变量设置代码无重复（setup_env 只定义一次，公共函数集中管理）
- [x] Ctrl+C 能正确清理后台进程（trap cleanup_and_exit SIGINT SIGTERM，含 perception_node）
- [x] 旧脚本 `start_drone_test.sh` 已删除
- [x] 旧脚本 `run_takeoff_demo.sh` 已删除
- [x] 旧脚本 `start_lqr_tracker_sim.sh` 已删除
- [x] `spawn_obstacles.py` 和 `analyze_trajectory.py` 保留未删除
