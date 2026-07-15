# Tasks

- [x] Task 1: 创建统一启动脚本 `scripts/start_sim.sh`
  - 支持 `--help` / `-h` 显示帮助
  - 支持 `takeoff` 模式（替代 `start_drone_test.sh` 和 `run_takeoff_demo.sh`）
  - 支持 `lqr` 模式（替代 `start_lqr_tracker_sim.sh`）
  - 支持 `perception` 模式（独立启动 rl_obstacle_avoidance 感知节点）
  - 支持 `sim` 模式（仅启动PX4 SITL + MAVROS）
  - 公共逻辑（环境变量设置、进程清理、MAVROS等待）抽取为函数，避免重复
  - 支持 Ctrl+C 信号处理，清理后台进程
  - 参数通过命令行传递（如 `--altitude`, `--trajectory`, `--with-obstacles`, `--with-perception`）

- [x] Task 2: 删除旧启动脚本
  - 删除 `scripts/start_drone_test.sh`
  - 删除 `scripts/run_takeoff_demo.sh`
  - 删除 `scripts/start_lqr_tracker_sim.sh`

# Task Dependencies
- Task 2 依赖于 Task 1
