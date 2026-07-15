# Tasks

- [x] Task 1: 创建 experiment_logger_node.py 实验数据记录节点
  - [x] 订阅 5 个话题：`/mavros/local_position/pose`、`/rl_planner/local_offset`、`/rl_planner/action`、`/lqr_tracker/position_error`、`/lqr_tracker/adjusted_ref`
  - [x] 实现 CSV 文件自动创建（带时间戳文件名）、表头写入
  - [x] 实现定时器回调（默认 10 Hz，可配置），汇总最新缓存数据写入一行
  - [x] 计算衍生字段：`is_avoiding`、`offset_magnitude_xy`
  - [x] 优雅降级：未收到消息的字段填 NaN
  - [x] 添加 ROS 参数：`output_dir`（默认 `/tmp/experiment_logs`）、`record_rate`（默认 10 Hz）

- [x] Task 2: 创建 experiment_logger.launch 启动文件
  - [x] 放在 `src/rl_obstacle_avoidance/launch/` 下
  - [x] 包含 `output_dir` 和 `record_rate` 可配置参数

- [x] Task 3: 修改 CMakeLists.txt 添加安装项
  - [x] 在 `catkin_install_python` 中添加 `experiment_logger_node.py`

# Task Dependencies
- Task 2 依赖于 Task 1（需要知道节点名称和参数名）
- Task 3 依赖于 Task 1（需要知道脚本文件名）
