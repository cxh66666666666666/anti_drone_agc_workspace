# Tasks

- [x] Task 1: 在 `lqr_tracker_node.h` 中添加飞行轨迹 Marker 相关的成员
  - [x] 添加 `visualization_msgs/Marker.h` 头文件
  - [x] 添加 `ros::Publisher flown_path_pub_` 成员
  - [x] 添加 `visualization_msgs::Marker flown_path_marker_` 成员（LINE_STRIP，红色）

- [x] Task 2: 在 `lqr_tracker_node.cpp` 中实现飞行轨迹 Marker 发布逻辑
  - [x] 构造函数中初始化 Marker（LINE_STRIP, frame_id="map", 红色, line_width=0.1）
  - [x] 构造函数中创建 publisher（`/lqr_tracker/flown_path`, queue_size=10）
  - [x] `trackingLoop()` 中每次控制迭代将 `current_state_.position` 追加到 Marker points 并 publish

- [x] Task 3: 更新 RViz 配置文件
  - [x] 添加 `/lqr_tracker/flown_path` 的 Marker 显示（红色 LINE_STRIP）
  - [x] 添加 `/rl_planner/local_goal` 的 Pose 显示（黄色小球，表示 RL 局部目标）

# Task Dependencies
- Task 2 依赖 Task 1
- Task 3 可与 Task 1、2 并行
