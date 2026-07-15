# Checklist

- [x] `lqr_tracker_node.h` 包含 `flown_path_pub_` 和 `flown_path_marker_` 成员声明
- [x] `lqr_tracker_node.cpp` 构造函数中正确初始化 LINE_STRIP Marker（红色, frame_id="map"）
- [x] `lqr_tracker_node.cpp` 的 `trackingLoop()` 中每次迭代追加位置点并发布 Marker
- [x] `lqr_tracker.rviz` 包含 `/lqr_tracker/flown_path` 红色 LINE_STRIP 显示
- [x] `lqr_tracker.rviz` 包含 `/rl_planner/local_goal` 黄色 Pose 显示
