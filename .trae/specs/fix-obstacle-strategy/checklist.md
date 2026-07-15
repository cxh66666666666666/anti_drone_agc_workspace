# Checklist

- [x] 非紧急避障不再选择"第一个"空扇区，而是选择障碍物两侧中更空旷的一侧
- [x] 紧急避障偏移量不低于 3.0m，足够快速脱离危险区域
- [x] 障碍物消失后 `rl_offset_` 正确归零，轨迹恢复正常跟踪
- [x] LQR 跟踪时 `drone_launch_controller` 不再同时发布 setpoint
- [x] 无障碍时 offset 持续为 0，有障碍时偏移方向正确绕行
- [x] `catkin build` 编译通过，无新增警告
