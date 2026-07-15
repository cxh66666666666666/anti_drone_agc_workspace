# Checklist

- [x] `plan_once()` 不再调用未定义的 `choose_action()` 和 `action_to_offset()`
- [x] `plan_once()` 正确调用 `compute_escape_offset()` 获取 body 帧偏移量
- [x] `features` 为 None 时有防护检查，不会因 NoneType 报错
- [x] body 帧到 map 帧的旋转变换逻辑保持不变（使用 `_quat_to_yaw`）
- [x] 避障偏移量通过 `/rl_planner/local_offset` 正确发布
- [x] LQR 跟踪器能正常订阅 `/rl_planner/local_offset` 并叠加偏移到参考轨迹
