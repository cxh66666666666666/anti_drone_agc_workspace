# Tasks

- [x] Task 1: 修复非紧急避障方向策略
  - 修改 `compute_escape_offset()` 非紧急分支：找到最近障碍物扇区索引 `min_idx`，向其左右两侧（90° 方向）分别采样扇区距离，选择更空旷的一侧
  - 逃逸角度 = 最近障碍物方向 ± 90°，取空旷侧
  - 保持原有偏移幅度计算不变
  - 验证：前方障碍物 + 右侧更空 → 向右侧偏移；前方障碍物 + 左侧更空 → 向左侧偏移

- [x] Task 2: 紧急避障增强
  - 将 `max_offset_xy` 默认值从 2.0 提升到 3.0（或通过参数可配）
  - 紧急避障时同时发布减速信号：在 offset 中反映"接近速度过快"的意图（如减小前向偏移分量）
  - 验证：紧急避障时偏移量足够让无人机快速脱离危险区域

- [x] Task 3: LQR 偏移量安全恢复
  - 在 `localOffsetCallback` 中检测零偏移量：当 offset 的 x/y/z 均为 0 时，将 `has_rl_offset_` 设为 false，`rl_offset_` 归零
  - 在 `computeLQRControl` 中，当进入安全状态时重置累积
  - 验证：障碍物消失后 `/rl_planner/local_offset` 发布 (0,0,0)，LQR 正确停止叠加偏移

- [x] Task 4: 控制权交接
  - 在 `TakeoffController` 中添加 `releaseControl()` 公开方法
  - 添加 `/drone_launch_controller/release_control` 服务（`std_srvs/Trigger`）
  - LQR 跟踪器在 `run()` 中 MAVROS 连接后调用该服务
  - 验证：LQR 启动后 `drone_launch_controller` 不再向 `/mavros/setpoint_position/local` 发布

# Task Dependencies
- Task 1, 2, 3 互不依赖，可并行
- Task 4 依赖 Task 2 的 LQR 侧准备好接管
