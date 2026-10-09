# LQR 轨迹跟踪器 (lqr_trajectory_tracker)

LQR 最优控制轨迹跟踪节点，驱动 PX4 无人机在 OFFBOARD 模式下沿预设轨迹飞行。

## 架构

```
/mavros/local_position/pose ──→ poseCallback()      ┌──────────────────┐
/mavros/state                ──→ stateCallback()     │  LQRTrackerNode  │
/rl_planner/local_offset     ──→ localOffsetCallback │                  │
                                    ↓                 │  trackingLoop()  │
                             ┌──────────────┐        │    ↓             │
                             │ loadParameters│        │  computeLQR()    │
                             └──────┬───────┘        │    ↓             │
                                    ↓                 │  publishSetpoint │
                             ┌──────────────┐        └──────┬───────────┘
                             │ generateTraj  │               ↓
                             └──────────────┘  /mavros/setpoint_position/local
```

- **职责单一**：仅做轨迹跟踪，不负责起飞/解锁/模式切换（由 `drone_launch_controller` 提供）
- 启动后等待 MAVROS 连接，连接成功后立即进入跟踪循环
- 可叠加 `/rl_planner/local_offset` 局部避障偏移（默认只修改位置参考，不转速度前馈）

## 输入输出

### 订阅话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 无人机当前位置 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | 无人机当前速度（LQR 反馈量） |
| `/mavros/state` | `mavros_msgs/State` | MAVROS 连接与飞行状态 |
| `/rl_planner/local_offset` | `geometry_msgs/Vector3Stamped` | RL/规则避障的位置参考偏移（map 系），叠加到参考轨迹；超时自动清零 |
| `/rl_planner/local_control` | `geometry_msgs/Vector3Stamped` | 速度缩放 / 暂停 / 相位方向指令（x=速度缩放, y>0.5=暂停, z<-0.5=反向） |
| `/lqr_tracker/waypoints` | `nav_msgs/Path` | A* 全局路径航点（仅 `waypoints` 轨迹类型使用） |

### 服务

| 服务 | 类型 | 用途 |
|------|------|------|
| `/lqr_tracker/reset_trajectory` | `std_srvs/Trigger` | 重置轨迹起始时间，从头开始跟踪当前轨迹 |
| `/lqr_tracker/reload_trajectory` | `std_srvs/Trigger` | 重新读取几何参数并生成轨迹（换场景用），相位复位到 t=0 |

### 发布话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | PX4 OFFBOARD 位置指令 |
| `/lqr_tracker/accel_cmd` | `geometry_msgs/AccelStamped` | LQR 加速度控制量（调试用） |
| `/lqr_tracker/ref_trajectory` | `nav_msgs/Path` | 参考轨迹可视化（RViz） |
| `/lqr_tracker/adjusted_ref` | `geometry_msgs/PoseStamped` | 叠加局部避障偏移后的参考位置 |
| `/lqr_tracker/reference_velocity` | `geometry_msgs/Vector3Stamped` | 参考轨迹速度（供感知/规划节点使用） |
| `/lqr_tracker/reference_mid_lookahead_velocity` | `geometry_msgs/Vector3Stamped` | 前视时间中点的轨迹切线速度 |
| `/lqr_tracker/reference_lookahead_velocity` | `geometry_msgs/Vector3Stamped` | 前视终点的轨迹切线速度 |
| `/lqr_tracker/flown_path` | `visualization_msgs/Marker` | 实际飞行轨迹可视化（RViz） |
| `/lqr_tracker/position_error` | `std_msgs/Float64` | 位置跟踪误差 [m] |
| `/lqr_tracker/velocity_error` | `std_msgs/Float64` | 速度跟踪误差 [m/s] |
| `/lqr_tracker/total_error` | `std_msgs/Float64` | 综合跟踪误差 |

## 运行

```bash
# 仿真环境（需先启动 PX4 SITL + MAVROS）
roslaunch lqr_trajectory_tracker lqr_tracker_sim.launch

# 切换轨迹类型（命令行覆盖参数）
roslaunch lqr_trajectory_tracker lqr_tracker_sim.launch trajectory_type:=circular
```

统一启动脚本：`scripts/start_sim.sh lqr`（一键启动 PX4 SITL + Gazebo + MAVROS + 起飞 + LQR Tracker，
支持 `--trajectory`、`--with-perception`、`--with-experiment-logger` 等选项）。

## 参数

所有参数在 `config/default_params.yaml`，按 7 组管理：

| 组 | 说明 |
|----|------|
| 控制参数 | 轨迹类型、总时长、控制频率、RL 偏移速度前馈（`rl_velocity_gain`、`max_rl_velocity`） |
| LQR 权重 | 位置/速度误差权重、控制量权重 |
| 直线轨迹 | 起终点坐标 |
| 圆形/螺旋 | 圆心、半径、高度、角速度 |
| 固定水平轨迹 | 左右圆心、半径、高度、角速度 |
| Gerono 八字 | 中心、X/Y 半径、高度、角速度 |
| 悬停轨迹 (step) | 悬停位置（供 RL 训练使用，位移完全由 `/rl_planner/local_offset` 驱动） |

## 轨迹类型

| 类型 | 关键字 | 说明 |
|------|--------|------|
| 直线 | `linear` | 起点到终点，两端有加/减速段 |
| 圆周 | `circular` | 定高匀速圆周 |
| 螺旋 | `helical` | 圆周叠加定高变化 |
| 固定水平轨迹 | `fixed_horizontal` | 半圆+直线组成的水平跑道/八字形，全程恒速 |
| Gerono 八字 | `figure_eight` | `x=r_x·sinθ, y=r_y·sinθ·cosθ` 参数曲线 |
| 悬停 | `step` | 固定位置、零速度/零加速度 |
| 航点 | `waypoints` | 由 `/lqr_tracker/waypoints` 输入，按弧长等速重采样，不循环 |

## 相关脚本

实验分析入口为 `scripts/evaluate_experiment.py`（配合 `experiment_logger_node` 的 CSV 输出计算六项评价指标）。

## 依赖

- ROS Noetic
- mavros_msgs
- visualization_msgs（`/lqr_tracker/flown_path` 可视化）
- Eigen3
- PX4 SITL（运行时需要）
