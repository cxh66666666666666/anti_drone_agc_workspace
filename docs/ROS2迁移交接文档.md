# ROS2 迁移交接文档

> 状态：**迁移前基线**（ROS1 参考实现冻结，作为对照）
> 日期：2026-10-09
> 目标：Ubuntu 24.04 + ROS 2 Jazzy + Nav2 全面重构
> 适用范围：迁移前的盘点、选型、风险与路线图；不含具体代码改造。

---

## 1. 文档目的

本文档是 ROS1(Noetic) → ROS2(Jazzy) 迁移的**交接基线**，用于：

1. 锁定迁移前的现状（包/节点/消息/接口/依赖），避免迁移中"事实漂移"。
2. 给出 ROS1→ROS2 的依赖与概念映射表，作为逐项改造的清单。
3. 明确不可丢失的**硬约束**（坐标系、唯一执行口、成功定义等），迁移后必须逐条复验。
4. 给出 Nav2 的集成边界、目标技术栈与分阶段路线图及验收标准。
5. 标出仍需用户拍板的**待决策项**。

---

## 2. 现状盘点（迁移前）

### 2.1 运行环境

| 项 | 现状 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS (focal) |
| ROS 发行版 | ROS1 Noetic |
| 构建系统 | catkin_tools（`catkin build`，产物在 `build/`、`devel/`） |
| 仿真平台 | Gazebo Classic（仓库内 vendored `gazebo_ros_pkgs`） |
| 飞控仿真 | PX4 SITL（机型 `iris_3d_lidar`） |
| 中间件 | MAVROS（`/mavros/*`） |
| 时钟 | 全局 `/use_sim_time=true` |
| 语言分布 | C++ 文件约 140 个；Python 文件约 87 个 |

### 2.2 包清单与职责

| 包 | 语言 | 职责 | 迁移处置 |
|---|---|---|---|
| `air_ground_core` | C++ | **空地协同核心**：感知融合、任务状态机、地面规划/执行、日志与评估 | 重点重构，保留接口语义 |
| `lqr_trajectory_tracker` | C++ | LQR 轨迹跟踪（linear/circular/helical/8字/step 悬停） | 迁移为 rclcpp，保留 |
| `drone_launch_controller` | C++ | 起飞/解锁/健康检查/模式切换 | 迁移，衔接 PX4 新链路 |
| `car_launch_controller` | C++ | 旧地面控制器 | **已废弃**，不迁移 |
| `rl_obstacle_avoidance` | Python | 无人机感知 + 规则避障 + A* + DQN 训练 | 规划部分由 Nav2 替代；RL 保留为离线对照 |
| `rl_ground_avoidance` | Python | 地面车 DQN 避障（镜像） | 同上，地面规划迁 Nav2 |
| `gazebo_ros_pkgs` | C++ | Gazebo Classic ROS 集成（vendored） | **删除**，改 `ros_gz` |
| `velodyne_gazebo_plugins` | C++ | Velodyne Gazebo **Classic** 插件 | **删除**，改 gz-sim 传感器插件 |

### 2.3 协同核心节点清单（`air_ground_core`，全部 C++）

| 节点 | 职责 | 迁移要点 |
|---|---|---|
| `air_odom_adapter` / `ground_odom_adapter` | 位姿适配（统一到 `agc_world`） | 帧名不变，改 rclcpp/tf2 |
| `gt_adapter` | 真值目标源 | 依赖 gazebo_msgs → 改 ros_gz 接口 |
| `air_perception` / `air_fusion` | 真值派生感知仿真 + 多源融合 | 逻辑保留，参数改 ros2 param |
| `map_adapter` | 地图/代价地图适配 | 与 Nav2 costmap 对接重做 |
| `task_manager` | 任务状态机（含终态语义） | 语义必须逐条复验 |
| `ground_planner` | 地面规划（规则/A*） | **由 Nav2 替换** |
| `ground_commander` / `ground_drive_adapter` | 地面指令与执行 | 唯一执行口 `/agc/ground/cmd_vel` 不变 |
| `intruder_driver` | 入侵目标驱动 | 迁移 |
| `scenario_reset` | 场景复位（Gazebo 交互） | Gazebo Classic API → ros_gz |
| `agc_run_logger` | 运行日志/评估记录 | 迁移，保留指标口径 |
| `air_control_bridge` | 空中控制权交接核查 | 与 PX4 新链路衔接 |
| `air_follow` | 无人机跟随（安全距离带 2.0–3.5m） | 迁移 |
| `car_commander` | 旧地面执行口 | **已废弃**，不迁移 |

> 其余节点：`rl_obstacle_avoidance`：`perception_node`、`astar_planner`、`rl_planner_node`、`experiment_logger_node`、`obstacle_visualizer`、`gazebo_env`；`rl_ground_avoidance`：`ground_env`、`ground_perception`。

### 2.4 消息与服务

| 包 | 消息 | 服务 |
|---|---|---|
| `air_ground_core` | `AgcTaskState`、`AirObservationArray`、`ObstacleObservation`、`ObstacleTrack`、`ObstacleTrackArray`、`SensorObservation`、`TargetObservation`、`TargetTrack`、`TargetTrackArray` | — |
| `drone_launch_controller` | `DroneHealthStatus`、`FlightStatus` | `ArmMotors`、`ConnectionStatus`、`DroneHealthCheck`、`Takeoff` |
| `car_launch_controller` | `CarHealthStatus`、`CarStatus` | `CarHealthCheck`、`ConnectionStatus`、`Drive`、`EnableMotion` |
| `rl_obstacle_avoidance` | `ObstacleFeatures` | `PlanPath` |

### 2.5 关键接口与不可丢失的硬约束

**接口/命名空间**
- 协同层统一命名空间 `/agc/`；地面唯一执行口 `/agc/ground/cmd_vel`。
- 任务门控：`/agc/task/start`、状态 `/agc/task/state`（`AgcTaskState`）。
- 场景复位：`/agc/scenario/reset`；`scenario_reset` 运行期门控。
- 参数入口（由 launch 单一设置，节点不得覆写）：`/agc/target/source`、`/agc/obstacles/source`、`/agc/perception/sensors_file`、`/agc/air/takeoff_altitude_m`、`/agc/air/follow/*`、`/agc/scenario/dynamic_obstacles`、`/agc/world_file`、`/agc/log/seed`。
- 空中控制：起飞阶段 `drone_launch_controller`，`release_control` 后由 `lqr_tracker_node` 接管。
- 坐标系基准：`agc_world` 为唯一基准；机体系固定为 `drone/base_link`、`car/base_link`（**禁止裸 `base_link`**）。
- PX4 SITL 会一次性发布 `map`/`odom`/`base_link_frd` 等历史静态帧，协同层不发布裸 `base_link`。

**硬约束（迁移后必须逐条复验）**
1. 8 字轨迹全程恒速、无加减速；角速度 0.1（线速度 0.3 m/s）。
2. 距离阈值顺序必须满足 紧急 < 碰撞 < 安全（默认 0.7m < 0.9m < 2.4m）。
3. 仿真运行口径：实证/验收**使用非无头（Gazebo GUI）**；默认仍无头以向后兼容。
4. 训练与评估成功定义统一：超时一律算失败，只有显式完成才算成功。
5. 每个 episode 起始必须把轨迹相位复位到 t=0。
6. 安全滤波器参数须用**真实逃逸能力**（如 `lateral_speed = min(max_offset_xy, max_offset_change/dt)`）。
7. 场景生成须保证出生间距 ≥2.4m；复位失败最多重试 3 次且不占 episode 预算。
8. 状态机终态为"抵近并持续跟踪"，`SUCCESS` **不得**表述为"拦截完成"。
9. 评估报告须含 `shield_infeasible_rate` 与 `reset_failure_rate` 指标。
10. 无人机跟随默认关闭（`--follow` 开启）；跟随要求目标新鲜（`ref_max_age_s=0.5`）且 `confidence≥0.5`，过期即冻结。

### 2.6 启动编排

- 唯一 run 入口：`src/air_ground_core/launch/agc_collab.launch`（XML）+ `scripts/start_agc_collab.sh`。
- 单体实验入口：`scripts/start_sim.sh`（791 行，sim/takeoff/lqr/perception 四模式）。
- 已知竞态（需在 ROS2 版修复）：`/agc/task/start` 门禁先校验 odom 再调用 `/agc/scenario/reset`，导致 `odom_valid` 被无条件置 false，`evaluate()` 读到 `ODOM_INVALID`。
- 已修复教训（需保留）：`ground_drive_adapter` 非运行期仍写车体位姿会覆盖 `car_start`，必须用 `/agc/task/state` 运行期门控。

---

## 3. 目标技术栈

| 项 | 目标 |
|---|---|
| 操作系统 | Ubuntu 24.04 LTS (noble) |
| ROS 发行版 | ROS 2 **Jazzy Jalisco**（24.04 对应 LTS） |
| 构建系统 | colcon + ament_cmake / ament_python |
| 仿真平台 | **GZ Sim (Harmonic)** + `ros_gz` |
| 飞控链路 | PX4 SITL + **micro-XRCE-DDS Agent + `px4_msgs`**（替换 MAVROS） |
| 导航 | **Nav2**（地面 2D） |
| 通信 | DDS（QoS 需按实时性/可靠性逐话题标定） |

> **硬约束**：Gazebo Classic 已 EOL（2025-01）且 24.04 无官方包，`gazebo_ros_pkgs` 与 `velodyne_gazebo_plugins` 必须整体替换，不能沿用。

---

## 4. ROS1 → ROS2 映射表

| ROS1 | ROS2 | 说明 |
|---|---|---|
| `catkin` / `catkin_tools` | `ament_cmake` / `ament_python` / colcon | `package.xml` 升 format 3 |
| `roscpp` | `rclcpp` | 16 个核心节点逐个改写 |
| `rospy` | `rclpy` | RL/工具脚本 |
| `ros::init` / `ros::NodeHandle` | `rclcpp::init` / `Node` | 生命周期与 executor 需重设 |
| `message_generation` / `message_runtime` | `rosidl_default_generators` / `rosidl_default_runtime` | msg/srv 用 `.msg`/`.srv` + rosidl |
| XML `.launch` + `rosparam` | `launch.py` + `parameters=[{}]` | 参数 YAML 结构重排 |
| `ros::param` / `getParam` | 声明式参数 `declare_parameter` | 需显式声明与类型 |
| `tf` / `tf2_ros` | `tf2_ros`（API 变动） | 帧约定不变 |
| `/use_sim_time` | `use_sim_time` 参数 + `/clock` | ros_gz 桥接 |
| `gazebo_msgs`（Get/SetModelState 等） | `ros_gz_interfaces` / `ros_gz_sim` | `gt_adapter`、`scenario_reset` 重写 |
| `mavros_msgs` | `px4_msgs` + micro-XRCE-DDS | `air_control_bridge`、`drone_launch_controller` 重写 |
| Gazebo Classic 传感器插件 | gz-sim 插件 + `ros_gz_bridge`（`sensor_msgs/PointCloud2`） | Velodyne 链路重做 |
| `rosrun` / `roslaunch` | `ros2 run` / `ros2 launch` | 脚本全面改写 |
| `rostopic` / `rosservice` / `rosbag` | `ros2 topic` / `service` / `bag` | 评测脚本与 CI 脚本改写 |

---

## 5. 风险清单

| 级别 | 风险 | 影响 | 缓解 |
|---|---|---|---|
| 高 | Nav2 只覆盖地面 2D；无人机 3D 跟踪无现成方案 | 空中链路不能靠 Nav2 解决 | 空中保留 LQR + 自研避障；Nav2 仅地面（见 §6） |
| 高 | PX4 链路由 MAVROS 换为 micro-XRCE-DDS | 控制时序/模式切换需重调（切 OFFBOARD 持续发 setpoint、`release_control` 等待由 0.5s→0.05s） | 单独里程碑，先打通悬停再接管 |
| 高 | Gazebo Classic → GZ Sim 语义差异大 | 世界/模型/插件/传感器全部重做 | 先迁移 `agc_world` 与车/UAV 模型，再补传感器 |
| 中 | 真值派生感知仿真依赖 gazebo 真值接口 | `air_perception`/`air_fusion` 数据源改写 | 用 `ros_gz` 真值桥替代 |
| 中 | QoS 默认不兼容（如 sensor/reliable） | 话题收不到/丢包 | 逐话题标定 QoS profile |
| 中 | 行为树/生命周期节点引入 | `task_manager` 状态机需重新映射 | 决定是否改用 Nav2 BT |
| 中 | 时序与实时性（单线程/多线程 executor） | 频率与数值漂移 | 迁移后**同场景实跑对比频率/数值**，未验证不得删旧实现 |
| 低 | `package.xml`/依赖声明不完整 | 构建失败 | colcon 逐包构建定位 |
| 低 | 中文路径（docs 下）跨平台 | 工具链兼容 | 保持 UTF-8 |

---

## 6. Nav2 集成边界

- **地面车（适配）**：Nav2 承担 costmap、全局/局部规划器、控制器、行为树；替换 `rl_ground_avoidance` 的 A*/规则避障与 `air_ground_core/ground_planner` 的规划角色。
  - 执行口仍收敛到 `/agc/ground/cmd_vel`（Nav2 控制器输出需转接到该唯一执行口）。
  - `map_adapter` 需产出/对接 Nav2 `OccupancyGrid` 与 costmap 图层。
- **无人机（不适用）**：Nav2 无 3D 规划器；保留 LQR 轨迹跟踪 + 自研避障 + `air_follow` 跟随；仅可复用 Nav2 的 BT/生命周期模式作为架构参考。
- **待定**：是否将 `task_manager` 迁移为 Nav2 行为树（BT）节点体系。

---

## 7. 分阶段路线图与验收

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| P0 环境 | 24.04 安装 ROS2 Jazzy、colcon、ros_gz、Nav2、PX4 + micro-XRCE | `ros2 topic list` 正常；GZ Sim 空世界可起 |
| P1 消息与骨架 | `air_ground_core` 新建 colcon 包、rosidl 生成 9 个 msg、空节点编译通过 | `ros2 interface show` 与 ROS1 定义逐字段一致 |
| P2 仿真链路 | `agc_world` + 车/UAV 模型迁 GZ Sim；ros_gz 桥；PX4 悬停闭环 | 无头/GUI 均可起飞悬停，`/clock` 同步 |
| P3 感知与适配 | odom/gt/perception/fusion/map 适配节点迁移 | 各话题频率与 ROS1 同场景一致（±容差） |
| P4 导航重做 | 地面接 Nav2；`/agc/ground/cmd_vel` 唯一执行口保持 | 地面按目标点自主导航，无第二写入者 |
| P5 任务与评估 | `task_manager`、`agc_run_logger`、场景与评估口径迁移 | §2.5 十条硬约束逐条复验通过；含 shield/reset 指标 |
| P6 冻结对照 | 新旧同场景实跑对比，出具对比报告 | 频率/数值差异在容差内，方可停用 ROS1 参考实现 |

> 每个阶段结束需对 §2.5 硬约束做一次回归；P1 之前不动 ROS1 代码（只读参照）。

---

## 8. 待决策项

1. **PX4 链路**：micro-XRCE-DDS + `px4_msgs`（推荐）还是 `mavros2` 桥接。
2. **工作空间形态**：新建独立 ROS2 工作空间（如 `ros2_ws/`，ROS1 只读对照）还是原地分层改造。
3. **Nav2 范围**：仅地面车，或为无人机自定义 3D 规划/控制器插件。
4. **`task_manager` 架构**：自研状态机还是改用 Nav2 行为树。
5. **RL 去留**：DQN 作为在线规划（与 Nav2 并存）还是转为离线对照基线。
6. **实机/纯仿真**：是否要求迁移后支持真机（影响 PX4 链路与传感器口径）。

---

## 9. 关键文件索引

| 文件 | 说明 |
|---|---|
| `README.md` | 项目总览与技术栈（含已过期的目录描述，迁移时同步更新） |
| `src/air_ground_core/launch/agc_collab.launch` | 唯一 run 入口 |
| `src/air_ground_core/config/agc_params.yaml` | 协同层参数总表 |
| `src/air_ground_core/config/perception_sensors.yaml` | 感知通道（真值派生感知仿真）配置 |
| `src/air_ground_core/config/scenario_*.yaml` | 场景定义（12 个） |
| `src/air_ground_core/msg/*.msg` | 9 个协同层消息定义 |
| `src/air_ground_core/worlds/agc_world*.world` | Gazebo 世界（迁移 GZ Sim） |
| `scripts/start_sim.sh`、`scripts/start_agc_collab.sh` | 启动编排脚本 |
| `scripts/verify_*.py` | 各里程碑验收脚本（口径需随迁移更新） |

---

## 10. 迁移前已完成的准备工作

1. 全仓库现状盘点（包/节点/消息/依赖/体积）。
2. ROS1→ROS2 映射表与风险清单（§4、§5）。
3. 硬约束清单固化（§2.5），作为迁移后回归基线。
4. Nav2 集成边界界定（§6）。
5. 分阶段路线图与验收标准（§7）。
6. 仓库卫生：`.gitignore` 补充大型生成产物（`experiments/`、`.pytest_cache/`），确保迁移基线可提交、可推送。