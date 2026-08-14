# 反无人机空地协同项目

简体中文 | [English](./README.en.md)

## 项目简介

anti_drone_agc_workspace 是一个基于 ROS 的反无人机空地协同系统实验平台。该项目集成了无人机控制、激光雷达 SLAM、LQR 轨迹跟踪控制、强化学习（DQN）局部避障以及 Gazebo 仿真等核心技术，旨在研究和验证空地协同探测与跟踪任务的相关算法。

### 核心功能

- **无人机发射与控制**：通过 `drone_launch_controller` 实现无人机的安全起飞、降落和模式切换
- **激光雷达 SLAM**：集成 A-LOAM 实现高精度地图构建
- **LQR 轨迹跟踪**：基于线性二次调节器的轨迹跟踪控制器，支持直线、圆周、螺旋、固定水平 8 字形、悬停（step）等轨迹
- **规则避障**：基于扇区距离的视觉避障，支持单障碍绕行和多障碍连续绕行
- **DQN 强化学习避障**：局部避障训练闭环（感知 → 动作 → LQR 偏移），含经验回放、贪心评估与模型管理
- **实验数据记录与评估**：`experiment_logger_node` 记录飞行数据，`scripts/evaluate_experiment.py` 计算六项评价指标
- **障碍物生成与分析**：提供轨迹分析和障碍物生成的辅助脚本
- **Gazebo 仿真支持**：完整的 PX4 SITL + Gazebo 虚拟仿真环境

### 技术栈

| 类别 | 技术 |
|------|------|
| 操作系统 | Ubuntu 20.04 + ROS Noetic |
| 仿真平台 | Gazebo + PX4 SITL（iris_3d_lidar 机型，兼容 XTDrone 模型库） |
| 传感器 | Velodyne VLP-16/HDL-32/HDL-64 激光雷达 |
| 飞控固件 | PX4 |
| 中间件 | MAVROS |
| 核心算法 | A-LOAM、LQR、DQN（PyTorch，Dueling + Double DQN + 优先经验回放） |
| 路径规划 | 扇区感知 + 规则避障 / DQN 局部避障 |

## 目录结构

```
anti_drone_agc_workspace/
├── docs/                          # 文档目录
│   ├── 指导书/                    # 课程指导书（《基于强化学习的无人机自主避障与路径规划系统指导书》）
│   ├── learning_plan.md           # 后续研发与学习规划
│   ├── dqn_training.md            # DQN 避障训练说明
│   ├── faq_log_issues.md          # ROS 日志常见问题与解决方法
│   └── project_guide.md           # 项目开发指南（规划中）
├── scripts/                       # 辅助脚本
│   ├── start_sim.sh               # 统一仿真启动脚本（sim/takeoff/lqr/perception 四种模式）
│   ├── analyze_trajectory.py      # 轨迹分析与安全障碍物位置生成
│   ├── evaluate_experiment.py     # 实验 CSV 分析，计算六项评价指标
│   └── generate_pptx.py           # 项目汇报 PPT 生成脚本
├── experiment_data/               # experiment_logger_node 输出的实验 CSV
├── training_runs/                 # DQN 训练运行目录（配置、日志、模型）
├── logs/                          # 仿真运行日志
└── src/                           # 源代码工作空间
    ├── A-LOAM/                    # 激光雷达 SLAM 算法包（VLP-16/HDL-32/HDL-64）
    ├── air_ground_core/           # 空地协同核心包（地面移动平台控制节点）
    ├── drone_launch_controller/   # 无人机发射控制器（连接监控/自检/解锁/起飞）
    ├── lqr_trajectory_tracker/    # LQR 轨迹跟踪器
    ├── rl_obstacle_avoidance/     # 感知 + 规则避障 + DQN 训练/评估包
    ├── gazebo_ros_pkgs/           # Gazebo ROS 集成包
    ├── velodyne_description/      # Velodyne 模型描述
    ├── velodyne_gazebo_plugins/   # Velodyne Gazebo 传感器插件
    └── velodyne_simulator/        # Velodyne 仿真元包
```

## 快速启动

统一启动脚本 `scripts/start_sim.sh` 支持四种模式：

| 模式 | 命令 | 说明 |
|------|------|------|
| 仿真环境 | `./scripts/start_sim.sh sim` | 仅启动 PX4 SITL + Gazebo + MAVROS |
| 起飞 | `./scripts/start_sim.sh takeoff --altitude 5.0` | 仿真 + 起飞控制器 + 起飞 |
| LQR 跟踪 | `./scripts/start_sim.sh lqr --with-perception` | 仿真 + 感知/避障 + 起飞 + LQR 轨迹跟踪 |
| 感知 | `./scripts/start_sim.sh perception` | 仿真 + 感知节点 |

常用选项：`--trajectory <类型>`（linear/circular/helical/fixed_horizontal/step）、`--with-perception`（启动感知+规则规划节点）、`--training`（仅感知，供 DQN 独占控制）、`--with-experiment-logger`（启动实验数据记录节点）、`--obstacle <x,y,z>`（spawn 静态障碍物，可多次指定）、`--headless`（禁用 Gazebo GUI）。

```bash
# 一键启动：三障碍规则避障实验（感知 + 规则避障 + 起飞 + LQR 跟踪 + 实验日志）
./scripts/start_sim.sh lqr --with-perception --with-experiment-logger \
    --obstacle 5,0,2 --obstacle 10,2,2 --obstacle 15,-2,2
```

## 规则避障系统

### 启动顺序

启动仿真环境（含感知+规划+起飞+LQR跟踪+实验日志）：

```bash
# 一键启动：三障碍避障实验
./scripts/start_sim.sh lqr --with-perception --with-experiment-logger \
    --obstacle 5,0,2 --obstacle 10,2,2 --obstacle 15,-2,2
```

**手动分步启动：**

```bash
# 1. 启动仿真环境（PX4 + Gazebo + MAVROS）
./scripts/start_sim.sh sim

# 2. 新终端：启动感知节点和避障规划节点
roslaunch rl_obstacle_avoidance perception.launch &
roslaunch rl_obstacle_avoidance rl_planner_node.launch &

# 3. 新终端：启动起飞控制器并执行起飞
./scripts/start_sim.sh takeoff --altitude 5.0

# 4. 新终端：启动 LQR 轨迹跟踪器和实验日志记录
./scripts/start_sim.sh lqr --with-experiment-logger
```

详细参数配置见 `src/rl_obstacle_avoidance/config/rule_avoidance.yaml`。

### 节点链路

```text
/iris/velodyne_points ──→ perception_node ──→ /obstacle_features
                                                    │
                                                    v
                        rl_planner_node ──→ /rl_planner/action（离散动作）
                             │              /rl_planner/local_goal（局部目标点）
                             │              /rl_planner/local_offset（LQR 位置参考偏移）
                             v
                lqr_tracker_node（LQR 轨迹跟踪）──→ /mavros/setpoint_position/local
```

### 实验验证

以下三个场景已通过 Gazebo 仿真验证：

| 场景 | 障碍物配置 | 结果 |
|------|-----------|------|
| 无障碍 | 无 | 无人机沿 8 字形轨迹正常飞行，无异常偏移 |
| 单障碍 | (5, 0, 2) 0.3×0.3×3m 柱体 | 稳定绕开障碍物，避障后恢复原轨迹 |
| 三障碍 | (5,0,2) (10,2,2) (15,-2,2) | 成功连续绕行三个障碍物 |

实验数据由 `experiment_logger_node` 记录于 `experiment_data/` 目录，使用 `scripts/evaluate_experiment.py` 计算避障成功率、最小障碍距离、跟踪误差、路径长度、避障耗时、偏移抖动六项指标：

```bash
python3 scripts/evaluate_experiment.py experiment_data/experiment_*.csv
```

## DQN 训练

DQN 训练使用独立的运行目录保存配置、训练日志、贪心评估日志以及最佳/末轮模型。
启动方式、指标定义和产物说明见 [DQN 避障训练](docs/dqn_training.md)。

## 构建与运行

### 环境要求

- Ubuntu 20.04 + ROS Noetic
- Catkin Tools
- Gazebo（ROS Noetic 配套版本，PX4 SITL 使用 iris_3d_lidar 机型）
- Python 3 + PyTorch（DQN 训练需要）
- PX4 Firmware 源码（`~/PX4_Firmware`，用于 PX4 SITL 仿真；XTDrone 模型库可选）

### 构建项目

```bash
# 进入工作空间目录
cd ~/anti_drone_agc_workspace

# 使用 catkin_tools 构建
catkin build

# 加载工作空间环境
source devel/setup.bash
```

### 运行

```bash
# 加载 ROS 环境并启动仿真（脚本内部会设置 PX4 SITL 相关环境变量）
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 启动仿真环境
./scripts/start_sim.sh sim

# 一键 LQR 跟踪（含感知与规则避障）
./scripts/start_sim.sh lqr --with-perception
```

## 文档索引

- [DQN 避障训练](docs/dqn_training.md)
- [ROS 日志常见问题与解决方法](docs/faq_log_issues.md)
- [后续研发与学习规划](docs/learning_plan.md)
- [课程指导书](docs/指导书/基于强化学习的无人机自主避障与路径规划系统.md)
