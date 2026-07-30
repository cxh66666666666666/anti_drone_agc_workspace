


# 反无人机空地协同项目

简体中文 | [English](./README_EN.md)

## 项目简介

anti_drone_agc_workspace 是一个基于 ROS 的反无人机空地协同系统实验平台。该项目集成了无人机控制、激光雷达 SLAM、LQR 轨迹跟踪控制以及 Gazebo 仿真等核心技术，旨在研究和验证空地协同探测与跟踪任务的相关算法。

### 核心功能

- **无人机发射与控制**：通过 drone_launch_controller 实现无人机的安全起飞、降落和模式切换
- **激光雷达 SLAM**：集成 A-LOAM 实现高精度地图构建
- **LQR 轨迹跟踪**：基于线性二次调节器的轨迹跟踪控制器
- **规则避障**：基于扇区距离的视觉避障，支持单障碍绕行和多障碍连续绕行
- **障碍物生成与分析**：提供轨迹分析和障碍物生成的辅助脚本
- **Gazebo 仿真支持**：完整的 XTDrone 虚拟仿真环境

### 技术栈

| 类别 | 技术 |
|------|------|
| 操作系统 | Ubuntu + ROS |
| 仿真平台 | Gazebo + XTDrone |
| 传感器 | Velodyne VLP-16/HDL-32 激光雷达 |
| 飞控固件 | PX4 |
| 中间件 | MAVROS |
| 核心算法 | A-LOAM, LQR |
| 路径规划 | 扇区感知 + 规则避障 |

## 目录结构

```
anti_drone_agc_workspace/
├── docs/                      # 文档目录
│   ├── learning_plan.md       # 后续研发与学习规划
│   ├── project_guide.md      # 项目开发指南
│   └── xtdrone_simulation_guide.md  # XTDrone仿真指南
├── scripts/                  # 辅助脚本
│   ├── analyze_trajectory.py    # 轨迹分析与障碍物生成
│   ├── spawn_obstacles.py      # 障碍物生成脚本
│   ├── start_drone_test.sh    # 无人机测试启动脚本
│   └── start_lqr_tracker_sim.sh  # LQR跟踪仿真启动脚本
└── src/                     # 源代码工作空间
    ├── A-LOAM/               # 激光雷达SLAM算法包
    ├── air_ground_core/      # 空地协同核心控制包
    ├── drone_launch_controller/  # 无人机发射控制器
    ├── lqr_trajectory_tracker/  # LQR轨迹跟踪器
    ├── rl_obstacle_avoidance/  # 规则避障规划包
    ├── gazebo_ros_pkgs/      # Gazebo ROS集成包
    └── velodyne_*            # Velodyne激光雷达支持包
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

### 实验验证

以下三个场景已通过 Gazebo 仿真验证：

| 场景 | 障碍物配置 | 结果 |
|------|-----------|------|
| 无障碍 | 无 | 无人机沿 8 字形轨迹正常飞行，无异常偏移 |
| 单障碍 | (5, 0, 2) 0.3×0.3×3m 柱体 | 稳定绕开障碍物，避障后恢复原轨迹 |
| 三障碍 | (5,0,2) (10,2,2) (15,-2,2) | 成功连续绕行三个障碍物 |

实验数据记录于 `experiment_data/` 目录，使用 `scripts/evaluate_experiment.py` 进行分析。

### DQN 训练

DQN 训练使用独立的运行目录保存配置、训练日志、贪心评估日志以及最佳/末轮模型。
启动方式、指标定义和产物说明见 [DQN 避障训练](docs/dqn_training.md)。

## 构建与运行

### 环境要求

- Ubuntu 18.04 或更高版本
- ROS Kinetic/Melodic
- Catkin Tools
- Gazebo 9+

### 构建项目

```bash
# 进入工作空间目录
cd ~/anti_drone_agc_workspace

# 初始化工作空间（如需要）
source /opt
