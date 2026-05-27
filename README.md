


# 反无人机空地协同项目

简体中文 | [English](./README_EN.md)

## 项目简介

anti_drone_agc_workspace 是一个基于 ROS 的反无人机空地协同系统实验平台。该项目集成了无人机控制、激光雷达 SLAM、LQR 轨迹跟踪控制以及 Gazebo 仿真等核心技术，旨在研究和验证空地协同探测与跟踪任务的相关算法。

### 核心功能

- **无人机发射与控制**：通过 drone_launch_controller 实现无人机的安全起飞、降落和模式切换
- **激光雷达 SLAM**：集成 A-LOAM 实现高精度地图构建
- **LQR 轨迹跟踪**：基于线性二次调节器的轨迹跟踪控制器
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
    ├── gazebo_ros_pkgs/      # Gazebo ROS集成包
    └── velodyne_*            # Velodyne激光雷达支持包
```

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