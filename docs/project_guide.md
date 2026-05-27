# 反无人机空地协同项目指南

## 1. 项目概述

### 1.1 项目定位

anti_drone_agc_workspace 是一个基于 ROS（Robot Operating System）的反无人机空地协同仿真与控制系统。项目名称中的 "anti_drone" 代表反无人机应用，"agc" 是 Air-Ground Coordination（空地协同）的缩写，表明该项目旨在实现无人机与地面移动平台之间的协同工作。

### 1.2 核心功能

本项目整合了多项关键 robotics 技术：

- **SLAM 定位与建图**：通过 A-LOAM 实现基于激光雷达的实时定位与地图构建
- **仿真环境**：利用 Gazebo 物理引擎实现机器人仿真
- **传感器模拟**：Velodyne 系列激光雷达的仿真支持
- **协同控制**：空地平台之间的指令控制与信息交互

### 1.3 技术栈

| 类别 | 技术 |
|------|------|
| 操作系统 | Ubuntu 20.04 |
| 机器人操作系统 | ROS 未确定 |
| 仿真平台 | Gazebo |
| 建图算法 | A-LOAM (Advanced LOAM) |
| 激光雷达 | Velodyne VLP-16, HDL-32E, HDL-64 |
| 构建系统 | Catkin |

---

## 2. 项目目录结构

```
anti_drone_agc_workspace/
├── src/                    # 源代码工作空间
├── build/                  # 编译产生的临时文件
├── devel/                  # 编译产生的可执行文件和库
├── logs/                   # 构建日志
├── devel/                  # 开发环境配置脚本
├── .catkin_tools/          # catkin 工具配置
├── CMakeLists.txt          # 工作空间级 CMake 配置
└── docs/                   # 项目文档
```

### 2.1 目录详细说明

#### src/ — 源代码工作空间

存放所有 ROS 软件包的源代码，是项目的核心区域。

#### build/ — 编译临时目录

Catkin 编译过程中产生的临时文件，包括 CMake 缓存、中间目标文件等。可以在此处查看编译错误信息。

#### devel/ — 开发目录

编译成功后生成的可执行文件、库文件以及环境设置脚本（setup.bash）存放于此。

#### logs/ — 构建日志

存储 catkin 构建过程中的日志文件，用于排查编译问题。

#### .catkin_tools/ — Catkin 配置

管理 catkin 工具的配置文件，包括包配置、构建 Profile 等。

---

## 3. 软件包详解

### 3.1 自定义软件包

#### air_ground_core — 空地协同核心控制包

**路径**：`src/air_ground_core/`

**功能**：项目的核心业务包，实现空地协同控制逻辑。

**主要文件**：

| 文件 | 用途 |
|------|------|
| package.xml | 包清单，定义依赖和元信息 |
| CMakeLists.txt | 构建配置 |
| src/car_commander.cpp | 移动平台控制节点 |

**car_commander.cpp** 实现了一个 ROS 节点，通过发布 `/cmd_vel` 话题控制地面移动平台。节点以 10Hz 频率发送速度指令（线速度 0.5m/s，角速度 0.2rad/s），实现移动平台的圆周运动控制。

**依赖项**：roscpp, geometry_msgs, std_msgs

#### A-LOAM — 激光雷达 SLAM 算法包

**路径**：`src/A-LOAM/`

**功能**：高级 LOAM 实现，用于激光雷达里程计与建图。LOAM（Lidar Odometry and Mapping）由 Ji Zhang 和 Sebastian Singh 提出，本项目采用 Eigen 和 Ceres Solver 进行了重构优化。

**主要源码文件**：

| 文件 | 功能 |
|------|------|
| scanRegistration.cpp | 激光扫描特征提取（角点、平面点） |
| laserOdometry.cpp | 激光里程计，计算实时位姿变化 |
| laserMapping.cpp | 激光建图，全局地图优化 |
| kittiHelper.cpp | KITTI 数据集格式转换工具 |

**Launch 文件**：

| 文件 | 适用场景 |
|------|----------|
| aloam_velodyne_VLP_16.launch | Velodyne VLP-16 激光雷达 |
| aloam_velodyne_HDL_32.launch | HDL-32E 激光雷达 |
| aloam_velodyne_HDL_64.launch | HDL-64 激光雷达 |
| kitti_helper.launch | KITTI 数据集处理 |

**依赖项**：Eigen, Ceres Solver, PCL, ROS 相关包

### 3.2 Gazebo 仿真相关包

#### gazebo_ros_pkgs — ROS 与 Gazebo 集成包

**路径**：`src/gazebo_ros_pkgs/`

Gazebo 官方 ROS 集成包集合，提供 Gazebo 与 ROS 之间的接口。

**子包说明**：

##### gazebo_msgs

**路径**：`src/gazebo_ros_pkgs/gazebo_msgs/`

定义 Gazebo 与 ROS 交互的消息和服务类型，是其他 gazebo_ros 包的基础依赖。

##### gazebo_ros

**路径**：`src/gazebo_ros_pkgs/gazebo_ros/`

核心接口包，提供 ROS 插件使 Gazebo 和 ROS 能够通信。主要插件：

- gazebo_ros_api_plugin：提供 ROS 服务接口
- gazebo_ros_paths_plugin：管理 Gazebo 模型路径

##### gazebo_plugins

**路径**：`src/gazebo_ros_pkgs/gazebo_plugins/`

丰富的 Gazebo 传感器和执行器插件库，支持：

| 插件 | 功能 |
|------|------|
| gazebo_ros_laser | 激光雷达传感器 |
| gazebo_ros_camera | 相机传感器 |
| gazebo_ros_imu | IMU 传感器 |
| gazebo_ros_diff_drive | 差速驱动 |
| gazebo_ros_tricycle_drive | 三轮车驱动 |
| gazebo_ros_skid_steer_drive | 滑移转向驱动 |
| gazebo_ros_joint_state_publisher | 关节状态发布 |

##### gazebo_ros_control

**路径**：`src/gazebo_ros_pkgs/gazebo_ros_control/`

提供机器人硬件抽象接口的实现，支持：

- controller_manager：控制器管理
- joint_limits_interface：关节限位
- transmission_interface：传动接口

### 3.3 Velodyne 激光雷达支持包

#### velodyne_description — 雷达描述包

**路径**：`src/velodyne_description/`

**功能**：Velodyne 激光雷达的 URDF 描述和 3D 模型文件。

**内容**：

| 目录/文件 | 用途 |
|----------|------|
| urdf/*.urdf.xacro | 雷达的 URDF 描述文件 |
| meshes/ | 3D 几何模型（STL/DAE 格式） |
| rviz/ | RViz 配置文件 |
| CHANGELOG.rst | 变更日志 |

**支持的雷达型号**：VLP-16, HDL-32E

#### velodyne_gazebo_plugins — Gazebo 雷达插件

**路径**：`src/velodyne_gazebo_plugins/`

**功能**：在 Gazebo 仿真环境中模拟 Velodyne 激光雷达的测量行为。

**核心文件**：GazeboRosVelodyneLaser.cpp

**依赖项**：gazebo_ros, gazebo_msgs

#### velodyne_simulator — 仿真元包

**路径**：`src/velodyne_simulator/`

**功能**：Velodyne 仿真组件的元包（metapackage），整合 velodyne_description 和 velodyne_gazebo_plugins。

---

## 4. 模块依赖关系

### 4.1 依赖层次图

```
air_ground_core
    └── ROS core (roscpp, geometry_msgs, std_msgs)

A-LOAM
    ├── ROS core (roscpp, rospy, std_msgs, sensor_msgs, nav_msgs, tf, image_transport)
    ├── PCL (Point Cloud Library)
    ├── Eigen (线性代数库)
    └── Ceres Solver (非线性优化)

gazebo_ros
    ├── gazebo_msgs (消息定义)
    └── gazebo_dev (Gazebo 开发库)

gazebo_plugins
    ├── gazebo_ros (ROS 接口)
    ├── gazebo_msgs (消息定义)
    └── 各种传感器/执行器库

gazebo_ros_control
    ├── gazebo_ros (ROS 接口)
    ├── controller_manager (控制器管理)
    └── hardware_interface (硬件接口)

velodyne_gazebo_plugins
    ├── gazebo_ros (ROS 接口)
    └── gazebo_msgs (消息定义)

velodyne_description
    └── urdf, xacro (模型描述)

velodyne_simulator (元包)
    ├── velodyne_description
    └── velodyne_gazebo_plugins
```

### 4.2 运行时数据流

```
[Gazebo 仿真环境]
    │
    ├── 发布: /gazebo/model_states (模型状态)
    ├── 发布: /scan (激光扫描数据)
    │
    ▼
[gazebo_ros] ←→ [gazebo_msgs] (消息转换)
    │
    ▼
[A-LOAM 节点]
    ├── 订阅: /scan (Velodyne 激光数据)
    ├── 发布: /laser_odom (激光里程计)
    └── 发布: /laser_mapping (建图结果)

[air_ground_core 节点]
    ├── 订阅: /laser_odom (获取位置)
    └── 发布: /cmd_vel (控制指令) → [Gazebo 仿真环境]
```

---

## 5. 构建与运行

### 5.1 构建项目

```bash
cd ~/anti_drone_agc_workspace
catkin_make
source devel/setup.bash
```

### 5.2 运行示例

**启动 A-LOAM 建图（VLP-16）**：
```bash
roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
```

**播放激光雷达数据**：
```bash
rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```

**启动地面移动平台控制**：
```bash
rosrun air_ground_core car_commander
```

### 5.3 环境变量

构建完成后，需要 source 环境脚本：
```bash
source devel/setup.bash
```

或添加到 ~/.bashrc 自动加载：
```bash
echo "source ~/anti_drone_agc_workspace/devel/setup.bash" >> ~/.bashrc
```

---

## 6. 关键配置文件

| 文件路径 | 用途 |
|----------|------|
| CMakeLists.txt | 工作空间级构建配置 |
| src/*/CMakeLists.txt | 各软件包构建配置 |
| src/*/package.xml | 各软件包元信息与依赖声明 |
| .catkin_tools/profiles/default/build.yaml | Catkin 构建配置 |

---

## 7. 新开发者快速上手指南

### 7.1 推荐阅读顺序

1. 阅读本文档，了解整体架构
2. 查看 air_ground_core/src/car_commander.cpp，理解 ROS 节点基本结构
3. 研究 A-LOAM 的 scanRegistration.cpp 和 laserOdometry.cpp，学习 SLAM 算法
4. 浏览 gazebo_plugins/src/ 中的传感器插件，了解 Gazebo 插件开发

### 7.2 常见问题排查

| 问题 | 解决方案 |
|------|----------|
| 编译失败 | 检查日志目录 logs/，查看具体错误信息 |
| ROS 找不到包 | 确保执行了 `source devel/setup.bash` |
| Gazebo 模型加载失败 | 检查 GAZEBO_MODEL_PATH 环境变量 |
| 激光数据无输出 | 确认 velodyne_gazebo_plugins 已正确加载 |

### 7.3 开发建议

- 修改源码后需要重新编译：`catkin_make`
- 调试 ROS 节点时使用 `rosrun rqt_graph rqt_graph` 查看节点关系
- 使用 `rostopic list` 和 `rostopic echo` 查看话题数据

---

## 8. 扩展指南

### 8.1 添加新的软件包

```bash
cd src
catkin_create_pkg my_package roscpp std_msgs
```

### 8.2 集成新的传感器

1. 在 src/ 下创建或添加传感器描述包
2. 如需 Gazebo 仿真，参考 velodyne_gazebo_plugins 实现 Gazebo 插件
3. 在相应的 launch 文件中添加节点配置

### 8.3 添加新的控制算法

参考 air_ground_core 的结构创建新的控制节点，通过 ROS topic 与其他模块通信。

---

本项目是一个功能完整的机器人仿真与控制系统，涵盖了从底层传感器仿真到高层 SLAM 算法的完整技术栈。希望本指南能帮助您快速上手本项目的开发和调试工作。
