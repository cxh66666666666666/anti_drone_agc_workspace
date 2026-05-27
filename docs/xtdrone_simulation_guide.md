# XTDrone 虚拟仿真实验操作指南

## 1. 环境检测与配置

| 组件 | 路径/版本 |
|------|-----------|
| XTDrone 安装路径 | /home/ubuntu/XTDrone |
| PX4_Firmware 路径 | /home/ubuntu/PX4_Firmware |
| ROS 版本 | Noetic |
| MAVROS 路径 | /opt/ros/noetic/share/mavros |
| Gazebo 版本 | 11.15.1 |

## 2. 环境变量配置

```bash
# 在 ~/.bashrc 中添加以下环境变量
export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH
export ROS_PACKAGE_PATH=/home/ubuntu/XTDrone:$ROS_PACKAGE_PATH
```

配置完成后，执行以下命令使环境变量生效：

```bash
source ~/.bashrc
```

## 3. 仿真启动命令

### 3.1 四轴无人机（Iris）单机仿真

```bash
cd /home/ubuntu/XTDrone
source ~/catkin_ws/devel/setup.bash
roslaunch mavros_posix_sitl.launch vehicle:=iris
```

### 3.2 多旋翼配置

```bash
# 六轴无人机 (Typhoon_H480)
cd /home/ubuntu/XTDrone
source ~/catkin_ws/devel/setup.bash
roslaunch mavros_posix_sitl.launch vehicle:=Typhoon_H480
```

### 3.3 多无人机仿真

```bash
# 启动多无人机模式（以双机为例）
cd /home/ubuntu/XTDrone
source ~/catkin_ws/devel/setup.bash
roslaunch mavros_posix_sitl_multi.launch vehicle:=iris viz:=1
```

## 4. 键盘控制测试

### 4.1 启动步骤

**终端 1**：启动仿真环境

```bash
cd /home/ubuntu/XTDrone
source ~/catkin_ws/devel/setup.bash
roslaunch mavros_posix_sitl.launch vehicle:=iris
```

**终端 2**：启动键盘控制程序

```bash
cd /home/ubuntu/XTDrone/control/keyboard
python attitude_contorl_demo.py
```

### 4.2 键盘控制说明

| 按键 | 功能 |
|------|------|
| `W/S` | 上升/下降 |
| `A/D` | 偏航左转/右转 |
| `↑/↓` | 前后移动 |
| `←/→` | 左右移动 |
| `Q` | 紧急停机 |
| `E` | 退出程序 |

## 5. MAVROS 连接验证

### 5.1 检查连接状态

```bash
rostopic echo /mavros/state
```

确认输出中 `connected: True`，表示 MAVROS 与 PX4 SITL 连接正常。

### 5.2 检查电池状态

```bash
rostopic echo /mavros/battery
```

### 5.3 检查本地位置

```bash
rostopic echo /mavros/local_position/pose
```

### 5.4 常用诊断命令

```bash
# 列出所有 MAVROS 相关话题
rostopic list | grep mavros

# 检查系统状态
rosrun mavros mavsys.py

# 查看飞行模式
rostopic echo /mavros/state | grep mode
```

## 6. drone_launch_controller 集成

### 6.1 启动流程

1. **终端 1**：启动 XTDrone 仿真

```bash
cd /home/ubuntu/XTDrone
source ~/catkin_ws/devel/setup.bash
roslaunch mavros_posix_sitl.launch vehicle:=iris
```

2. **终端 2**：启动 drone_launch_controller

```bash
roslaunch drone_launch_controller drone_controller.launch
```

### 6.2 启动参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `vehicle_ns` | 无人机命名空间 | iris |
| `fcu_url` | MAVROS 连接地址 | udp://:24540@localhost:34580 |

## 7. 常见问题排查

### 7.1 PX4 SITL 启动失败

**问题描述**：仿真环境无法启动，提示找不到 px4_sitl

**解决方案**：

1. 检查 PX4_Firmware 编译产物是否存在

```bash
ls -la /home/ubuntu/PX4_Firmware/build/px4_sitl_default
```

2. 若不存在，重新编译 PX4_Firmware

```bash
cd /home/ubuntu/PX4_Firmware
make posix_sitl_erlecopter_px4
```

3. 检查环境变量是否正确配置

```bash
echo $GAZEBO_MODEL_PATH
```

### 7.2 MAVROS 连接失败

**问题描述**：rostopic echo /mavros/state 显示 connected: False

**解决方案**：

1. 检查 fcu_url 参数配置

```bash
roslaunch mavros_posix_sitl.launch vehicle:=iris fcu_url:=udp://:24540@localhost:34580
```

2. 确认防火墙设置，允许本地 UDP 端口通信

```bash
sudo ufw allow 24540/udp
sudo ufw allow 34580/udp
```

3. 检查 MAVROS 是否正确安装

```bash
rospack find mavros
```

### 7.3 Gazebo 模型加载失败

**问题描述**：Gazebo 启动后缺少无人机模型

**解决方案**：

1. 确认 GAZEBO_MODEL_PATH 环境变量

```bash
export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH
```

2. 检查模型文件是否存在

```bash
ls /home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models/iris/
```

3. 重新设置 Gazebo 模型路径后启动

```bash
export GAZEBO_MODEL_PATH=/home/ubuntu/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH
roslaunch mavros_posix_sitl.launch vehicle:=iris
```

### 7.4 键盘控制无响应

**问题描述**：attitude_contorl_demo.py 运行正常但无人机无响应

**解决方案**：

1. 确认 MAVROS 连接状态正常

```bash
rostopic echo /mavros/state
```

2. 检查 MAVLink 话题是否正常

```bash
rostopic list | grep mavlink
```

3. 确认遥控模式已切换至 offboard 模式

```bash
rostopic pub -1 /mavros/set_mode std_msgs/String "offboard"
```

## 8. 快速启动检查清单

- [ ] XTDrone 安装路径正确
- [ ] PX4_Firmware 已编译
- [ ] 环境变量已配置并生效
- [ ] Gazebo 模型路径已设置
- [ ] MAVROS 连接正常
- [ ] 键盘控制程序可运行

## 9. 性能优化建议

1. **关闭不必要的可视化**：使用 `gui:=false` 参数减少 GPU 占用

```bash
roslaunch mavros_posix_sitl.launch vehicle:=iris gui:=false
```

2. **减少仿真线程**：使用 `headless:=true` 启用无头模式

```bash
roslaunch mavros_posix_sitl.launch vehicle:=iris headless:=true
```

3. **调整实时因子**：若仿真运行过快，可在 PX4 参数中调整 `RTL_SPEED` 相关参数