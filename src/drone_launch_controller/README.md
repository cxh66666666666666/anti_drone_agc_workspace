# drone_launch_controller

无人机启动控制器软件包，实现无人机连接初始化、系统自检、电机解锁和起飞控制功能。

## 功能概述

- 无人机与飞控的连接状态监控
- 系统健康状态自检（电池、GPS、IMU、气压计、磁力计、ESC）
- 电机解锁控制（支持强制解锁）
- 起飞高度控制与飞行状态发布

## 系统要求

- **操作系统**: Ubuntu 18.04 / 20.04 / 22.04
- **ROS 版本**: ROS Melodic / Noetic
- **依赖包**: roscpp, std_msgs, geometry_msgs, sensor_msgs, mavros_msgs, actionlib, Boost

## 快速开始

```bash
# 1. 启动 MAVROS (PX4)
roslaunch mavros px4.launch

# 2. 启动控制器
roslaunch drone_launch_controller drone_controller.launch

# 3. 检查连接
rosservice call /drone_launch_controller/connection_status

# 4. 系统自检
rosservice call /drone_launch_controller/drone_health_check

# 5. 解锁电机
rosservice call /drone_launch_controller/arm_motors "force: false"

# 6. 起飞
rosservice call /drone_launch_controller/takeoff "altitude: 5.0" "timeout: 30.0"
```

## 核心服务

| 服务 | 描述 |
|------|------|
| `/drone_launch_controller/connection_status` | 查询连接状态 |
| `/drone_launch_controller/drone_health_check` | 系统自检 |
| `/drone_launch_controller/arm_motors` | 电机解锁 |
| `/drone_launch_controller/takeoff` | 起飞控制 |

## 话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/drone_launch_controller/flight_status` | FlightStatus | 飞行状态 |
| `/mavros/state` | mavros_msgs/State | 飞控状态 |

## 使用示例 (Python)

```python
#!/usr/bin/env python3
import rospy
from drone_launch_controller.srv import ArmMotors, Takeoff

rospy.init_node('drone_example')
rospy.wait_for_service('/drone_launch_controller/arm_motors')
rospy.wait_for_service('/drone_launch_controller/takeoff')

arm_srv = rospy.ServiceProxy('/drone_launch_controller/arm_motors', ArmMotors)
takeoff_srv = rospy.ServiceProxy('/drone_launch_controller/takeoff', Takeoff)

arm_srv(force=False)
takeoff_srv(altitude=5.0, timeout=30.0)
```

## 许可证

BSD License
