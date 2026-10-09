# drone_launch_controller

无人机启动控制器，管理"地面静态 → 空中悬停"的完整生命周期，
包括 MAVROS 连接、系统自检、电机解锁和起飞控制。

## 功能概述

- 无人机与飞控的连接状态监控
- 系统健康状态自检（电池、GPS、IMU、ESC）
- 电机解锁控制
- 起飞高度控制与飞行状态发布

## 系统要求

- **操作系统**: Ubuntu 18.04 / 20.04 / 22.04
- **ROS 版本**: ROS Melodic / Noetic
- **依赖包**: roscpp, std_msgs, geometry_msgs, sensor_msgs, mavros_msgs

## 快速开始

```bash
# 1. 启动 PX4 SITL + MAVROS（iris_3d_lidar 机型，含 Velodyne 激光雷达）
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris vehicle_sdf:=iris_3d_lidar

# 2. 启动控制器
roslaunch drone_launch_controller drone_controller.launch

# 3. 检查连接状态
rosservice call /drone_launch_controller/connection_status

# 4. 系统自检
rosservice call /drone_launch_controller/drone_health_check

# 5. 解锁电机
rosservice call /drone_launch_controller/arm_motors

# 6. 起飞
rosservice call /drone_launch_controller/takeoff "altitude: 5.0" "timeout: 30.0"
```

也可以使用工作空间统一启动脚本一键完成仿真启动 + 起飞：

```bash
./scripts/start_sim.sh takeoff --altitude 5.0
```

## 核心服务

| 服务名称 | srv 类型 | 说明 |
|----------|----------|------|
| `/drone_launch_controller/connection_status` | ConnectionStatus | 查询 MAVROS 连接状态（返回 connected, system_status, message） |
| `/drone_launch_controller/drone_health_check` | DroneHealthCheck | 触发全局自检，返回 success, result, health_status, message |
| `/drone_launch_controller/arm_motors` | ArmMotors | 电机解锁（请求: 空, 响应: success, result, message） |
| `/drone_launch_controller/takeoff` | Takeoff | 起飞控制（请求: altitude, timeout, 响应: success, result, achieved_altitude, message） |
| `/drone_launch_controller/release_control` | std_srvs/Trigger | 释放起飞控制权（停止悬停保持发布，响应: success, message） |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/drone_launch_controller/flight_status` | FlightStatus | 飞行状态（flight_mode, current_altitude, armed, state，10 Hz） |
| `/mavros/setpoint_position/local` | PoseStamped | 起飞过程中的位置设定点（交由起飞控制器发布） |

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/mavros/state` | mavros_msgs/State | 飞控连接状态与模式 |
| `/mavros/local_position/pose` | PoseStamped | 无人机当前位姿 |
| `/mavros/battery` | BatteryState | 电池状态 |
| `/mavros/global_position/global` | NavSatFix | GPS 状态 |
| `/mavros/imu/data` | Imu | IMU 数据 |
| `/mavros/esc_status` | ESCStatus | ESC 状态 |

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mavros_ns` | /mavros | MAVROS 命名空间 |
| `quiet_flight_status` | false | 为 true 时不将 flight_status 回显到日志 |

## 健康检查状态码

| 位 | 含义 | 条件 |
|----|------|------|
| 0x01 | 电池异常 | 电压无数据（≤0V）或百分比 < 20% |
| 0x02 | GPS 异常 | 未固定或服务不可用 |
| 0x04 | IMU 异常 | 数据无效或过期 (>1s) |
| 0x08 | ESC 异常 | 数据为空或电压 ≤ 0V |
| 0x00 | 全部通过 | 以上检查均正常 |

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

arm_srv()
takeoff_srv(altitude=5.0, timeout=30.0)
```

## 许可证

BSD License
