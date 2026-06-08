# drone_launch_controller

无人机启动控制器，管理"地面静态 → 空中悬停"的完整生命周期，
包括 MAVROS 连接、系统自检、电机解锁和起飞控制。

## 功能概述

- 无人机与飞控的连接状态监控
- 系统健康状态自检（电池、GPS、IMU、ESC）
- 电机解锁控制（支持强制解锁）
- 起飞高度控制与飞行状态发布

## 系统要求

- **操作系统**: Ubuntu 18.04 / 20.04 / 22.04
- **ROS 版本**: ROS Melodic / Noetic
- **依赖包**: roscpp, std_msgs, geometry_msgs, sensor_msgs, mavros_msgs

## 快速开始

```bash
# 1. 启动 PX4 SITL + MAVROS
roslaunch px4 mavros_posix_sitl.launch

# 2. 启动控制器
roslaunch drone_launch_controller drone_controller.launch

# 3. 检查连接状态
rosservice call /drone_launch_controller/connection_status

# 4. 系统自检
rosservice call /drone_launch_controller/drone_health_check

# 5. 解锁电机
rosservice call /drone_launch_controller/arm_motors "force: false"

# 6. 起飞
rosservice call /drone_launch_controller/takeoff "altitude: 5.0" "timeout: 30.0"
```

## 核心服务

| 服务名称 | srv 类型 | 说明 |
|----------|----------|------|
| `/drone_launch_controller/connection_status` | ConnectionStatus | 查询 MAVROS 连接状态（返回 connected, system_status, message） |
| `/drone_launch_controller/drone_health_check` | DroneHealthCheck | 触发全局自检，返回 success, result, health_status, message |
| `/drone_launch_controller/arm_motors` | ArmMotors | 电机解锁（请求: force, 响应: success, result, message） |
| `/drone_launch_controller/takeoff` | Takeoff | 起飞控制（请求: altitude, timeout, 响应: success, result, achieved_altitude, message） |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/drone_launch_controller/flight_status` | FlightStatus | 飞行状态（10 Hz） |

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
| `connection_timeout` | 10.0 | MAVROS 连接超时 (秒) |
| `target_altitude` | 5.0 | 默认起飞高度 (米) |
| `takeoff_timeout` | 30.0 | 起飞超时 (秒) |
| `arm_timeout` | 5.0 | 电机解锁超时 (秒) |
| `max_altitude_limit` | 100.0 | 最大高度限制 (米) |
| `max_speed_limit` | 10.0 | 最大速度限制 (m/s) |
| `geofence_radius` | 500.0 | 地理围栏半径 (米) |
| `status_publish_rate` | 10.0 | 状态发布频率 (Hz) |
| `mavros_ns` | /mavros | MAVROS 命名空间 |
| `verbose` | false | 启用详细的飞行状态日志 |

## 健康检查状态码

| 位 | 含义 | 条件 |
|----|------|------|
| 0x01 | 电池异常 | 电压 < 3.3V 或百分比 < 20% |
| 0x02 | GPS 异常 | 未固定或服务不可用 |
| 0x04 | IMU 异常 | 数据无效或过期 (>1s) |
| 0x08 | ESC 异常 | 数据为空或电压异常 |
| 0x00 | 全部通过 | 以上检查均正常 |

注: 气压计和磁力计字段保留但始终为 false（当前 MAVROS 话题不支持直接检测）。

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
