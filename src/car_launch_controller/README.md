# car_launch_controller

地面小车启动控制器，管理"仿真就绪 → 运动使能 → 行驶 → 释放控制权"的完整生命周期，
包括 Gazebo 连接、车况自检、运动使能和行驶控制。

## 功能概述

- 小车与 Gazebo 仿真的连接状态监控
- 车况自检（车模存在性、位姿新鲜度、感知数据、控制服务可用性）
- 运动使能（接管 `ground_car` 车模）
- 沿 world +x 行驶指定距离，并在到位后保持位姿
- 释放控制权，交还 RL 流程

## 系统要求

- **操作系统**: Ubuntu 20.04
- **ROS 版本**: ROS Noetic
- **依赖包**: roscpp, std_msgs, geometry_msgs, std_srvs, gazebo_msgs, rl_ground_avoidance

## 快速开始

```bash
# 1. 一键启动仿真栈（gzserver + 感知节点）+ 控制器 [终端 A]
roslaunch car_launch_controller car_sim.launch

# 2. 检查连接状态 [终端 B]
rosservice call /car_launch_controller/connection_status

# 3. 车况自检 [终端 B]
rosservice call /car_launch_controller/car_health_check

# 4. 使能运动控制（接管车模）[终端 B]
rosservice call /car_launch_controller/enable_motion

# 5. 行驶：前进 3 米，速度 0.5 m/s，超时 30 s [终端 B]
rosservice call /car_launch_controller/drive "distance: 3.0" "speed: 0.5" "timeout: 30.0"

# 6. 释放控制权，交还 RL 流程 [终端 B]
rosservice call /car_launch_controller/release_control
```

> 步骤 2–6 为同一条生命周期的顺序调用，需在仿真已启动（步骤 1）后执行。

## 核心服务

| 服务名称 | srv 类型 | 说明 |
|----------|----------|------|
| `/car_launch_controller/connection_status` | ConnectionStatus | 查询 Gazebo 连接状态（返回 connected, system_status, message） |
| `/car_launch_controller/car_health_check` | CarHealthCheck | 触发车况自检，返回 success, result, health_status, message |
| `/car_launch_controller/enable_motion` | EnableMotion | 使能运动控制并接管车模（请求为空，响应 success, result, message） |
| `/car_launch_controller/drive` | Drive | 行驶控制（请求 distance, speed, timeout，响应 success, result, achieved_distance, message） |
| `/car_launch_controller/release_control` | std_srvs/Trigger | 释放控制权，把 `/ground_gazebo/set_model_state` 交还 RL 流程（响应 success, message） |

> `drive` 中 `distance <= 0` 直接失败；`speed <= 0` 时取节点私有参数 `~reference_speed`（默认 1.0）；
> `timeout <= 0` 时取 30.0 s。超时未到达则返回失败，`achieved_distance` 为已行进距离。

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/car_launch_controller/car_status` | CarStatus | 小车状态（10 Hz）：control_mode（控制权归属）、position_x/position_y（位置）、yaw（偏航角）、linear_speed（线速度）、motion_enabled（使能标志）、state（状态枚举） |

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ground_gazebo/link_states` | gazebo_msgs/LinkStates | 车模位姿（条目名 `ground_car::base_link`） |
| `/ground_obstacle_sectors` | std_msgs/Float32MultiArray | 感知节点发布的 12 扇区归一化距离 |

> 以上话题位于 Gazebo 节点命名空间 `/ground_gazebo` 下，其中 `link_states` 通过 launch 参数
> `gazebo_ns` 重映射；`/ground_obstacle_sectors` 由感知节点固定发布，不在重映射范围内。

## Launch 参数

### car_controller.launch

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `quiet_car_status` | false | 为 true 时不将 car_status 回显到日志 |
| `gazebo_ns` | /ground_gazebo | Gazebo 节点命名空间 |

### car_sim.launch

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gui` | true | 是否启动 Gazebo 客户端界面（gzclient） |
| `quiet_car_status` | false | 为 true 时不将 car_status 回显到日志 |

> `car_sim.launch` 会 include `rl_ground_avoidance/ground_sim.launch`（gzserver + 感知节点），
> `GAZEBO_MODEL_PATH` 与 `/use_sim_time` 由它统一负责，本包不重复设置。

> 节点私有参数 `~model_name`（默认 `ground_car`）、`~control_rate`（默认 20.0）、
> `~reference_speed`（默认 1.0）未在 launch 中暴露，可通过额外 `<param>` 覆盖。

## 状态枚举

| 值 | 状态 | 说明 |
|----|------|------|
| 0 | INIT | 节点初始化 |
| 1 | CONNECTED | 已连接，未接管控制权 |
| 2 | HEALTH_CHECK | 车况自检中 |
| 3 | ENABLED | 运动控制已使能 |
| 4 | DRIVING | 正在行驶 |
| 5 | HOLDING | 到位后保持位姿 |
| 6 | STOPPED | 已停止 |
| 7 | EMERGENCY | 紧急状态 |

## 健康检查状态码

| 位 | 含义 | 条件 |
|----|------|------|
| 0x01 | 车模缺失 | `/ground_gazebo/link_states` 中没有 `ground_car::base_link` |
| 0x02 | 位姿数据过期 | 距上次位姿更新 > 1s |
| 0x04 | 感知数据异常 | `/ground_obstacle_sectors` 未发布、数据为空或过期 > 1s |
| 0x08 | 控制服务不可用 | `/ground_gazebo/set_model_state` 不存在 |
| 0x00 | 全部通过 | 以上检查均正常 |

## 控制权互斥

本节点使能后独占 `/ground_gazebo/set_model_state` 的写入权；`rl_ground_avoidance` 的
RL 训练/评估流程（`ground_env.py`）也会写同一个服务。**跑 RL 训练或评估前必须先调用
`/car_launch_controller/release_control`**，否则两边会互相抢车模位置。

`car_status` 的 `control_mode` 字段取值：

- `NONE`：本节点未持有控制权
- `CONTROLLER`：本节点持有控制权

## 使用示例 (Python)

```python
#!/usr/bin/env python3
import rospy
from car_launch_controller.srv import EnableMotion, Drive

rospy.init_node('car_example')
rospy.wait_for_service('/car_launch_controller/enable_motion')
rospy.wait_for_service('/car_launch_controller/drive')

enable_srv = rospy.ServiceProxy('/car_launch_controller/enable_motion', EnableMotion)
drive_srv = rospy.ServiceProxy('/car_launch_controller/drive', Drive)

enable_srv()
drive_srv(distance=3.0, speed=0.5, timeout=30.0)
```

## 许可证

BSD License