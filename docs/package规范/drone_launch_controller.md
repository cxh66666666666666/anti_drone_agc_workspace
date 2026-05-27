# drone_launch_controller

`drone_launch_controller` 专职负责无人机从"地面静态"到"空中悬停"的状态机管理，屏蔽 MAVROS 底层复杂的通信细节。

---

## 2.1 状态机设计

节点内部维护的标准飞行状态（`FlightState`）：

| 状态值 | 状态名 | 说明 |
|:---:|:---|:---|
| 0 | `INIT` | 节点初始化 |
| 1 | `CONNECTED` | 与 MAVLink / MAVROS 建立连接 |
| 2 | `HEALTH_CHECK` | 传感器与硬件自检中 |
| 3 | `ARMED` | 电机解锁成功 |
| 4 | `TAKEOFF` | 正在垂直上升 |
| 5 | `HOVER` | 到达目标高度，转入定点悬停，可交付 LQR 追踪器 |
| 6 | `LAND` | 无人机已降落 |
| 7 | `EMERGENCY` | 异常终止，触发自动降落 / 切断动力 |

---

## 2.2 ROS Topics

### 发布（Publish）

| 项目 | 内容 |
|:---|:---|
| **Topic** | `/drone_launch_controller/flight_status` |
| **类型** | `drone_launch_controller/FlightStatus`（自定义） |
| **速率** | 10 Hz |
| **含义** | 向外广播当前无人机所处的内部状态机状态及基础遥测数据 |

### 订阅（Subscribe）

| 项目 | 内容 |
|:---|:---|
| **Topic** | `/mavros/state` |
| **类型** | `mavros_msgs/State` |
| **含义** | 监听底层飞控是否连接、是否解锁及当前所处的 PX4 飞行模式（如 `OFFBOARD`） |

---

## 2.3 ROS Services

此前为规避依赖使用了 `std_srvs/Empty`，为提升工业级鲁棒性，重构后收敛为显式带返回值的自定义服务：

| 服务名称 | 类型 | 输入参数 | 输出参数 | 业务逻辑 |
|:---|:---|:---|:---|:---|
| `~request_health_check` | `DroneHealthCheck.srv` | 无 | `bool success`, `string message` | 触发全局电池、GPS、IMU 自检，返回健康报告 |
| `~request_takeoff` | `Takeoff.srv` | `float32 target_altitude` | `bool success` | 触发电机解锁，并在 OFFBOARD 模式下控制无人机上升至指定高度 |
