# drone_launch_controller

无人机启动控制器软件包，实现无人机连接初始化、系统自检、电机解锁和起飞控制功能。

## 软件包概述

drone_launch_controller 是基于 ROS/Mavros 的无人机控制软件包，提供了完整的无人机起飞前检查和起飞流程管理。该软件包负责：

- 无人机与飞控的连接状态监控
- 系统健康状态自检（电池、GPS、IMU、气压计、磁力计、ESC）
- 电机解锁控制
- 起飞高度控制与飞行状态发布

## 功能列表

| 功能模块 | 描述 |
|---------|------|
| 连接状态监控 | 监控 MAVROS 与飞控的连接状态 |
| 系统自检 | 检查电池、GPS、IMU、气压计、磁力计、ESC 状态 |
| 电机解锁 | 控制电机解锁，支持强制解锁选项 |
| 起飞控制 | 控制无人机起飞到指定高度 |
| 状态发布 | 实时发布飞行状态信息 |

## 系统要求

- **操作系统**: Ubuntu 18.04 / 20.04 / 22.04
- **ROS 版本**: ROS Melodic / Noetic
- **依赖包**:
  - roscpp
  - std_msgs
  - geometry_msgs
  - sensor_msgs
  - mavros_msgs
  - actionlib
  - Boost (thread, mutex)

## 安装说明

1. 将软件包复制到 ROS 工作空间的 `src` 目录：

```bash
cd ~/catkin_ws/src
git clone https://github.com/your_repo/drone_launch_controller.git
```

2. 安装依赖：

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译工作空间：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

4. 验证软件包是否正确安装：

```bash
rospack find drone_launch_controller
```

## 快速开始

### 1. 启动 MAVROS

确保飞控连接正常，首先启动 MAVROS：

```bash
roslaunch mavros apm.launch
```

或对于 PX4 飞控：

```bash
roslaunch mavros px4.launch
```

### 2. 启动无人机控制器

```bash
roslaunch drone_launch_controller drone_controller.launch
```

### 3. 检查连接状态

```bash
rosservice call /drone_launch_controller/connection_status
```

### 4. 执行系统自检

```bash
rosservice call /drone_launch_controller/drone_health_check
```

### 5. 解锁电机

```bash
rosservice call /drone_launch_controller/arm_motors "force: false"
```

### 6. 执行起飞

```bash
rosservice call /drone_launch_controller/takeoff "altitude: 5.0" "timeout: 30.0"
```

## API参考

### 服务 (Services)

#### /drone_launch_controller/connection_status

查询无人机与飞控的连接状态。

**请求**: 无

**响应**:

| 字段 | 类型 | 描述 |
|------|------|------|
| connected | bool | 是否已连接 |
| system_id | uint8 | MAVROS 系统 ID |
| message | string | 状态消息 |

**示例**:
```bash
rosservice call /drone_launch_controller/connection_status
```

---

#### /drone_launch_controller/drone_health_check

执行完整的系统自检。

**请求**: 无

**响应**:

| 字段 | 类型 | 描述 |
|------|------|------|
| success | bool | 自检是否成功 |
| result | uint8 | 结果码（0=成功，1=警告，2=失败） |
| health_status | DroneHealthStatus | 健康状态详情 |
| message | string | 状态消息 |

**示例**:
```bash
rosservice call /drone_launch_controller/drone_health_check
```

---

#### /drone_launch_controller/arm_motors

解锁无人机电机。

**请求**:

| 字段 | 类型 | 描述 |
|------|------|------|
| force | bool | 是否强制解锁（跳过安全检查） |

**响应**:

| 字段 | 类型 | 描述 |
|------|------|------|
| success | bool | 解锁是否成功 |
| result | uint8 | 结果码 |
| message | string | 状态消息 |

**示例**:
```bash
rosservice call /drone_launch_controller/arm_motors "force: false"
```

---

#### /drone_launch_controller/takeoff

执行无人机起飞。

**请求**:

| 字段 | 类型 | 描述 |
|------|------|------|
| altitude | float32 | 目标起飞高度（米） |
| timeout | float32 | 超时时间（秒） |

**响应**:

| 字段 | 类型 | 描述 |
|------|------|------|
| success | bool | 起飞是否成功 |
| result | uint8 | 结果码 |
| achieved_altitude | float32 | 实际达到的高度 |
| message | string | 状态消息 |

**示例**:
```bash
rosservice call /drone_launch_controller/takeoff "altitude: 10.0" "timeout: 30.0"
```

---

### 话题 (Topics)

#### /drone_launch_controller/flight_status

飞行状态话题（发布）。

**类型**: drone_launch_controller/FlightStatus

| 字段 | 类型 | 描述 |
|------|------|------|
| flight_mode | string | 当前飞行模式 |
| current_altitude | float32 | 当前高度（米） |
| target_altitude | float32 | 目标高度（米） |
| latitude | float32 | 纬度 |
| longitude | float32 | 经度 |
| armed | uint8 | 电机锁定状态（0=锁定，1=解锁） |
| state | uint8 | 飞行状态码 |
| message | string | 状态消息 |

---

#### /mavros/state

MAVROS 飞控状态话题。

**类型**: mavros_msgs/State

| 字段 | 类型 | 描述 |
|------|------|------|
| connected | bool | 连接状态 |
| armed | bool | 电机状态 |
| guided | bool | 引导模式 |
| mode | string | 当前飞行模式 |

---

#### /mavros/battery

电池状态话题。

**类型**: sensor_msgs/BatteryState

| 字段 | 类型 | 描述 |
|------|------|------|
| voltage | float32 | 电池电压 |
| percentage | float32 | 电池百分比 |
| current | float32 | 当前电流 |
| charge | float32 | 剩余容量 |

---

#### /mavros/local_position/pose

当前位置话题。

**类型**: geometry_msgs/PoseStamped

| 字段 | 类型 | 描述 |
|------|------|------|
| header | Header | 消息头 |
| pose.position | Vector3 | 位置（x, y, z） |
| pose.orientation | Quaternion | 姿态（四元数） |

---

### 消息类型

#### drone_launch_controller/DroneHealthStatus

```msg
string drone_name       # 无人机名称
float32 battery_voltage # 电池电压
float32 battery_percentage # 电池百分比
bool gps_status         # GPS状态
bool imu_status         # IMU状态
bool barometer_status   # 气压计状态
bool magnetometer_status # 磁力计状态
bool esc_status         # ESC状态
string status_message   # 状态消息
uint16 error_code       # 错误码
```

---

#### drone_launch_controller/FlightStatus

```msg
string flight_mode      # 飞行模式
float32 current_altitude # 当前高度
float32 target_altitude # 目标高度
float32 latitude        # 纬度
float32 longitude       # 经度
uint8 armed             # 电机状态
uint8 state             # 飞行状态
string message          # 状态消息
```

---

## 错误码说明

### DroneHealthCheck 结果码

| 码值 | 含义 |
|------|------|
| 0 | 系统健康，可以飞行 |
| 1 | 系统存在警告，但可以飞行 |
| 2 | 系统检查失败，不能飞行 |

### ArmMotors 结果码

| 码值 | 含义 |
|------|------|
| 0 | 解锁成功 |
| 1 | 解锁失败 |
| 2 | 安全检查未通过 |
| 3 | 超时 |

### Takeoff 结果码

| 码值 | 含义 |
|------|------|
| 0 | 起飞成功 |
| 1 | 未进入 OFFBOARD 模式 |
| 2 | 起飞未完成 |
| 3 | 起飞启动失败 |

### FlightState 状态码

| 码值 | 状态 |
|------|------|
| 0 | INIT - 初始化 |
| 1 | CONNECTED - 已连接 |
| 2 | HEALTH_CHECK - 健康检查 |
| 3 | ARMED - 已解锁 |
| 4 | TAKEOFF - 起飞中 |
| 5 | HOVER - 悬停 |
| 6 | LAND - 降落 |
| 7 | EMERGENCY - 紧急状态 |

### DroneHealthStatus 错误码

| 码值 | 含义 |
|------|------|
| 0x0001 | 电池电压过低 |
| 0x0002 | GPS 信号弱 |
| 0x0004 | IMU 异常 |
| 0x0008 | 气压计异常 |
| 0x0010 | 磁力计异常 |
| 0x0020 | ESC 异常 |
| 0x0040 | 连接超时 |
| 0x0080 | 系统错误 |

---

## 使用示例

### C++ 示例

```cpp
#include <ros/ros.h>
#include <drone_launch_controller/ConnectionStatus.h>
#include <drone_launch_controller/DroneHealthCheck.h>
#include <drone_launch_controller/ArmMotors.h>
#include <drone_launch_controller/Takeoff.h>
#include <drone_launch_controller/FlightStatus.h>

class DroneController {
public:
    DroneController() : nh_("~") {
        connection_client_ = nh_.serviceClient<drone_launch_controller::ConnectionStatus>(
            "/drone_launch_controller/connection_status");
        health_client_ = nh_.serviceClient<drone_launch_controller::DroneHealthCheck>(
            "/drone_launch_controller/drone_health_check");
        arm_client_ = nh_.serviceClient<drone_launch_controller::ArmMotors>(
            "/drone_launch_controller/arm_motors");
        takeoff_client_ = nh_.serviceClient<drone_launch_controller::Takeoff>(
            "/drone_launch_controller/takeoff");
        status_sub_ = nh_.subscribe("/drone_launch_controller/flight_status", 10,
            &DroneController::statusCallback, this);
    }

    bool checkConnection() {
        drone_launch_controller::ConnectionStatus srv;
        if (connection_client_.call(srv)) {
            ROS_INFO("Connection Status: %s, System ID: %d",
                     srv.response.connected ? "Connected" : "Disconnected",
                     srv.response.system_id);
            return srv.response.connected;
        }
        return false;
    }

    bool performHealthCheck() {
        drone_launch_controller::DroneHealthCheck srv;
        if (health_client_.call(srv)) {
            ROS_INFO("Health Check: %s", srv.response.success ? "PASS" : "FAIL");
            ROS_INFO("Battery: %.2fV (%.0f%%)",
                     srv.response.health_status.battery_voltage,
                     srv.response.health_status.battery_percentage * 100);
            return srv.response.success;
        }
        return false;
    }

    bool armMotors(bool force = false) {
        drone_launch_controller::ArmMotors srv;
        srv.request.force = force;
        if (arm_client_.call(srv)) {
            ROS_INFO("Arm Motors: %s", srv.response.success ? "SUCCESS" : "FAIL");
            return srv.response.success;
        }
        return false;
    }

    bool takeoff(float altitude, float timeout = 30.0) {
        drone_launch_controller::Takeoff srv;
        srv.request.altitude = altitude;
        srv.request.timeout = timeout;
        if (takeoff_client_.call(srv)) {
            ROS_INFO("Takeoff: %s (Altitude: %.2f / %.2f)",
                     srv.response.success ? "SUCCESS" : "FAIL",
                     srv.response.achieved_altitude,
                     altitude);
            return srv.response.success;
        }
        return false;
    }

private:
    void statusCallback(const drone_launch_controller::FlightStatus::ConstPtr& msg) {
        ROS_DEBUG("Flight Status: Mode=%s, Alt=%.2f, State=%d",
                  msg->flight_mode.c_str(),
                  msg->current_altitude,
                  msg->state);
    }

    ros::NodeHandle nh_;
    ros::ServiceClient connection_client_;
    ros::ServiceClient health_client_;
    ros::ServiceClient arm_client_;
    ros::ServiceClient takeoff_client_;
    ros::Subscriber status_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_control_example");

    DroneController drone;

    ROS_INFO("=== Drone Control Example ===");

    if (!drone.checkConnection()) {
        ROS_ERROR("Drone not connected!");
        return -1;
    }

    if (!drone.performHealthCheck()) {
        ROS_WARN("Health check failed, proceeding anyway...");
    }

    if (!drone.armMotors(false)) {
        ROS_ERROR("Failed to arm motors!");
        return -1;
    }

    ros::Duration(1.0).sleep();

    if (!drone.takeoff(5.0, 30.0)) {
        ROS_ERROR("Takeoff failed!");
        return -1;
    }

    ROS_INFO("Takeoff successful!");

    ros::spin();
    return 0;
}
```

### Python 示例

```python
#!/usr/bin/env python3

import rospy
from drone_launch_controller.srv import (
    ConnectionStatus, ConnectionStatusRequest,
    DroneHealthCheck, DroneHealthCheckRequest,
    ArmMotors, ArmMotorsRequest,
    Takeoff, TakeoffRequest
)
from drone_launch_controller.msg import FlightStatus

class DroneController:
    def __init__(self):
        rospy.init_node('drone_control_example', anonymous=True)

        rospy.wait_for_service('/drone_launch_controller/connection_status')
        rospy.wait_for_service('/drone_launch_controller/drone_health_check')
        rospy.wait_for_service('/drone_launch_controller/arm_motors')
        rospy.wait_for_service('/drone_launch_controller/takeoff')

        self.connection_srv = rospy.ServiceProxy(
            '/drone_launch_controller/connection_status', ConnectionStatus)
        self.health_srv = rospy.ServiceProxy(
            '/drone_launch_controller/drone_health_check', DroneHealthCheck)
        self.arm_srv = rospy.ServiceProxy(
            '/drone_launch_controller/arm_motors', ArmMotors)
        self.takeoff_srv = rospy.ServiceProxy(
            '/drone_launch_controller/takeoff', Takeoff)

        self.status_sub = rospy.Subscriber(
            '/drone_launch_controller/flight_status', FlightStatus,
            self.status_callback)

        self.latest_status = None

    def status_callback(self, msg):
        self.latest_status = msg
        rospy.logdebug_throttle(1.0,
            f"Flight Status: Mode={msg.flight_mode}, "
            f"Alt={msg.current_altitude:.2f}, State={msg.state}")

    def check_connection(self):
        try:
            resp = self.connection_srv()
            rospy.loginfo(f"Connection: {'Connected' if resp.connected else 'Disconnected'}, "
                         f"System ID: {resp.system_id}")
            return resp.connected
        except rospy.ServiceException as e:
            rospy.logerr(f"Connection check failed: {e}")
            return False

    def perform_health_check(self):
        try:
            resp = self.health_srv()
            rospy.loginfo(f"Health Check: {'PASS' if resp.success else 'FAIL'}")
            rospy.loginfo(f"Battery: {resp.health_status.battery_voltage:.2f}V "
                         f"({resp.health_status.battery_percentage*100:.0f}%)")
            rospy.loginfo(f"GPS: {'OK' if resp.health_status.gps_status else 'FAIL'}, "
                         f"IMU: {'OK' if resp.health_status.imu_status else 'FAIL'}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Health check failed: {e}")
            return False

    def arm_motors(self, force=False):
        try:
            req = ArmMotorsRequest()
            req.force = force
            resp = self.arm_srv(req)
            rospy.loginfo(f"Arm Motors: {'SUCCESS' if resp.success else 'FAIL'} - {resp.message}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Arm motors failed: {e}")
            return False

    def takeoff(self, altitude, timeout=30.0):
        try:
            req = TakeoffRequest()
            req.altitude = altitude
            req.timeout = timeout
            resp = self.takeoff_srv(req)
            rospy.loginfo(f"Takeoff: {'SUCCESS' if resp.success else 'FAIL'} - {resp.message}")
            rospy.loginfo(f" Achieved altitude: {resp.achieved_altitude:.2f}m")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Takeoff failed: {e}")
            return False

def main():
    drone = DroneController()

    rospy.loginfo("=== Drone Control Example ===")

    if not drone.check_connection():
        rospy.logerr("Drone not connected!")
        return

    if not drone.perform_health_check():
        rospy.logwarn("Health check failed, proceeding anyway...")

    if not drone.arm_motors(force=False):
        rospy.logerr("Failed to arm motors!")
        return

    rospy.sleep(1.0)

    if not drone.takeoff(altitude=5.0, timeout=30.0):
        rospy.logerr("Takeoff failed!")
        return

    rospy.loginfo("Takeoff successful! Monitoring flight status...")

    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            if drone.latest_status:
                rospy.loginfo(f"Current: {drone.latest_status.flight_mode}, "
                             f"Alt: {drone.latest_status.current_altitude:.2f}m / "
                             f"{drone.latest_status.target_altitude:.2f}m, "
                             f"State: {drone.latest_status.state}")
            rate.sleep()
    except rospyInterruptException:
        pass

if __name__ == '__main__':
    main()
```

---

## 故障排除

### 常见问题

#### 1. 服务调用无响应

**症状**: 调用服务时长时间无响应或超时。

**可能原因**:
- drone_launch_controller 节点未启动
- MAVROS 未正确连接飞控

**解决方案**:
```bash
# 检查节点是否运行
rosnode list

# 检查服务是否可用
rosservice list | grep drone_launch_controller

# 重启节点
roslaunch drone_launch_controller drone_controller.launch
```

#### 2. 解锁失败

**症状**: arm_motors 服务返回失败。

**可能原因**:
- 无人机未处于 OFFBOARD 模式
- 安全开关未关闭
- 电池电压过低
- 飞行区域限制（地理围栏）

**解决方案**:
```bash
# 检查当前模式
rostopic echo /mavros/state

# 确保切换到 OFFBOARD 模式
rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"

# 使用强制解锁（谨慎使用）
rosservice call /drone_launch_controller/arm_motors "force: true"
```

#### 3. 起飞失败

**症状**: takeoff 服务返回失败或无人机不执行起飞。

**可能原因**:
- 未先执行电机解锁
- 未切换到 OFFBOARD 模式
- 目标高度设置不合理
- GPS 信号不足

**解决方案**:
```bash
# 确认电机已解锁
rostopic echo /mavros/state

# 检查 GPS 状态
rostopic echo /mavros/global_position/global

# 尝试较低高度
rosservice call /drone_launch_controller/takeoff "altitude: 2.0" "timeout: 30.0"
```

#### 4. 飞行状态消息无输出

**症状**: /drone_launch_controller/flight_status 话题无消息发布。

**可能原因**:
- 节点未正确初始化
- 与 MAVROS 连接中断

**解决方案**:
```bash
# 检查话题是否有发布者
rostopic info /drone_launch_controller/flight_status

# 检查 MAVROS 连接
rostopic echo /mavros/state
```

#### 5. 电池电压显示异常

**症状**: 电池状态显示为 NaN 或异常值。

**可能原因**:
- 飞控未正确配置电池传感器
- MAVROS 电池话题未发布

**解决方案**:
```bash
# 检查电池话题
rostopic echo /mavros/battery

# 检查飞控参数
# 在地面站中确认电池监控参数已正确配置
```

### 日志级别调整

如需查看更详细的调试信息，可以调整 ROS 日志级别：

```bash
# 查看所有话题
rqt_console

# 或在终端设置
export ROSCONSOLE_CONFIG_FILE=~/.ros/rosconsole.config
```

### 性能监控

监控无人机系统性能：

```bash
# 查看 CPU/内存使用
rosnode info /drone_launch_controller_node

# 查看话题带宽
rostopic bw /drone_launch_controller/flight_status

# 查看消息频率
rostopic hz /drone_launch_controller/flight_status
```

---

## 许可证

BSD License

## 维护者

anti_drone_team (anti_drone_team@example.com)
