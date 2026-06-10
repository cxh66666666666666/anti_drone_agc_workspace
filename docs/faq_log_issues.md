# ROS 日志常见问题与解决方法

## 一、中文日志显示乱码（?????）

### 现象
ROS_INFO / ROS_WARN / ROS_ERROR 中的中文字符全部显示为 `????`：

```
[INFO] [12345678.90, 30.5]: [Takeoff] ????????????????????: 0 -> 4
```

### 原因
ROS 节点进程默认使用 `C` locale，不识别 UTF-8 多字节字符。`printf` 系列函数在此 locale 下会将非 ASCII 字节替换为 `?`。

### 解决方法

两步：

**1. 系统环境设置（`~/.bashrc`）**

```bash
export LC_ALL=en_US.UTF-8
```

**2. 每个 ROS 节点 `main()` 中调用 `setlocale`**

在 `ros::init()` 之前加两行：

```cpp
#include <clocale>

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");  // 继承环境变量中的 UTF-8 区域设置
    ros::init(argc, argv, "node_name");
    // ...
}
```

`setlocale(LC_ALL, "")` 让进程读取环境变量（`LC_ALL` / `LANG`）中的区域设置，从而正确输出中文。

### 已修改的文件

| 文件 | 节点 |
|------|------|
| `src/lqr_trajectory_tracker/src/lqr_tracker_node.cpp` | lqr_tracker_node |
| `src/drone_launch_controller/src/drone_launch_controller_node.cpp` | drone_launch_controller（含 TakeoffController、HealthChecker 等子组件） |
| `src/air_ground_core/src/car_commander.cpp` | car_commander |

> 注意：`drone_launch_controller_node` 的主线程设了 locale 后，其内部所有子组件（TakeoffController、HealthChecker、FlightStatusPublisher）的日志也自动生效，因为它们共享同一进程。

### 后续新增节点

遇到含中文日志的新节点，按上述第 2 步在 `main()` 中添加即可。

---

## 二、TF2 警告刷屏

### 现象

```
[WARN] [1781102883.652677840, 60.760000000]: Invalid argument passed to canTransform argument source_frame in tf2 frame_ids cannot be empty
[WARN] [1781102883.685067011, 60.792000000]: Invalid argument passed to canTransform argument source_frame in tf2 frame_ids cannot be empty
```

### 原因
RViz 在渲染 TF/可视化时，内部调用 `tf2::BufferCore::canTransform()` 时传入了空的 `source_frame`。此警告来自 tf2 库本身，不影响功能，但频率较高时会刷屏干扰日志阅读。

### 解决方法

用 ROS console 配置文件将 tf2 相关 logger 的日志级别调为 ERROR（只显示错误，不显示 WARN/INFO）。

**1. 创建配置文件**

内容（已创建于 `src/lqr_trajectory_tracker/config/rosconsole.config` 和 `src/drone_launch_controller/config/rosconsole.config`）：

```
log4j.logger.ros.tf2=ERROR
log4j.logger.ros.tf2_ros=ERROR
log4j.logger.ros.tf=ERROR
```

**2. 在 launch 文件中引用**

在每个 `<node>` 标签内添加：

```xml
<env name="ROSCONSOLE_CONFIG_FILE" value="$(find package_name)/config/rosconsole.config"/>
```

### 已修改的 launch 文件

| 文件 | 说明 |
|------|------|
| `src/lqr_trajectory_tracker/launch/lqr_tracker_sim.launch` | RViz + lqr_tracker_node 均添加 |
| `src/drone_launch_controller/launch/drone_controller.launch` | 所有节点均添加 |

### 全局备选方案

如果有通过命令行直接启动的节点（不经过 launch 文件），可在 `~/.bashrc` 中添加：

```bash
export ROSCONSOLE_CONFIG_FILE=/home/ubuntu/anti_drone_agc_workspace/src/lqr_trajectory_tracker/config/rosconsole.config
```
