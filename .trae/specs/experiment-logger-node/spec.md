# 实验数据记录节点 Spec

## Why
当前规则避障 baseline 已跑通，但缺乏系统化的实验数据记录。在进入 DQN 强化学习之前，需要先建立数据采集能力，才能量化对比 baseline 与后续 RL 策略的效果。

## What Changes
- 新增 `experiment_logger_node.py`：纯数据记录节点，不改控制链路，订阅关键话题并输出 CSV
- 新增对应的 launch 文件 `experiment_logger.launch`
- 在 `CMakeLists.txt` 中添加该脚本的安装项

## Impact
- Affected specs: 无（新功能）
- Affected code: `src/rl_obstacle_avoidance/scripts/experiment_logger_node.py`（新增）、`src/rl_obstacle_avoidance/launch/experiment_logger.launch`（新增）、`src/rl_obstacle_avoidance/CMakeLists.txt`（修改）

## ADDED Requirements

### Requirement: CSV 实验数据记录
系统 SHALL 提供一个独立的数据记录节点，订阅以下话题并以 CSV 格式输出实验数据，不修改任何控制链路。

#### Scenario: 正常记录所有话题数据
- **WHEN** 所有订阅话题均有数据发布
- **THEN** 节点按固定频率（默认 10 Hz）将一行 CSV 记录写入文件，包含时间戳和所有话题字段

#### Scenario: 部分话题缺失时的优雅降级
- **WHEN** 某个话题尚未收到首条消息
- **THEN** 对应字段填写 NaN，不阻塞其他字段的记录

#### Scenario: 文件自动命名
- **WHEN** 节点启动
- **THEN** 自动生成带时间戳的 CSV 文件名（如 `experiment_20260708_143052.csv`），保存到可配置的输出目录

### Requirement: 记录字段定义
系统 SHALL 记录以下字段：

| 字段 | 来源话题 | 来源类型 | 说明 |
|------|----------|----------|------|
| `timestamp` | ROS 时间 | `rospy.Time.now()` | 记录时刻 |
| `pose_x, pose_y, pose_z` | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 无人机位置 |
| `pose_qx, pose_qy, pose_qz, pose_qw` | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 无人机姿态四元数 |
| `offset_x, offset_y, offset_z` | `/rl_planner/local_offset` | `geometry_msgs/Vector3Stamped` | 避障偏移量 |
| `action` | `/rl_planner/action` | `std_msgs/Int32` | 0=安全, 1=左绕, 2=右绕, 3=紧急 |
| `position_error` | `/lqr_tracker/position_error` | `std_msgs/Float64` | LQR 位置跟踪误差 (m) |
| `adjusted_ref_x, adjusted_ref_y, adjusted_ref_z` | `/lqr_tracker/adjusted_ref` | `geometry_msgs/PoseStamped` | LQR 修正后参考点 |

#### Scenario: 验证字段完整性
- **WHEN** 查看输出的 CSV 文件
- **THEN** 表头包含上述所有字段，每行数据与表头一一对应

### Requirement: 避障状态衍生字段
系统 SHALL 根据原始数据计算并记录以下衍生字段：

| 字段 | 计算方式 | 说明 |
|------|----------|------|
| `is_avoiding` | `action != 0` | 是否正在执行避障 |
| `offset_magnitude_xy` | `sqrt(offset_x² + offset_y²)` | 水平偏移幅值 |

#### Scenario: 避障完成判定
- **WHEN** 连续 1 秒内 `action == 0`
- **THEN** 视为一次避障事件完成（仅用于后续分析参考，不写入 CSV 行级字段）

### Requirement: Launch 文件
系统 SHALL 提供一个独立的 launch 文件用于启动实验记录节点。

#### Scenario: 启动实验记录
- **WHEN** 执行 `roslaunch rl_obstacle_avoidance experiment_logger.launch`
- **THEN** 实验记录节点启动，参数可配置（输出目录、记录频率）
