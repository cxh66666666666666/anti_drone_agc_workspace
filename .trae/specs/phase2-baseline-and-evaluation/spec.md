# 第二阶段：A*/RRT 基线实现与综合评估 Spec

## Why
当前 DQN 已在前两个静态障碍物场景下训练完成（成功率 100%，平均 reward ~32），但缺乏与传统路径规划算法（A*/RRT）的对比基线，且场景覆盖仅到稀疏静态（2 个障碍物）。指导书明确要求第 5 周实现 A*/RRT 基线，第 6 周完成动态障碍和综合评估。

## What Changes
- 新增 A* 路径规划基线模块（2D 占据栅格 + 路径平滑）
- 新增 RRT 路径规划基线模块（连续空间采样 + 碰撞检测）
- 新增独立评估脚本 `evaluate_policy.py`，支持 DQN 模型、规则基线、A*、RRT 的统一评估
- 新增场景生成脚本 `generate_world.py`，支持从 YAML 配置生成 Gazebo world 文件
- 新增场景配置文件（sparse 3-5 障碍物、dense 8-20 障碍物）
- 训练日志和评估结果按指导书第 6.2 节格式组织

## Impact
- Affected specs: 新增 `src/rl_obstacle_avoidance/scripts/` 下的 A*、RRT、评估、场景生成脚本
- Affected code: 无现有代码修改，纯新增

## ADDED Requirements

### Requirement: A* 路径规划基线
系统 SHALL 提供基于 2D 占据栅格的 A* 路径规划器，输出全局航点交由 LQR 跟踪。

#### Scenario: 稀疏障碍物场景规划成功
- **WHEN** 给定起点 (0,0,2)、终点 (12,8,2) 和 3-5 个静态障碍物
- **THEN** A* 规划器输出一条无碰撞的栅格路径，经平滑后用于 LQR 跟踪

#### Scenario: 障碍物膨胀安全半径
- **WHEN** 进行碰撞检测
- **THEN** 障碍物膨胀安全半径 0.5m-1.0m，确保无人机安全通过

### Requirement: RRT 路径规划基线
系统 SHALL 提供基于连续空间采样的 RRT 路径规划器，输出全局航点交由 LQR 跟踪。

#### Scenario: 连续空间路径规划
- **WHEN** 给定起点和终点
- **THEN** RRT 在连续空间中采样、扩展树节点，找到可行路径后进行 shortcut 平滑

#### Scenario: 碰撞检测
- **WHEN** 扩展 RRT 树节点
- **THEN** 使用线段与障碍物膨胀体的距离进行碰撞检测

### Requirement: 独立评估脚本
系统 SHALL 提供 `evaluate_policy.py` 脚本，支持对 DQN 模型、规则基线、A*、RRT 的统一评估。

#### Scenario: 多算法多场景评估
- **WHEN** 运行 `evaluate_policy.py --algorithms dqn,astar,rrt --scenario medium_static`
- **THEN** 输出包含成功率、碰撞率、平均路径长度、平均规划时间、轨迹平滑度的汇总表

#### Scenario: 固定随机种子复现
- **WHEN** 指定 `--seed` 参数
- **THEN** 评估结果可完全复现

### Requirement: 场景生成脚本
系统 SHALL 提供 `generate_world.py` 脚本，从 YAML 配置生成 Gazebo world 文件。

#### Scenario: 从 YAML 生成 world
- **WHEN** 运行 `generate_world.py --config scenarios_medium.yaml`
- **THEN** 生成对应的 `.world` 文件，包含指定数量、位置、尺寸的障碍物

### Requirement: 场景配置
系统 SHALL 提供三类场景配置文件：
- `scenarios_sparse.yaml`：3-5 个静态障碍物
- `scenarios_dense.yaml`：8-20 个静态障碍物
- 动态障碍物场景（后续阶段）