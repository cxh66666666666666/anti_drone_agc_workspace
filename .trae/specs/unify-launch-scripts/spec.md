# 统一启动脚本 Spec

## Why
现有5个脚本功能重叠、入口分散、维护成本高。各包已解耦，需要一个统一脚本按模式一键启动不同组件组合，同时删除旧脚本避免混淆。

## What Changes
- 新建统一脚本 `scripts/start_sim.sh`，支持多种运行模式（通过参数切换）
- 保留 `scripts/spawn_obstacles.py` 和 `scripts/analyze_trajectory.py`（它们是功能模块，不是独立启动脚本）
- 删除旧启动脚本：`start_drone_test.sh`、`run_takeoff_demo.sh`、`start_lqr_tracker_sim.sh`
- 集成 `rl_obstacle_avoidance` 感知节点包
- 环境变量设置集中管理，避免重复代码

## Impact
- Affected specs: N/A
- Affected code: `scripts/*.sh`（仅启动类脚本）

## ADDED Requirements

### Requirement: 统一启动脚本
系统 SHALL 提供一个单一入口 `scripts/start_sim.sh`，通过命令行参数选择运行模式。

#### Scenario: 查看帮助
- **WHEN** 用户执行 `./scripts/start_sim.sh --help` 或 `-h`
- **THEN** 显示所有可用模式及参数说明

#### Scenario: 起飞模式 (takeoff)
- **WHEN** 用户执行 `./scripts/start_sim.sh takeoff [--altitude 5.0]`
- **THEN** 依次执行：清理旧进程 → 启动PX4 SITL + MAVROS（后台） → 等待MAVROS连接 → 启动 drone_launch_controller → 发送起飞指令
- **THEN** 支持 Ctrl+C 停止所有进程

#### Scenario: LQR跟踪模式 (lqr)
- **WHEN** 用户执行 `./scripts/start_sim.sh lqr [--trajectory fixed_horizontal] [--with-obstacles] [--with-perception]`
- **THEN** 依次执行：清理旧进程 → 启动PX4 SITL + MAVROS（后台） → 等待MAVROS连接 → [可选]加载障碍物 → [可选]启动感知节点 → 启动 LQR轨迹跟踪器

#### Scenario: 仅启动仿真 (sim)
- **WHEN** 用户执行 `./scripts/start_sim.sh sim`
- **THEN** 仅执行：清理旧进程 → 启动PX4 SITL + MAVROS（后台） → 等待MAVROS连接

#### Scenario: 感知模式 (perception)
- **WHEN** 用户执行 `./scripts/start_sim.sh perception`
- **THEN** 依次执行：清理旧进程 → 启动PX4 SITL + MAVROS（后台） → 等待MAVROS连接 → 启动 rl_obstacle_avoidance 感知节点

## REMOVED Requirements
### Requirement: 分散的独立启动脚本
**Reason**: 功能整合到统一脚本中，避免维护多份重复代码
**Migration**: 用户改用 `./scripts/start_sim.sh <mode>` 替代旧脚本

旧脚本对照：
| 旧脚本 | 新命令 |
|---|---|
| `start_drone_test.sh` | `start_sim.sh takeoff` |
| `run_takeoff_demo.sh` | `start_sim.sh takeoff --altitude 5.0` |
| `start_lqr_tracker_sim.sh` | `start_sim.sh lqr --with-obstacles` |
