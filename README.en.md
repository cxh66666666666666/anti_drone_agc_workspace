# Anti-Drone Air-Ground Collaboration Project

[简体中文](./README.md) | English

## Project Overview

anti_drone_agc_workspace is a ROS-based experimental platform for anti-drone air-ground collaboration systems. This project integrates core technologies including drone control, LiDAR SLAM, LQR trajectory tracking control, DQN-based reinforcement learning for local obstacle avoidance, and Gazebo simulation, aiming to research and validate algorithms related to air-ground collaborative detection and tracking tasks.

### Core Features

- **Drone Launch and Control**: Achieves safe takeoff, landing, and mode switching via drone_launch_controller
- **LiDAR SLAM**: Integrates A-LOAM for high-precision map building
- **LQR Trajectory Tracking**: Trajectory tracking controller based on Linear Quadratic Regulator, supporting linear, circular, helical, fixed horizontal figure-8, and step (hover) trajectories
- **Rule-Based Obstacle Avoidance**: Sector-distance-based avoidance supporting single- and multi-obstacle detours
- **DQN Reinforcement Learning Avoidance**: Local avoidance training loop (perception → action → LQR offset) with experience replay, greedy evaluation, and model management
- **Experiment Logging and Evaluation**: experiment_logger_node records flight data; scripts/evaluate_experiment.py computes six evaluation metrics
- **Obstacle Generation and Analysis**: Auxiliary scripts for trajectory analysis and obstacle generation
- **Gazebo Simulation Support**: Complete PX4 SITL + Gazebo virtual simulation environment

### Technology Stack

| Category | Technology |
|----------|------------|
| Operating System | Ubuntu 20.04 + ROS Noetic |
| Simulation Platform | Gazebo + PX4 SITL (iris_3d_lidar airframe, XTDrone model library compatible) |
| Sensors | Velodyne VLP-16/HDL-32/HDL-64 LiDAR |
| Flight Controller Firmware | PX4 |
| Middleware | MAVROS |
| Core Algorithms | A-LOAM, LQR, DQN (PyTorch, Dueling + Double DQN + prioritized experience replay) |
| Path Planning | Sector perception + rule-based avoidance / DQN local avoidance |

## Directory Structure

```
anti_drone_agc_workspace/
├── docs/                          # Documentation directory
│   ├── 指导书/                    # Course guide (RL-based UAV obstacle avoidance and path planning)
│   ├── learning_plan.md           # Future development and learning plan
│   ├── dqn_training.md            # DQN avoidance training guide
│   ├── faq_log_issues.md          # ROS logging FAQ and fixes
│   └── project_guide.md           # Project development guide (planned)
├── scripts/                       # Auxiliary scripts
│   ├── start_sim.sh               # Unified simulation launcher (sim/takeoff/lqr/perception modes)
│   ├── analyze_trajectory.py      # Trajectory analysis and safe obstacle placement
│   ├── evaluate_experiment.py     # Experiment CSV analysis, six evaluation metrics
│   └── generate_pptx.py           # Project presentation PPT generator
├── experiment_data/               # Experiment CSVs written by experiment_logger_node
├── training_runs/                 # DQN training run directories (configs, logs, models)
├── logs/                          # Simulation run logs
└── src/                           # Source code workspace
    ├── A-LOAM/                    # LiDAR SLAM algorithm package (VLP-16/HDL-32/HDL-64)
    ├── air_ground_core/           # Air-ground core package (ground vehicle control node)
    ├── drone_launch_controller/   # Drone launch controller (connection/health check/arm/takeoff)
    ├── lqr_trajectory_tracker/    # LQR trajectory tracker
    ├── rl_obstacle_avoidance/     # Perception + rule avoidance + DQN training/evaluation package
    ├── gazebo_ros_pkgs/           # Gazebo ROS integration packages
    ├── velodyne_description/      # Velodyne model descriptions
    ├── velodyne_gazebo_plugins/   # Velodyne Gazebo sensor plugins
    └── velodyne_simulator/        # Velodyne simulation metapackage
```

## Quick Start

The unified launcher `scripts/start_sim.sh` supports four modes:

| Mode | Command | Description |
|------|---------|-------------|
| Simulation | `./scripts/start_sim.sh sim` | Start PX4 SITL + Gazebo + MAVROS only |
| Takeoff | `./scripts/start_sim.sh takeoff --altitude 5.0` | Simulation + takeoff controller + takeoff |
| LQR Tracking | `./scripts/start_sim.sh lqr --with-perception` | Simulation + perception/avoidance + takeoff + LQR tracking |
| Perception | `./scripts/start_sim.sh perception` | Simulation + perception node |

Common options: `--trajectory <type>` (linear/circular/helical/fixed_horizontal/step), `--with-perception` (start perception + rule planner nodes), `--training` (perception only, for exclusive DQN control), `--with-experiment-logger` (start the experiment logger node), `--obstacle <x,y,z>` (spawn a static obstacle, repeatable), `--headless` (disable the Gazebo GUI).

```bash
# One-shot launch: three-obstacle rule-based avoidance experiment
# (perception + rule avoidance + takeoff + LQR tracking + experiment logging)
./scripts/start_sim.sh lqr --with-perception --with-experiment-logger \
    --obstacle 5,0,2 --obstacle 10,2,2 --obstacle 15,-2,2
```

## Rule-Based Obstacle Avoidance

### Startup Order

Start the simulation environment (perception + planning + takeoff + LQR tracking + experiment logging):

```bash
# One-shot launch: three-obstacle avoidance experiment
./scripts/start_sim.sh lqr --with-perception --with-experiment-logger \
    --obstacle 5,0,2 --obstacle 10,2,2 --obstacle 15,-2,2
```

**Manual step-by-step startup:**

```bash
# 1. Start the simulation environment (PX4 + Gazebo + MAVROS)
./scripts/start_sim.sh sim

# 2. New terminal: start the perception and avoidance planner nodes
roslaunch rl_obstacle_avoidance perception.launch &
roslaunch rl_obstacle_avoidance rl_planner_node.launch &

# 3. New terminal: start the takeoff controller and take off
./scripts/start_sim.sh takeoff --altitude 5.0

# 4. New terminal: start the LQR tracker and experiment logging
./scripts/start_sim.sh lqr --with-experiment-logger
```

See `src/rl_obstacle_avoidance/config/rule_avoidance.yaml` for detailed parameter configuration.

### Node Pipeline

```text
/iris/velodyne_points ──→ perception_node ──→ /obstacle_features
                                                    │
                                                    v
                        rl_planner_node ──→ /rl_planner/action (discrete action)
                             │              /rl_planner/local_goal (local goal pose)
                             │              /rl_planner/local_offset (LQR reference offset)
                             v
                lqr_tracker_node (LQR tracking) ──→ /mavros/setpoint_position/local
```

### Experimental Validation

The following three scenarios have been validated in Gazebo simulation:

| Scenario | Obstacle Configuration | Result |
|----------|------------------------|--------|
| No obstacle | None | The drone flies the figure-8 trajectory normally with no abnormal deviation |
| Single obstacle | (5, 0, 2) 0.3×0.3×3m pillar | Stably detours the obstacle and resumes the original trajectory |
| Three obstacles | (5,0,2) (10,2,2) (15,-2,2) | Successfully detours three obstacles in succession |

Experiment data is recorded by `experiment_logger_node` in `experiment_data/` and analyzed with `scripts/evaluate_experiment.py`, which computes six metrics: avoidance success rate, minimum obstacle distance, tracking error, path length, avoidance duration, and offset jitter:

```bash
python3 scripts/evaluate_experiment.py experiment_data/experiment_*.csv
```

## DQN Training

DQN training uses independent run directories to save configuration, training logs, greedy evaluation logs, and best/last models.
See [DQN Avoidance Training](docs/dqn_training.md) for startup instructions, metric definitions, and artifact descriptions.

## Build and Run

### System Requirements

- Ubuntu 20.04 + ROS Noetic
- Catkin Tools
- Gazebo (the version paired with ROS Noetic; PX4 SITL uses the iris_3d_lidar airframe)
- Python 3 + PyTorch (required for DQN training)
- PX4 Firmware source (`~/PX4_Firmware`, used for PX4 SITL; the XTDrone model library is optional)

### Build the Project

```bash
# Enter the workspace directory
cd ~/anti_drone_agc_workspace

# Build with catkin_tools
catkin build

# Source the workspace environment
source devel/setup.bash
```

### Run

```bash
# Source the ROS environment and start the simulation
# (the script sets the PX4 SITL environment variables itself)
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Start the simulation environment
./scripts/start_sim.sh sim

# One-shot LQR tracking (with perception and rule-based avoidance)
./scripts/start_sim.sh lqr --with-perception
```

## Documentation Index

- [DQN Avoidance Training](docs/dqn_training.md)
- [ROS Logging FAQ and Fixes](docs/faq_log_issues.md)
- [Future Development and Learning Plan](docs/learning_plan.md)
- [Course Guide](docs/指导书/基于强化学习的无人机自主避障与路径规划系统.md)
