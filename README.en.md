# Anti-Drone Air-Ground Collaboration Project

Simplified Chinese | [English](./README_EN.md)

## Project Overview

anti_drone_agc_workspace is a ROS-based experimental platform for anti-drone air-ground collaboration systems. This project integrates core technologies including drone control, LiDAR SLAM, LQR trajectory tracking control, and Gazebo simulation, aiming to research and validate algorithms related to air-ground collaborative detection and tracking tasks.

### Core Features

- **Drone Launch and Control**: Achieves safe takeoff, landing, and mode switching via drone_launch_controller
- **LiDAR SLAM**: Integrates A-LOAM for high-precision map building
- **LQR Trajectory Tracking**: Trajectory tracking controller based on Linear Quadratic Regulator
- **Obstacle Generation and Analysis**: Provides auxiliary scripts for trajectory analysis and obstacle generation
- **Gazebo Simulation Support**: Complete virtual simulation environment based on XTDrone

### Technology Stack

| Category | Technology |
|----------|------------|
| Operating System | Ubuntu + ROS |
| Simulation Platform | Gazebo + XTDrone |
| Sensors | Velodyne VLP-16/HDL-32 LiDAR |
| Flight Controller Firmware | PX4 |
| Middleware | MAVROS |
| Core Algorithms | A-LOAM, LQR |

## Directory Structure

```
anti_drone_agc_workspace/
├── docs/                      # Documentation directory
│   ├── learning_plan.md       # Future development and learning plan
│   ├── project_guide.md       # Project development guide
│   └── xtdrone_simulation_guide.md  # XTDrone simulation guide
├── scripts/                  # Auxiliary scripts
│   ├── analyze_trajectory.py    # Trajectory analysis and obstacle generation
│   ├── spawn_obstacles.py      # Obstacle generation script
│   ├── start_drone_test.sh    # Drone test launch script
│   └── start_lqr_tracker_sim.sh  # LQR tracking simulation launch script
└── src/                     # Source code workspace
    ├── A-LOAM/               # LiDAR SLAM algorithm package
    ├── air_ground_core/      # Air-ground collaboration core control package
    ├── drone_launch_controller/  # Drone launch controller
    ├── lqr_trajectory_tracker/  # LQR trajectory tracker
    ├── gazebo_ros_pkgs/      # Gazebo ROS integration package
    └── velodyne_*            # Velodyne LiDAR support packages
```

## Build and Run

### System Requirements

- Ubuntu 18.04 or higher
- ROS Kinetic/Melodic
- Catkin Tools
- Gazebo 9+

### Build the Project

```bash
# Enter the workspace directory
cd ~/anti_drone_agc_workspace

# Initialize workspace (if needed)
source /opt
```