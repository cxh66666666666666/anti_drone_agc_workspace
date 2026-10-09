cmake_minimum_required(VERSION 3.0.2)
project(rl_ground_avoidance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
  gazebo_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ground_utils.py
  scripts/ground_perception.py
  scripts/ground_env.py
  scripts/train_ground_dqn.py
  scripts/evaluate_ground_policy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
