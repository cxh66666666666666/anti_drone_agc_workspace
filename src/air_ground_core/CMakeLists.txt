cmake_minimum_required(VERSION 3.0.2)
project(air_ground_core)

## 协同层（M0 接口契约 v1.1）
## 语言策略：实时/控制/任务链路节点一律 C++；
## 协同层 12 个节点 + agc_common 已全部为 C++，scripts/ 下仅保留两个
## 离线工具（summarize_runs.py、audit_interfaces.py）。

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf2_ros
  gazebo_msgs
  mavros_msgs
)

## 系统依赖：YAML 解析（场景文件）与 SHA1（run 哈希）
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

################################################
## 消息
################################################
add_message_files(
  FILES
  TargetTrack.msg
  TargetTrackArray.msg
  AgcTaskState.msg
  # M4 感知与融合（契约 v1.2）
  TargetObservation.msg
  ObstacleObservation.msg
  SensorObservation.msg
  AirObservationArray.msg
  ObstacleTrack.msg
  ObstacleTrackArray.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agc_core
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp std_msgs std_srvs tf2_ros gazebo_msgs mavros_msgs
)

###########
## 共享库与节点
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenSSL_INCLUDE_DIR}
)

## 共享基础模块（对应 Python 的 scripts/agc_common.py）
add_library(agc_core
  src/agc_common.cpp
)
add_dependencies(agc_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agc_core
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  OpenSSL::Crypto
)

## 协同层运行时节点（全部 C++）
set(AGC_CPP_NODES
  ground_drive_adapter
  ground_commander
  task_manager
  air_odom_adapter
  ground_odom_adapter
  ground_planner
  gt_adapter
  intruder_driver
  map_adapter
  scenario_reset
  air_control_bridge
  agc_run_logger
  # M4 感知前端与多源融合（契约 v1.2）
  air_perception
  air_fusion
  # N2 无人机跟随移动目标（安全距离带）
  air_follow
)

foreach(node ${AGC_CPP_NODES})
  add_executable(${node} src/${node}.cpp)
  add_dependencies(${node} agc_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node}
    agc_core
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    OpenSSL::Crypto
  )
endforeach()

###########
### 离线工具（见 AGENTS.md 第四节：不迁移，保持 Python）
###########
catkin_install_python(PROGRAMS
  scripts/summarize_runs.py
  scripts/audit_interfaces.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## 安装
#############
install(DIRECTORY launch config worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)