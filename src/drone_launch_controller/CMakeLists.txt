cmake_minimum_required(VERSION 3.0.2)
project(drone_launch_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  std_srvs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    mavros_msgs
    std_srvs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drone_launch_controller_node
  src/drone_launch_controller_node.cpp
  src/drone_connector.cpp
  src/health_checker.cpp
  src/arm_controller.cpp
  src/takeoff_controller.cpp
  src/flight_status_publisher.cpp
)

add_dependencies(drone_launch_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(drone_launch_controller_node
  ${catkin_LIBRARIES}
)

install(TARGETS drone_launch_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
