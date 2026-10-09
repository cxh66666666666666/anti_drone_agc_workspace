cmake_minimum_required(VERSION 3.0.2)
project(car_launch_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  gazebo_msgs
  message_generation
)

add_message_files(
  FILES
    CarStatus.msg
    CarHealthStatus.msg
)

add_service_files(
  FILES
    ConnectionStatus.srv
    CarHealthCheck.srv
    EnableMotion.srv
    Drive.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    std_srvs
    gazebo_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_launch_controller_node
  src/car_launch_controller_node.cpp
  src/health_checker.cpp
  src/motion_controller.cpp
  src/car_status_publisher.cpp
)

add_dependencies(car_launch_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(car_launch_controller_node
  ${catkin_LIBRARIES}
)

install(TARGETS car_launch_controller_node
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