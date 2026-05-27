cmake_minimum_required(VERSION 3.0.2)
project(lqr_trajectory_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    mavros_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(lqr_tracker_node
  src/lqr_tracker_node.cpp
  src/lqr_controller.cpp
  src/trajectory_generator.cpp
)

add_dependencies(lqr_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lqr_tracker_node
  ${catkin_LIBRARIES}
)

install(TARGETS lqr_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
