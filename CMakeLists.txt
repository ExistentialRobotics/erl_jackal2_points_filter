cmake_minimum_required(VERSION 2.8.3)
project(erl_jackal2_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(erl_jackal2_filter src/erl_jackal2_lidar_filter_node.cpp)

add_dependencies(erl_jackal2_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(erl_jackal2_filter ${catkin_LIBRARIES})

install(TARGETS erl_jackal2_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
