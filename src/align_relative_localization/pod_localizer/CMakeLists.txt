cmake_minimum_required(VERSION 2.8.3)
project(pod_localizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  apriltag_ros
  image_geometry
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pod_localizer
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tag_pod_localizer src/april_tag_goal_pub.cpp)
target_link_libraries(tag_pod_localizer
  ${catkin_LIBRARIES}
 )

add_dependencies(tag_pod_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(laser_pod_localizer src/goal_pub.cpp)
target_link_libraries(laser_pod_localizer
  ${catkin_LIBRARIES}
)

#install(TARGETS
#  tag_pod_localizer
#
#)

