cmake_minimum_required(VERSION 2.8.3)
project(obstacle_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hms_client
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_2d
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_2d src/obstacle_2d.cpp)
target_link_libraries(obstacle_2d ${catkin_LIBRARIES})
add_dependencies(obstacle_2d hms_client_generate_messages_cpp)
