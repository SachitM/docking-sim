cmake_minimum_required(VERSION 2.8.3)

project(align_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  tf
  state_machine
)

catkin_package(
  CATKIN_DEPENDS state_machine
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(odom_transformation_node src/odom_transformation_node.cpp)
#add_executable(goal_pub_node src/goal_pub.cpp)

target_link_libraries(odom_transformation_node
  ${catkin_LIBRARIES}
 )

#target_link_libraries(goal_pub_node
#  ${catkin_LIBRARIES}
#)

install(PROGRAMS
   scripts/ackermann_controller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
