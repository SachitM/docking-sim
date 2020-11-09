cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   StateIn.msg
   StateOut.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES state_machine
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

<<<<<<< HEAD
add_executable(state_machine_node src/state_machine_node.cpp)
add_dependencies(state_machine_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine_node ${catkin_LIBRARIES})
=======
add_executable(${PROJECT_NAME}_node src/state_machine_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

>>>>>>> 096d22c1453cb67df0ddbc8bb941118a43426a9c
