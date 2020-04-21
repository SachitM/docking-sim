cmake_minimum_required(VERSION 2.8.3)
project(hms_client)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   hms_msg.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ping_pong.srv
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hms_client
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(hms_client src/hms_client.cpp)
add_dependencies(hms_client ${catkin_EXPORTED_TARGETS})
add_dependencies(hms_client node1_generate_messages_cpp)
add_dependencies(hms_client node2_generate_messages_cpp)
add_dependencies(hms_client hms_client_generate_messages_cpp)
target_link_libraries(hms_client ${catkin_LIBRARIES} )