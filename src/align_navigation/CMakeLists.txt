cmake_minimum_required(VERSION 2.8.3)
project(align_navigation)

find_package(catkin REQUIRED COMPONENTS
	nav_core
	roscpp
	std_msgs
	angles
	costmap_2d
	dynamic_reconfigure
	geometry_msgs
	navfn
	nav_msgs
	pluginlib
	roscpp
	tf2_geometry_msgs
	tf2_ros
)
catkin_package(
CATKIN_DEPENDS 
  	roscpp std_msgs costmap_2d dynamic_reconfigure 
    geometry_msgs nav_core navfn nav_msgs
    pluginlib tf2_ros
)

include_directories(
  include/dubins_planner
  include/reeds_shepp_planner
  ${catkin_INCLUDE_DIRS}
)

add_library(dubins_planner_lib 
	src/global_dubins.cpp
	src/dubins.cpp
)

add_library(reeds_shepp_planner_lib 
	src/global_reeds_shepp.cpp
	src/reeds_shepp.cpp
)
