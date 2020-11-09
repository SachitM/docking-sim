cmake_minimum_required(VERSION 2.8.3)

project(phz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  state_machine
)

<<<<<<< HEAD
catkin_package(
  CATKIN_DEPENDS state_machine
  )
=======
catkin_package(CATKIN_DEPENDS state_machine)
>>>>>>> 096d22c1453cb67df0ddbc8bb941118a43426a9c

include_directories(${catkin_INCLUDE_DIRS})

add_executable(identify_phz_node src/identify_phz.cpp)
add_dependencies(identify_phz_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(identify_phz_node 
	${catkin_LIBRARIES}
)

add_executable(groundtruth_publisher_node src/groundtruth_publisher.cpp)
target_link_libraries(groundtruth_publisher_node 
	${catkin_LIBRARIES}
)

add_executable(waypoint_publisher_node src/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher_node 
	${catkin_LIBRARIES}
)

add_executable(viz_node src/viz.cpp)
target_link_libraries(viz_node 
	${catkin_LIBRARIES}
)
