cmake_minimum_required(VERSION 2.8.3)
project(lightware_sf40_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
  std_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS 
	roscpp
  	serial
  	std_msgs
	message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${modular_lib_pkg_INCLUDE_DIRS} ${serial_INCLUDE_DIRS}
)

add_executable(sf40_node src/sf40_node.cpp)
add_executable(farzi_node src/farzi_node.cpp)
# add_dependencies(sf40_node)
target_link_libraries(sf40_node 
 ${catkin_LIBRARIES} ${modular_lib_pkg_LIBRARIES} ${serial_LIBRARIES}
)
target_link_libraries(farzi_node 
 ${catkin_LIBRARIES} ${modular_lib_pkg_LIBRARIES} ${serial_LIBRARIES}
)