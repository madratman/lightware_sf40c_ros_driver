cmake_minimum_required(VERSION 2.8.3)
project(lightware_sf40_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}") # use cpp11

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
target_link_libraries(sf40_node 
 ${catkin_LIBRARIES} ${modular_lib_pkg_LIBRARIES} ${serial_LIBRARIES}
)
