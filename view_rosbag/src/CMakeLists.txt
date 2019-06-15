cmake_minimum_required(VERSION 2.8.9)
cmake_policy(SET CMP0015 NEW)

project (view_rosbag)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-write-strings -g -pthread")

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  grid_map_core
#  grid_map_msgs
#  grid_map_cv
#  sensor_msgs
#  nav_msgs
#  std_msgs
#  geometry_msgs
#  cv_bridge
#  rosbag
#  tf
#  visualization_msgs
#)

include_directories(/opt/ros/kinetic/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
include_directories(/opt/ros/kinetic/include/opencv-3.3.1-dev/)

# Creating a variable
file(GLOB GRID_MAP_SRC "*.cpp")

# Create an executable using the attached cpp files
add_executable(view_rosbag ${GRID_MAP_SRC})

target_link_libraries(view_rosbag /opt/ros/kinetic/lib/libroscpp_serialization.so /opt/ros/kinetic/lib/libroscpp.so /opt/ros/kinetic/lib/librosconsole.so  /opt/ros/kinetic/lib/libcpp_common.so /opt/ros/kinetic/lib/librostime.so rt ${Boost_LIBRARIES}   ${catkin_LIBRARIES})