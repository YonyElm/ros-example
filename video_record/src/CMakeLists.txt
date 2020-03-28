cmake_minimum_required(VERSION 2.8.9)
cmake_policy(SET CMP0015 NEW)

project (video_record)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-write-strings -g -pthread")

# Loading package settings, will store them in variable ${catkin_LIBRARIES} / ${catkin_LIBS}
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_msgs
  grid_map_cv
  grid_map_ros
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  rosbag
  tf
  visualization_msgs
)

# Used in order to include these paths in library search (-I flag)
include_directories(/opt/ros/kinetic/include
                    ${Boost_INCLUDE_DIRS}
                    /usr/include/eigen3
                    /opt/ros/kinetic/include/opencv-3.3.1-dev/)

# Creating a variable
file(GLOB GRID_MAP_SRC "*.cpp")

# Create an executable using the attached cpp files
add_executable(video_record ${GRID_MAP_SRC})

# Used in order for the linker to link libraries context (-l flag)
target_link_libraries(video_record 
                      /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so 
                      /opt/ros/kinetic/lib/libroscpp_serialization.so 
                      /opt/ros/kinetic/lib/libroscpp.so 
                      /opt/ros/kinetic/lib/librosconsole.so 
                      /opt/ros/kinetic/lib/libcpp_common.so 
                      /opt/ros/kinetic/lib/librostime.so 
                      rt 
                      ${Boost_LIBRARIES} 
                      ${catkin_LIBRARIES})
