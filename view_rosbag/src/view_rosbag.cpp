//============================================================================
// Name        : view_rosbag.cpp
// Author      : techyE
// Version     :
// Copyright   :
// Description :
//============================================================================

#include <iostream>

// Can be found with '<>' after 'include_directories)
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
//#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;  // Giving the ability to use 'string' and 'int' without declaring 'std::' prefix
// argc = The amount of arguments the program had received (execution of the program itself is an argument)
// argv = The value of the arguments that had been received (argv[0] = video_recorder)
int main(int argc, char** argv) {

    // Initiating ROS
    ros::init(argc, argv, "my_ros_node");
    ros::NodeHandle ros_node;

    ros::Publisher video_pub = ros_node.advertise<grid_map_msgs::GridMap>("my_topic/video", 1000);


    ros::Rate loop_rate(10); // Loop at 10Hz => (100ms)
    while (ros::ok()) {
    }

    cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
    return 0;
}
