//============================================================================
// Name        : record_video.cpp
// Author      : techyE
// Version     :
// Copyright   :
// Description :
//============================================================================

#include <iostream>

// '<>' used for "included directories"
// '""' used for specific location
// ROS
#include <ros/ros.h>
//#include <std_msgs/Float32MultiArray.h> // http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rosbag/bag.h>
// Open device
#include <fcntl.h>
#include <unistd.h>
// Video for Linux
#include <sys/ioctl.h> // Handling special device
#include <linux/videodev2.h>
// Open CV
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
// Env Variables
#include <stdlib.h>

using namespace std;  // Giving the ability to use 'string' and 'int' without declaring 'std::' prefix

void videoToGridmap(cv::VideoCapture videoDevice, grid_map_msgs::GridMap &message) { // Passing object as refrence
	cv::Mat frame;
    //grid_map::GridMap gridMapObj;
	grid_map::GridMap gridMapObj;
	//grid_map::setFrameId("map");
	gridMapObj.setGeometry(grid_map::Length(1, 1), 0.005, grid_map::Position(0.0, 0.0));
	videoDevice >> frame; // >> = getInput, get new frame from camera
	const std::string layer = "basic"; // layer of image on Gridmap Obj
	grid_map::GridMapCvConverter::addColorLayerFromImage<unsigned short, 1>(frame, layer, gridMapObj); // <> is a C++ template allowing  to change Type of variable used
	grid_map::GridMapRosConverter::toMessage(gridMapObj, message);
}

// argc = The amount of arguments the program had received (execution of the program itself is an argument)
// argv = The value of the arguments that had been received (argv[0] = video_recorder)
int main(int argc, char** argv) {

    // Initiating ROS
    ros::init(argc, argv, "my_ros_node2");
    ros::NodeHandle ros_node2;
    rosbag::Bag bag;
    char* envToRosbag = getenv("TO_ROSBAG");
    if (std::strcmp(envToRosbag,"TRUE")) { // Tests strcmp() function for better compare
    	bag.open("mono8_200x200.bag", rosbag::bagmode::Write);
    }

    // Setting up a publisher
    ros::Publisher pub = ros_node2.advertise<grid_map_msgs::GridMap>("my_topic/grid_map", 10);
    ros::Rate loop_rate(10); // 10 Mhz

    // Open video device
	cv::VideoCapture device("/dev/video0");
	if (!device.isOpened()){
    	perror("Couldn't open /dev/video0");
    	exit(EXIT_FAILURE);
	}

    grid_map_msgs::GridMap msg;
    std_msgs::Float32MultiArray img_val;
    while (ros::ok())
    {

    	videoToGridmap(device, msg);

    	pub.publish(msg);
        if (std::strcmp(envToRosbag,"TRUE")) {
        	bag.write("my_topic/grid_map", ros::Time::now(), msg);
        }
    	msg.data.pop_back(); // clear video buffer
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
