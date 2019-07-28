//============================================================================
// Name        : view_rosbag.cpp
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
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
// Open device
#include <fcntl.h>
#include <unistd.h>
// Video for Linux
#include <sys/ioctl.h> // Handling special device
#include <linux/videodev2.h>
// Open CV
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

using namespace std;  // Giving the ability to use 'string' and 'int' without declaring 'std::' prefix

void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& message, int device) {

	// Objects that are used in order to convert ROS to Image
	grid_map::GridMap gridMapObj;
	grid_map::GridMapRosConverter converterObj;
	cv_bridge::CvImage cvImageObj;
	cv::Mat dImg;

	if (message != NULL) {
		converterObj.fromMessage(*message, gridMapObj);
        converterObj.toCvImage(gridMapObj, message->layers[0], "mono8", cvImageObj);
        dImg = cvImageObj.image;
        if (write(device, (const char*)dImg.data,dImg.cols*dImg.rows*dImg.channels()) != dImg.cols*dImg.rows*dImg.channels()) {
        	std::cout << "failed to write to device\n";
        }
    } else {
        ROS_WARN("Unable to load data from ROS message.");
    }
}

// argc = The amount of arguments the program had received (execution of the program itself is an argument)
// argv = The value of the arguments that had been received (argv[0] = video_recorder)
int main(int argc, char** argv) {

    // Initiate video object
    int device;
    struct v4l2_format v4l2Obj;

    device = open("/dev/video10", O_RDWR);
    if (device == -1) {
    	perror("Couldn't open /dev/video10");
    	exit(EXIT_FAILURE);
    }
    ioctl(device, VIDIOC_G_FMT, &v4l2Obj); // G = get (update v4l2Obj with device properties)
    v4l2Obj.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    v4l2Obj.fmt.pix.width = 200;
    v4l2Obj.fmt.pix.height = 200;
    v4l2Obj.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY; /* 16  YUV 4:2:2     */
    v4l2Obj.fmt.pix.sizeimage = 200*200*8;
    ioctl(device, VIDIOC_S_FMT, &v4l2Obj); // S = set


    // Initiating ROS
    ros::init(argc, argv, "my_ros_node");
    ros::NodeHandle ros_node;

    // Listen for topic to be published
    //ros::Subscriber sub = ros_node.subscribe("/topic/gridMap", 1000, gridMapCallback);
    ros::Subscriber sub = ros_node.subscribe<grid_map_msgs::GridMap>("my_topic/grid_map", 10, boost::bind(gridMapCallback, _1, device));
    // Refresh subscriber
    ros::spin();

    return 0;
}
