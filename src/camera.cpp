#include "ros/ros.h"
#include <iostream>
#include <std_msgs/UInt8.h>
#include <cstdio>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "camera");
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);
    image_transport::Publisher image_pub = it.advertise("camera/image_raw",1);
    
    ros::Rate loop_rate(10);
    Mat frame;
    Mat image_resized;
    VideoCapture cap(0);

    while(!cap.isOpened() && ros::ok())
    {
        ROS_INFO("Cannot connect to camera......");
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        cap>>frame;
        resize(frame, image_resized, Size(640,360));
        ROS_INFO("%d  %d", image_resized.rows, image_resized.cols);
        sensor_msgs::ImagePtr msg;
        if(!frame.empty())
        {
            imshow("source",image_resized);
            waitKey(1);
            msg  =cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_resized).toImageMsg();
            image_pub.publish(msg);
        }

    	//printf("afterrosok\n");
    ros::spinOnce();	
    loop_rate.sleep();
    }

    return 0;
}