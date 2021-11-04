#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

float diff = 0;
void Callback_diff(const std_msgs::Float32ConstPtr& msg){
    diff = msg->data;
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "main_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_diff = nh.subscribe<std_msgs::Float32>("./dip_final/diff", 10, Callback_diff);


    return 0;
}