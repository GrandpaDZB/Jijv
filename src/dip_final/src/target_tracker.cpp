#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <SignalDetector.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

std_msgs::Bool msg_usePersonTracker;
std_msgs::Float32 msg_diff;
cv::Mat src;

void Callback_image(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat tmp_src = cv_bridge::toCvShare(msg, "bgr8")->image;
    tmp_src.copyTo(src);
    return;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "target_traker_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("/dip_final/image", 1, Callback_image);
    ros::Publisher pub_use_PersonTracker = nh.advertise<std_msgs::Bool>("/dip_final/use_PersonTracker", 10);
    msg_usePersonTracker.data = false;
    ros::Publisher pub_diff = nh.advertise<std_msgs::Float32>("/dip_final/diff", 10);


    SignalDetector MomentsDetector;

    ros::Rate rate(50);
    cv::Point3i center;
    int counter = 0;

    while(ros::ok()){
        ros::spinOnce();
        if(!src.empty()){
             if(!MomentsDetector.detect_signal(src, &center)){
                 counter ++;
             }else{
                 counter = 0;
                 msg_usePersonTracker.data = false;
                 pub_use_PersonTracker.publish(msg_usePersonTracker);
             }
             if(counter > 5){
                 msg_usePersonTracker.data = true;
                 pub_use_PersonTracker.publish(msg_usePersonTracker);
             }

             if(!msg_usePersonTracker.data){
                 float diff = center.y - src.cols/2.0;
                 msg_diff.data = diff;
                 pub_diff.publish(msg_diff);
             }
        }
        rate.sleep();
    }

    destroyAllWindows();
    return 0;
}