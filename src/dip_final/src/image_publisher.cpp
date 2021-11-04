#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv){

    ros::init(argc, argv,"image_publisher_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_img = it.advertise("/dip_final/image", 1);

    cv::VideoCapture camera(0);
    cv::Mat src;
    camera.read(src);

    // computing shrink rate.
    float shrink_rate = 0;
    if(src.cols < src.rows){
        shrink_rate = 320.0/src.cols;
    }else{
        shrink_rate = 320.0/src.rows;
    }

    while(cv::waitKey(1) != (int)'q'){
        camera.read(src);
        cv::resize(src, src, cv::Size(),shrink_rate, shrink_rate);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        pub_img.publish(msg);
    }

    camera.release();
    return 0;
}