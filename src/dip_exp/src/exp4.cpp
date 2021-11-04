#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <DFT.h>
#include <DIP_common_include.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace std;
using namespace cv;

cv::Mat hsv_src;
cv::VideoCapture camera(0);
int threshold_values[6] = {255, 0, 255, 95, 255, 120};

geometry_msgs::Twist msg_forward;
geometry_msgs::Twist msg_backward;
geometry_msgs::Twist msg_turnleft;
geometry_msgs::Twist msg_turnright;
geometry_msgs::Twist msg_stop;

void Callback_threshold(int, void*){
    return;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "exp4_node");
    ros::NodeHandle nh;

    ros::Publisher pub_velocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    msg_forward.linear.x = 0.1;
    msg_backward.linear.x = -0.1;
    msg_turnleft.angular.z = 0.1;
    msg_turnright.angular.z = -0.1;

    cv::namedWindow("HSV_threshold");
    
    cv::createTrackbar("H_High_threshold:", "HSV_threshold", &threshold_values[0], 255, Callback_threshold);
    cv::createTrackbar("H_Low_threshold:", "HSV_threshold", &threshold_values[1], 255, Callback_threshold);
    cv::createTrackbar("S_High_threshold:", "HSV_threshold", &threshold_values[2], 255, Callback_threshold);
    cv::createTrackbar("S_Low_threshold:", "HSV_threshold", &threshold_values[3], 255, Callback_threshold);
    cv::createTrackbar("V_High_threshold:", "HSV_threshold", &threshold_values[4], 255, Callback_threshold);
    cv::createTrackbar("V_Low_threshold:", "HSV_threshold", &threshold_values[5], 255, Callback_threshold);

    cv::Mat src;
    cv::Mat binary_hsv;
    while(waitKey(1) != (int)'q'){
        camera.read(src);
        cv::GaussianBlur(src, src, cv::Size(3,3), 1.0, 1.0);
        if(binary_hsv.empty()){
            binary_hsv = cv::Mat(src.rows, src.cols, CV_8UC1);
        }

        // HSV threshold
        hsv_src = BGR2HSV(src);
        binary_hsv = HSV_binary(hsv_src, threshold_values);
        // preprocessing
        cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
        cv::erode(binary_hsv, binary_hsv, morph_kernel);
        cv::dilate(binary_hsv, binary_hsv, morph_kernel);

        cv::Rect target_area = cv::boundingRect(binary_hsv);
        cv::rectangle(src, target_area, cv::Scalar(60, 120, 20), 2);

        // color statistics
        float color_rate_counter[7] = {0};
        cv::Mat statistic_figure = color_statistic_figure(src, hsv_src, target_area, color_rate_counter);

        // sending velocity command
        switch (color_check_by_color_rate_counter(color_rate_counter))
        {
        case dip::RED:{
            pub_velocity.publish(msg_forward);
            break;
        }
        case dip::GREEN:{
            pub_velocity.publish(msg_backward);
            break;
        }
        case dip::BLUE:{
            pub_velocity.publish(msg_turnleft);
            break;
        }
        case dip::YELLOW:{
            pub_velocity.publish(msg_turnright);
            break;
        }
        default:{
            pub_velocity.publish(msg_stop);
            break;
        }
        }

        imshow("Original", src);
        imshow("HSV_threshold", binary_hsv);
        imshow("color_statistic", statistic_figure);
    }    

    cv::destroyAllWindows();

    return 0;
}