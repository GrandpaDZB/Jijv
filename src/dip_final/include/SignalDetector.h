#ifndef SIGNAL_DETECTOR
#define SIGNAL_DETECTOR

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;


class SignalDetector
{
public:
    cv::Mat std_figure;

    bool detect_signal(cv::Mat src, cv::Point3i* center);
    cv::Mat cut_down_signal(cv::Mat src, cv::Vec3f circle);
    SignalDetector(/* args */);
    ~SignalDetector();
};


bool SignalDetector::detect_signal(cv::Mat src, cv::Point3i* center){
    bool if_finded = false;
    // preprocess
    // cv::resize(src, src, cv::Size(), 0.6,0.6);
    cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(src, src, cv::Size(3,3), 1.0, 1.0);
    // split S channel
    vector<Mat> channels;
    cv::split(src, channels);
    cv::Mat dst = channels[1];
    
    // hough circle finding signals
    vector<cv::Vec3f> params;
    cv::HoughCircles(dst, params, cv::HOUGH_GRADIENT, 1.5, 50, 110, 110, 10, 500);

    cv::Mat mark;
    // plot
    if(params.size() != 0){
        // cv::circle(dst, cv::Point((int)params[0][0],(int)params[0][1]), (int)params[0][2], cv::Scalar(255), 2);
        mark = this->cut_down_signal(dst, params[0]);
        cv::resize(mark, mark, cv::Size(320,320));
        cv::threshold(mark, mark, 70, 255, cv::THRESH_BINARY);

        // Hu moments
        cv::Moments moments = cv::moments(mark, false);
        double hu_moments[7];
        cv::HuMoments(moments, hu_moments);

        for(int k = 0; k < 7; k++){
            if( hu_moments[k] >= 0){
                hu_moments[k] = std::log(hu_moments[k]);
            }else{
                hu_moments[k] = -std::log(-hu_moments[k]);
            }
        }
        double diff1 = cv::matchShapes(this->std_figure, mark, cv::CONTOURS_MATCH_I1, 0); // <= 0.009
        double diff2 = cv::matchShapes(this->std_figure, mark, cv::CONTOURS_MATCH_I2, 0); // <= 0.05
        // double diff3 = cv::matchShapes(this->std_figure, mark, cv::CONTOURS_MATCH_I3, 0); 
        if(diff1 <= 0.007 && diff2 <= 0.05){
            if_finded = true;
        }
        center->x = (int)params[0][1];
        center->y = (int)params[0][0];
        center->z = (int)params[0][2];
    }
    // cv::imshow("test", dst);
    
    // cv::waitKey(1);
    return if_finded;
}

cv::Mat SignalDetector::cut_down_signal(cv::Mat src, cv::Vec3f circle){
    int R = (int)circle[2];
    int a = (int)circle[1], b = (int)circle[0];

    cv::Mat sub_img(2*R+1, 2*R+1, CV_8UC1);

    for(int i = 0; i < 2*R+1; i++){
        for(int j = 0; j < 2*R+1; j++){
            if((i-R)*(i-R)+(j-R)*(j-R) <= R*R){
                sub_img.at<uint8_t>(i, j) = src.at<uint8_t>(a+i-R, b+j-R);
            }else{
                sub_img.at<uint8_t>(i, j) = 0;
            }
        }
    }
    return sub_img;
}

SignalDetector::SignalDetector(/* args */){
    this->std_figure = cv::imread("/home/grandpadzb/github/Jijv/src/dip_final/assets/std_figure.jpg");
    cv::cvtColor(this->std_figure, this->std_figure, cv::COLOR_BGR2GRAY);
    cv::threshold(this->std_figure, this->std_figure, 120, 255, cv::THRESH_BINARY);
}

SignalDetector::~SignalDetector()
{
}


#endif