#ifndef CANNY_PROCESSOR
#define CANNY_PROCESSOR

#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef struct gradient_matrix{
    cv::Mat mag_matrix;
    cv::Mat theta_matrix;
}gradient_matrix;

class CannyProcessor{
public:
    cv::Mat src;

    cv::Mat preprocess(cv::Mat src, float resize_rate = 0.5);
    cv::Mat GaussianSmoothing(cv::Mat src, int kernel_size = 3, float sigma = 3.0);
    cv::Mat Conv(cv::Mat X, cv::Mat H);
    gradient_matrix SobelConv(cv::Mat src);

    CannyProcessor(/* args */);
    ~CannyProcessor();
};

cv::Mat CannyProcessor::Conv(cv::Mat X, cv::Mat H){
    cv::Mat dst;
    int side = (H.cols-1)/2;
    X.copyTo(dst);
    for(int i = 0; i < X.rows; i++){
        for(int j = 0; j < X.cols; j++){
            float new_gray = 0;
            for(int m = -side; m < side+1; m++){
                for(int n = -side; n < side+1; n++){
                    int new_i = i+m;
                    int new_j = j+n;
                    if(new_i < 0){
                        new_i += X.rows;
                    }else if(new_i > X.rows-1){
                        new_i -= X.rows;
                    }
                    if(new_j < 0){
                        new_j += X.cols;
                    }else if(new_j > X.cols-1){
                        new_j -= X.cols;
                    }
                    new_gray += H.at<uint8_t>(m+side, n+side)*X.at<uint8_t>(new_i, new_j);
                }
            }
            dst.at<uint8_t>(i, j) = (int)new_gray;
        }
    }
    return dst;
}

gradient_matrix CannyProcessor::SobelConv(cv::Mat src){
    cv::Mat theta_matrix, mag_matrix;
    theta_matrix.create(src.rows, src.cols, CV_32FC1);
    mag_matrix.create(src.rows, src.cols, CV_32FC1);
    float max_mag = 0;

    

}

cv::Mat CannyProcessor::preprocess(cv::Mat src, float resize_rate){
    cv::Mat dst;
    cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
    cv::resize(dst, dst, cv::Size(), resize_rate, resize_rate);
    return dst;
}

cv::Mat CannyProcessor::GaussianSmoothing(cv::Mat src, int kernel_size ,float sigma){
    cv::Mat dst;
    cv::GaussianBlur(src, dst, cv::Size(kernel_size,kernel_size), sigma, sigma);
    return dst;
}

CannyProcessor::CannyProcessor(/* args */)
{
}

CannyProcessor::~CannyProcessor()
{
}




#endif