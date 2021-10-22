#ifndef MORPHOLOGICAL_PROCESSOR
#define MORPHOLOGICAL_PROCESSOR

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MorphologicalProcessor
{
public:
    cv::Mat src;

    int kernel_size = 3;
    bool* kernel = new bool[9];

    bool full3_kernel[9] = {1,1,1,1,1,1,1,1,1};
    bool cross3_kernel[9] = {0,1,0,1,1,1,0,1,0};

    bool full5_kernel[25] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

    int match_condition(int x, int y);

    cv::Mat dilation();
    cv::Mat erosion();

    void preprocess(float resize_rate = 1.0, int threshold_value = 125);
    MorphologicalProcessor(/* args */);
    ~MorphologicalProcessor();
};

cv::Mat MorphologicalProcessor::dilation(){
    cv::Mat dst;
    this->src.copyTo(dst);
    for(int i = 0; i < this->src.rows; i ++){
        for(int j = 0; j < this->src.cols; j ++){
            if(this->match_condition(i, j) != 0){
                dst.at<uint8_t>(i, j) = 255;
            }
        }
    }
    return dst;
}

cv::Mat MorphologicalProcessor::erosion(){
    cv::Mat dst;
    this->src.copyTo(dst);
    for(int i = 0; i < this->src.rows; i ++){
        for(int j = 0; j < this->src.cols; j ++){
            if(this->match_condition(i, j) != 2){
                dst.at<uint8_t>(i, j) = 0;
            }
        }
    }
    return dst;
}

// 0: miss  1:hit   2:fit
int MorphologicalProcessor::match_condition(int x, int y){
    int side = (this->kernel_size-1)/2;
    int hit_num = 0;
    int index = 0;
    for(int m = -side; m < side+1; m ++){
        for(int n = -side; n < side+1; n ++){
            int new_x = x+m;
            int new_y = y+n;
            if(new_x < 0){
                new_x += src.rows;
            }else if(new_x > src.rows-1){
                new_x -= src.rows;
            }
            if(new_y < 0){
                new_y += src.cols;
            }else if(new_y > src.cols-1){
                new_y -= src.cols;
            }
            if((int)src.at<uint8_t>(new_x, new_y) == 255 && this->kernel[index] == true){
                hit_num ++;
            }
            index ++;
        }
    }
    if(hit_num == 0){
        return 0;
    }else if(hit_num == this->kernel_size*this->kernel_size){
        return 2;
    }else{
        return 1;
    }
}

void MorphologicalProcessor::preprocess(float resize_rate, int threshold_value){
    cv::cvtColor(this->src, this->src, cv::COLOR_BGR2GRAY);
    cv::threshold(this->src, this->src, threshold_value, 255, cv::THRESH_BINARY);
    cv::resize(this->src, this->src, cv::Size(), resize_rate, resize_rate);
    return;
}

MorphologicalProcessor::MorphologicalProcessor(/* args */)
{
}

MorphologicalProcessor::~MorphologicalProcessor()
{
}



#endif