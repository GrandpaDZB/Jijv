#ifndef _HISTAVE_
#define _HISTAVE_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
using namespace std;
using namespace cv;


class HistAve{
public:

    bool show_histogram = true;
    int hist_before[256] = {0};
    int hist_after[256]= {0};
    
    cv::Mat src;
    cv::Mat src_old;

    void process();
    void clear_hist();
    void draw_histo(cv::Mat* src, int* hist);
    void import_img(cv::Mat src);
};

void HistAve::import_img(cv::Mat src){
    src.copyTo(this->src);
    src.copyTo(this->src_old);
};

void HistAve::draw_histo(cv::Mat* src, int* hist){
    int cols = src->cols;
    int rows = src->rows;
    cv::Point2d start_point((int)(cols*0.1), (int)(rows*0.9));
    cv::putText(*src, "0", start_point, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255), 2);
    int interval = (int)(0.8*cols/255.0);
    if(interval < 1){
        ROS_ERROR("Histogram drawing interval smaller than 1! ");
        return;
    }
    for(int i = 0; i < 256; i ++){
        start_point.x += interval;
        cv::Point2d end_point(start_point.x, start_point.y-(int)(3.0*rows*((float)hist[i]/(cols*rows))));
        cv::line(*src, start_point, end_point, cv::Scalar(255), 1);
    }
    cv::putText(*src, "255", start_point, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255), 2);
    return;
}

void HistAve::clear_hist(){
    for(int i = 0; i<256; i++){
        this->hist_after[i] = 0;
        this->hist_before[i] = 0;
    }
    return;
}

void HistAve::process(){
    // build histogram before processing
    int cols = this->src.cols;
    int rows = this->src.rows;
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            int gray_level = (int)this->src.at<uchar>(i,j);
            this->hist_before[gray_level] ++;
        }
    }
    // compute accumulated histogram
    float acc_histo[256] = {0};
    acc_histo[0] = this->hist_before[0];
    for(int i = 1; i < 256; i ++){
        acc_histo[i] = acc_histo[i-1] + this->hist_before[i];
    }
    // process image & build histogram after processing
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            int gray_level = (int)this->src.at<uchar>(i,j);
            int new_gray_level = (int)((acc_histo[gray_level]/(cols*rows))*255);
            this->src.at<uchar>(i,j) = new_gray_level;
            this->hist_after[new_gray_level] ++;
        }
    }

    if(this->show_histogram){
        this->draw_histo(&this->src, this->hist_after);
        this->draw_histo(&this->src_old, this->hist_before);
    }

    this->clear_hist();
    return;
}










#endif 