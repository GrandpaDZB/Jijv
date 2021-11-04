#ifndef DIP_COMMON_INCLUDE
#define DIP_COMMON_INCLUDE

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;


cv::Mat BGR2HSV(cv::Mat src){
    cv::Mat dst;
    src.copyTo(dst);
    for(int i = 0; i < src.rows; i++){
        for(int j = 0; j < src.cols; j++){
            cv::Vec3b tmp_bgr = src.at<cv::Vec3b>(i, j);
            cv::Vec3f bgr;
            bgr[0] = tmp_bgr[0]/255.0; bgr[1] = tmp_bgr[1]/255.0; bgr[2] = tmp_bgr[2]/255.0;
            cv::Vec3b hsv;
            float max_bgr = 0, min_bgr = 1.0;
            for(int k = 0; k < 3; k++){
                if( bgr[k] > max_bgr ){
                    max_bgr = bgr[k];
                }
                if( bgr[k] < min_bgr ){
                    min_bgr = bgr[k];
                }
            }
            float delta = max_bgr - min_bgr;

            // H
            if(delta == 0){
                hsv[0] = 0;
            }else if(max_bgr == bgr[2]){
                hsv[0] = (int)(60*((bgr[1]-bgr[0])/delta+0)*255.0/360.0);
            }else if(max_bgr == bgr[1]){
                hsv[0] = (int)(60*((bgr[0]-bgr[2])/delta+2)*255.0/360.0);
            }else if(max_bgr == bgr[0]){
                hsv[0] = (int)(60*((bgr[2]-bgr[1])/delta+4)*255.0/360.0);
            }

            // S
            if(max_bgr == 0){
                hsv[1] = 0;
            }else{
                hsv[1] = (int)(delta/max_bgr*255.0);
            }

            // V
            hsv[2] = (int)(max_bgr*255.0);

            dst.at<cv::Vec3b>(i, j) = hsv;
        }
    }
    return dst;
}

cv::Mat HSV_binary(cv::Mat hsv_src, int threshold_values[6]){
    cv::Mat binary_hsv(hsv_src.rows, hsv_src.cols, CV_8UC1);
    for(int i = 0; i < hsv_src.rows; i++){
        for(int j = 0; j < hsv_src.cols; j++){
            cv::Vec3b level = hsv_src.at<cv::Vec3b>(i, j);
            if(
                level[0] > threshold_values[0] || level[0] < threshold_values[1]  ||
                level[1] > threshold_values[2] || level[1] < threshold_values[3]  ||
                level[2] > threshold_values[4] || level[2] < threshold_values[5]
            ){
                binary_hsv.at<uint8_t>(i, j) = 0;
            }else{
                binary_hsv.at<uint8_t>(i, j) = 255;
            }
        }
    }
    return binary_hsv;
}

namespace dip{

enum COLOR{
    RED,
    ORANGE,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    PURPLE
};

cv::Vec3b COLOR_RED(0,0,255);
cv::Vec3b COLOR_ORANGE(0,97,255);
cv::Vec3b COLOR_YELLOW(0,255,255);
cv::Vec3b COLOR_GREEN(0,255,0);
cv::Vec3b COLOR_CYAN(255,255,0);
cv::Vec3b COLOR_BLUE(255,0,0);
cv::Vec3b COLOR_PURPLE(255,0,139);


}

bool if_in_range(int num, int a, int b, float rate = 255/180.0){
    if(num <= b*rate && num >= a*rate){
        return true;
    }else{
        return false;
    }
}

int color_check(int h){
    if( if_in_range(h, 0, 10) || if_in_range(h, 155, 180) ){
        return dip::RED;
    }else if( if_in_range(h, 10, 25) ){
        return dip::ORANGE;
    }else if( if_in_range(h, 25, 34) ){
        return dip::YELLOW;
    }else if( if_in_range(h, 34, 77) ){
        return dip::GREEN;
    }else if( if_in_range(h, 77, 99) ){
        return dip::CYAN;
    }else if( if_in_range(h, 99, 124) ){
        return dip::BLUE;
    }else if( if_in_range(h, 124, 155)){
        return dip::PURPLE;
    }
    return -1;
}

int color_check_by_color_rate_counter(float color_rate_counter[7]){
    float max_rate = 0;
    int max_index = -1;
    for(int i = 0; i < 7; i++){
        if(color_rate_counter[i] > max_rate){
            max_rate = color_rate_counter[i];
            max_index = i;
        }
    }
    return max_index;
}

void accumulate_color_num(cv::Mat hsv_src, cv::Rect area ,int color_num_counter[7]){
    for(int i = 0; i < 7; i++){
        color_num_counter[i] = 0;
    }
    for(int i = area.y; i < area.y+area.height+1; i++){
        for(int j = area.x; j < area.x+area.width+1; j++){
            int h = hsv_src.at<cv::Vec3b>(i, j)[0];
            color_num_counter[color_check(h)] ++;
        }
    }
    return;
}


void colorNum2colorRate(int color_num_counter[7], float color_rate_counter[7]){
    int sum_num = 0;
    for(int i = 0; i < 7; i++){
        sum_num += color_num_counter[i];
    }
    for(int i = 0; i < 7; i++){
        color_rate_counter[i] = (float)color_num_counter[i]/sum_num;
    }
    return;
}

void accumulate_color_rate(cv::Mat hsv_src, cv::Rect area ,float color_rate_counter[7]){
    int color_num_counter[7] = {0};
    for(int i = area.y; i < area.y+area.height+1; i++){
        for(int j = area.x; j < area.x+area.width+1; j++){
            int h = hsv_src.at<cv::Vec3b>(i, j)[0];
            color_num_counter[color_check(h)] ++;
        }
    }
    colorNum2colorRate(color_num_counter, color_rate_counter);
    return;
}

cv::Mat color_statistic_figure(cv::Mat src, cv::Mat hsv_src, cv::Rect area, float color_rate_counter[7]){

    if(area.empty()){
        for(int i = 0; i < 7; i++){
            color_rate_counter[i] = 0;
        }
    }else{
        accumulate_color_rate(hsv_src, area, color_rate_counter);
    }

    cv::Mat statistic_figure(src.rows, 70, CV_8UC3);
    for(int i = 0; i < 7; i++){
        int max_height = (int)(color_rate_counter[i]*src.rows);
        for(int j = 0; j < 10; j++){
            for(int h = 0; h < src.rows; h++){
                if( h+1+max_height <= src.rows ){
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = cv::Vec3b(0,0,0);
                }else{
                switch (i)
                {
                case dip::RED:{
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = dip::COLOR_RED;
                    break;
                }
                case dip::ORANGE:{
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = dip::COLOR_ORANGE;
                    break;
                }
                case dip::YELLOW:{
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = dip::COLOR_YELLOW;
                    break;
                }
                case dip::GREEN:{
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = dip::COLOR_GREEN;
                    break;
                }
                case dip::CYAN:{
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = dip::COLOR_CYAN;
                    break;
                }
                case dip::BLUE:{
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = dip::COLOR_BLUE;
                    break;
                }
                case dip::PURPLE:{
                    statistic_figure.at<cv::Vec3b>(h, i*10+j) = dip::COLOR_PURPLE;
                    break;
                }
                }
                }
            }
            
        }
    }
    cv::Mat figure(src.rows, src.cols+70, CV_8UC3);
    vector<cv::Mat> input_figure;
    input_figure.push_back(src);
    input_figure.push_back(statistic_figure);
    cv::hconcat(input_figure, figure);

    return figure;
}


#endif