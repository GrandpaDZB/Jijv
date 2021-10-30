#ifndef CANNY_PROCESSOR
#define CANNY_PROCESSOR

#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
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
    cv::Mat NMS(gradient_matrix GM);
    void DoubleThreshold(cv::Mat src, int th1, int th2, cv::Mat* ptr);
    int constrain(int num, int max);
    gradient_matrix SobelConv(cv::Mat src);
    vector<vector<int>> find_link_neighbors(cv::Mat p, int x, int y);
    cv::Mat Link(cv::Mat p1, cv::Mat p2, int index = 4);

    CannyProcessor(/* args */);
    ~CannyProcessor();
};

float angle2rad(float angle){
    return angle*3.1415926/180.0;
}

vector<vector<int>> CannyProcessor::find_link_neighbors(cv::Mat p, int x, int y){
    vector<vector<int>> neighbors;
    for(int i = -1; i < 2; i ++){
        for(int j = -1; j < 2; j ++){
            if(x+i < 0 || x+i > p.rows - 1 || y+j < 0 || y+j > p.cols - 1 || (i==0 && j ==0)){
                continue;
            }
            if(p.at<uint8_t>(x+i, y+j) == 255){
                vector<int> tmp;
                tmp.push_back(x+i);
                tmp.push_back(y+j);
                neighbors.push_back(tmp);
            }
        }
    }
    return neighbors;
}

void CannyProcessor::DoubleThreshold(cv::Mat src, int th1, int th2, cv::Mat* ptr){
    cv::threshold(src, ptr[0], th1, 255, cv::THRESH_BINARY);
    cv::threshold(src, ptr[1], th2, 255, cv::THRESH_BINARY);
    return;
}

cv::Mat CannyProcessor::Link(cv::Mat q1, cv::Mat q2, int index){
    cv::Mat p1,p2;
    q1.copyTo(p1);
    q2.copyTo(p2);
    
    bool** visit = new bool*[p1.rows];
    for(int i = 0; i < p1.rows; i++){
        visit[i] = new bool[p1.cols];
    }
    bool finish_check = false;
    // p2 = transform_c(index);

    while(!finish_check){
        for(int i = 0; i < p1.rows; i ++){
            for(int j = 0; j < p1.cols; j ++){

                // cout << " ====================== " <<endl;
                // cout << i << " " << j << endl;

                if(i == p1.rows-1 && j == p1.cols-1){ finish_check = true; break; }
                if(visit[i][j]){ continue; }
                else if( !visit[i][j] && p2.at<uint8_t>(i, j) == 255){
                    int x = i; int y = j;
                    bool end_flag = false;
                    bool valid_neighbor = false;
                    while(!end_flag){

                        // cout << x << " " << y << endl;

                        vector<vector<int>> neighbors = this->find_link_neighbors(p2, x, y);
                        if(neighbors.size() == 0){
                            vector<vector<int>> p1_neighbors = this->find_link_neighbors(p1, x, y);
                            if(p1_neighbors.size() == 0){
                                end_flag = true;
                                break;
                            }
                            for(auto &each:p1_neighbors){
                                if(!visit[each[0]][each[1]]){
                                    x = each[0];
                                    y = each[1];
                                    visit[x][y] = true;
                                    p2.at<uint8_t>(x, y) = 255;
                                    valid_neighbor = true;
                                    break;
                                }
                            }
                            if(valid_neighbor){ valid_neighbor = false; }
                            else{
                                end_flag = true;
                                break;
                            }
                            break;
                        }
                        for(auto &each:neighbors){

                            // cout << "neighbor: " << each[0] << " " << each[1] << endl;

                            if(!visit[each[0]][each[1]]){
                                x = each[0];
                                y = each[1];
                                visit[x][y] = true;
                                valid_neighbor = true;
                                break;
                            }
                        }
                        if(valid_neighbor){ valid_neighbor = false; }
                        else{
                            vector<vector<int>> p1_neighbors = this->find_link_neighbors(p1, x, y);
                            if(p1_neighbors.size() == 0){
                                end_flag = true;
                            }
                            for(auto &each:p1_neighbors){
                                if(!visit[each[0]][each[1]]){
                                    x = each[0];
                                    y = each[1];
                                    visit[x][y] = true;
                                    p2.at<uint8_t>(x, y) = 255;
                                    valid_neighbor = true;
                                    break;
                                }
                            }
                            if(valid_neighbor){ valid_neighbor = false; }
                            else{
                                end_flag = true;
                                break;
                            }
                        }
                    }

                }
            }
        }
    }
    return p2;
}

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
                    new_gray += H.at<float>(m+side, n+side)*X.at<uint8_t>(new_i, new_j);
                }
            }
            dst.at<uint8_t>(i, j) = (int)abs(new_gray);
        }
    }
    return dst;
}

int CannyProcessor::constrain(int num, int max){
    if(num < 0){
        num += max;
    }else if(num > max - 1){
        num -= max;
    }
    return num;
}

cv::Mat CannyProcessor::NMS(gradient_matrix GM){
    cv::Mat dst;
    GM.mag_matrix.copyTo(dst);

    for(int i = 0; i < dst.rows; i ++){
        for(int j = 0; j < dst.cols; j ++){
            int m=0,n=0;
            float theta = (float)GM.theta_matrix.at<float>(i, j);
            if( theta > angle2rad(-22.5) && theta < angle2rad(22.5)){
                m = 0; n = 1;
            }else if(theta > angle2rad(22.5) && theta < angle2rad(67.5)){
                m = -1; n = 1;
            }else if(theta > angle2rad(67.5) && theta < angle2rad(112.5)){
                m = -1; n = 0;
            }else if(theta > angle2rad(112.5) && theta < angle2rad(112.5+45)){
                m = -1; n = -1;
            }else if(theta > angle2rad(112.5+45) || theta < angle2rad(-112.5-45)){
                m = 0; n = -1;
            }else if(theta > angle2rad(-112.5-45) && theta < angle2rad(-112.5)){
                m = 1; n = -1;
            }else if(theta > angle2rad(-112.5) && theta < angle2rad(-67.5)){
                m = 1; n = 0;
            }else if(theta > angle2rad(-67.5) && theta < angle2rad(-22.5)){
                m = 1; n = 1;
            }
            int new_x_p = this->constrain(i + m, dst.rows);
            int new_y_p = this->constrain(j + n, dst.cols);
            int new_x_n = this->constrain(i - m, dst.rows);
            int new_y_n = this->constrain(j - n, dst.cols);
            
            if(dst.at<uint8_t>(i, j) >= dst.at<uint8_t>(new_x_p, new_y_p) && dst.at<uint8_t>(i, j) >= dst.at<uint8_t>(new_x_n, new_y_n)){}
            else{
                dst.at<uint8_t>(i, j) = 0;
            }
        }
    }
    return dst;
}

gradient_matrix CannyProcessor::SobelConv(cv::Mat src){
    gradient_matrix result;
    cv::Mat theta_matrix, mag_matrix;
    theta_matrix.create(src.rows, src.cols, CV_32FC1);
    mag_matrix.create(src.rows, src.cols, CV_8UC1);
    float max_mag = 0;

    cv::Mat SobelKernel_h(3,3, CV_32FC1);
    cv::Mat SobelKernel_v(3,3, CV_32FC1);

    /* 
    With a std Sobel kernel, as you can imagine, the gray level of abs_G would be in [0, 1020]
    To make sure that the gray level of Mag_matrix could be constrained in [0, 255]
    We have to use Sobel kernel/8
     */
    SobelKernel_h = (cv::Mat_<float>(3,3) << -1/8., 0, 1/8., -2/8., 0, 2/8., -1/8., 0, 1/8.);
    SobelKernel_v = (cv::Mat_<float>(3,3) << -1/8., -2/8., -1/8., 0, 0, 0, 1/8., 2/8., 1/8.);

    cv::Mat abs_Gx = this->Conv(src, SobelKernel_h);
    cv::Mat abs_Gy = this->Conv(src, SobelKernel_v);

    for(int i = 0; i < src.rows; i++){
        for(int j = 0; j < src.cols; j++){
            float y = (float)abs_Gy.at<uint8_t>(i, j);
            float x = (float)abs_Gx.at<uint8_t>(i, j);

            mag_matrix.at<uint8_t>(i, j) = x+y;
            if(x+y > max_mag){
                max_mag = x+y;
            }
            if(x >= 0.01){
                theta_matrix.at<float>(i, j) = atan(y/x);
            }else{
                theta_matrix.at<float>(i, j) = 3.1415926/2;
            }
        }
    }
    for(int i = 0; i < src.rows; i++){
        for(int j = 0; j < src.cols; j++){
            mag_matrix.at<uint8_t>(i, j) = (int)(mag_matrix.at<uint8_t>(i, j)*255/max_mag);
        }
    }
    result.mag_matrix = mag_matrix;
    result.theta_matrix = theta_matrix;
    return result;
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