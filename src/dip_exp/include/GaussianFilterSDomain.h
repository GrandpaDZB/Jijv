#ifndef GAUSSIAN_FILTER_SDOMAIN
#define GAUSSIAN_FILTER_SDOMAIN

#include<iostream>
#include<opencv2/opencv.hpp>
#define pi 3.1415926

using namespace std;
using namespace cv;

class GaussianFilterSDomain
{
public:
    float sigma = 0.1;
    float kernel_size = 3;
    float* kernel = new float[3];
    cv::Mat run(cv::Mat src);

    void build_kernel();

    GaussianFilterSDomain(/* args */);
    ~GaussianFilterSDomain();
};

void GaussianFilterSDomain::build_kernel(){
    int index = 0;
    int side = (this->kernel_size-1)/2;
    float sum = 0;
    delete[] this->kernel;
    this->kernel = new float[int(this->kernel_size*this->kernel_size)]; 
    for(int i = -side; i < side + 1; i++){
        for(int j = -side; j < side + 1; j++){
            this->kernel[index] = (1/(2*pi*this->sigma))*exp(-(i*i + j*j)/(2*this->sigma*this->sigma));    
            sum += this->kernel[index];
            index ++;
        }
    }
    for(int i = 0; i < this->kernel_size*this->kernel_size ;i ++){
        this->kernel[i] /= sum;
    }
    return;
}

cv::Mat GaussianFilterSDomain::run(cv::Mat src){
    cv::Mat dst;
    src.copyTo(dst);
    int cols = src.cols;
    int rows = src.rows;
    int side = (this->kernel_size-1)/2;
    this->build_kernel();
    for(int i = 0; i < rows; i ++){
        for(int j = 0; j < cols; j ++){
            float new_gray = 0;
            int index = 0;
            for(int m = -side; m < side+1; m++){
                for(int n = -side; n < side+1; n++){
                    int new_x = i+m;
                    int new_y = j+n;
                    if(new_x < 0){
                        new_x += rows;
                    }else if (new_x > rows-1){
                        new_x -= rows;
                    }
                    if(new_y < 0){
                        new_y += cols;
                    }else if (new_y > cols-1){
                        new_y -= cols;
                    }
                    new_gray += this->kernel[index]*src.at<uint8_t>(new_x, new_y);
                    index ++;
                }    
            }
            dst.at<uint8_t>(i, j) = int(new_gray);
        }
    }
    return dst;
}

GaussianFilterSDomain::GaussianFilterSDomain(/* args */)
{
}

GaussianFilterSDomain::~GaussianFilterSDomain()
{
    delete[] this->kernel;
}




#endif