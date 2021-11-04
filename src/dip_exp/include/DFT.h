#ifndef DFT_H
#define DFT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

cv::Mat DFT(cv::Mat A){
    int M = A.rows;
    int N = A.cols;
    MatrixXcf U(M,M);
    MatrixXcf V(N,N);

    MatrixXcf f(M,N);
    MatrixXcf F(M,N);

    for(int i = 0; i < M; i++){
        for(int j = 0; j < M; j++){
            U(i, j) = Eigen::scomplex(cos(2*3.1415926*(i*j)/M), -sin(2*3.1415926*(i*j)/M));
        }
    }
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            V(i, j) = Eigen::scomplex(cos(2*3.1415926*(i*j)/N), -sin(2*3.1415926*(i*j)/N));
        }
    }

    for(int i = 0; i < M; i++){
        for(int j = 0; j < N; j++){
            f(i, j) = Eigen::scomplex(A.at<uint8_t>(j, i), 0);
        }
    }

    F = U*f*V;

    cv::Mat F_img(M, N, CV_8SC1);
    float max_norm = 0;
    for(int i = 0; i < M; i++){
        for(int j = 0; j < N; j++){
            Eigen::scomplex tmp = F(i, j);
            float norm = sqrt(tmp.real()*tmp.real()+tmp.imag()*tmp.imag());
            if(norm > max_norm){
                max_norm = norm;
            }
            F(i ,j) = norm;
        }
    }
    for(int i = 0; i < M; i++){
        for(int j = 0; j < N; j++){
            int m, n;
            if(i+(int)(M/2)<M){
                m = i+(int)(M/2);
            }else{
                m = i+(int)(M/2)-M;
            }
            if(j+(int)(N/2)<N){
                n = j+(int)(N/2);
            }else{
                n = j+(int)(N/2)-N;
            }
            F_img.at<uint8_t>(n ,m) = (F(i ,j)/max_norm).real()*255;
        }
    }
    return F_img;
}


#endif