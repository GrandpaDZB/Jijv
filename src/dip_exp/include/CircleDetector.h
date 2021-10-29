#ifndef CIRCLE_DETECTOR
#define CIRCLE_DETECTOR

#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <math.h>
using namespace std;
using namespace cv;
using namespace Eigen;

class CircleDetector
{
public:

    void find_RegionPoints(cv::Mat src, vector<vector<cv::Point>>* PointSets, int min_len = 10, int SearchSize = 5);
    vector<int> find_link_neighbors(cv::Mat* p, int x, int y, bool** visit, int kernel_size = 5);

    cv::Point3f Adam_optimizer(vector<cv::Point> PointSet, float learning_rate, int max_iteration = 200);

    CircleDetector(/* args */);
    ~CircleDetector();
};



void CircleDetector::find_RegionPoints(cv::Mat src, vector<vector<cv::Point>>* PointSets, int min_len, int SearchSize){
    cv::Mat dst;
    src.copyTo(dst);

    // preprocess
    cv::cvtColor(dst,dst, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(dst,dst, cv::Size(5,5), 3.0, 3.0);
    // cv::equalizeHist(dst, dst);

    cv::threshold(dst, dst, 220, 255, cv::THRESH_BINARY);

    // canny
    cv::Mat edge_img;
    cv::Canny(dst, edge_img, 160, 200, 3);

    imshow("canny", edge_img);
    waitKey(0);


    bool** visit = new bool*[edge_img.rows];
    for(int i = 0; i < edge_img.rows; i++){
        visit[i] = new bool[edge_img.cols];
        for(int j = 0; j < edge_img.cols; j++){
            visit[i][j] = false;
        }
    }

    for(int i = 0; i < edge_img.rows; i++){
        for(int j = 0; j < edge_img.cols; j++){
            if(visit[i][j] == true){ continue; }
            if(edge_img.at<uint8_t>(i,j) == 0){ visit[i][j] = true; continue;}
            vector<cv::Point> PointSet;
            PointSet.clear();
            PointSet.push_back(cv::Point(i,j));
            vector<int> neighbor;
            neighbor.push_back(i);
            neighbor.push_back(j);
            neighbor = this->find_link_neighbors(&edge_img, neighbor[0], neighbor[1], visit, SearchSize);
            while(neighbor.size() != 0){
                PointSet.push_back(cv::Point(neighbor[0], neighbor[1]));
                neighbor = this->find_link_neighbors(&edge_img, neighbor[0], neighbor[1], visit, SearchSize);
            }
            if(PointSet.size() > min_len){
                PointSets->push_back(PointSet);
            }
        }
    }
    for(int i = 0; i < edge_img.rows; i++){
        delete[] visit[i];
    }
    delete[] visit;
    return;
}

vector<int> CircleDetector::find_link_neighbors(cv::Mat* p, int x, int y, bool** visit, int kernel_size){
    vector<int> neighbor;
    int side = (kernel_size-1)/2;
    for(int i = -side; i < side+1; i ++){
        for(int j = -side; j < side+1; j ++){
            if(x+i < 0 || x+i > p->rows - 1 || y+j < 0 || y+j > p->cols - 1 || (i==0 && j ==0)){
                continue;
            }
            if(p->at<uint8_t>(x+i, y+j) == 255 && visit[x+i][y+j] == false){
                neighbor.push_back(x+i);
                neighbor.push_back(y+j);
                visit[x+i][y+j] = true;
                return neighbor;
            }else{
                visit[x+i][y+j] = true;
            }
        }
    }
    return neighbor;
}

cv::Point3f CircleDetector::Adam_optimizer(vector<cv::Point> PointSet, float learning_rate, int max_iteration){
    int length = PointSet.size();
    Vector2f theta(0, 0);
    float beta1 = 0.9;
    float beta2 = 0.999;
    Vector2f s(0, 0);
    Vector2f r(0, 0);
    Vector2f s_hat(0, 0);
    Vector2f r_hat(0, 0);
    VectorXf x(length);
    VectorXf y(length);
    VectorXf d(length);
    Vector2f grad(0,0);
    Vector2f grad_square(0,0);
    // initialize
    for(int i = 0; i < length; i++){
        x(i) = PointSet[i].x;
        y(i) = PointSet[i].y;
    }
    float mean_x = x.mean();
    float mean_y = y.mean();
    float mean_d = 0;
    // iteration
    for(int iter = 1; iter < max_iteration+1; iter++){
        for(int i = 0; i < length; i++){
            d(i) = (x(i)-theta(0))*(x(i)-theta(0)) + (y(i)-theta(1))*(y(i)-theta(1));
        }
        mean_d = d.mean();
        // gradient
        grad(0) = 0; grad(1) = 0;
        for(int i = 0; i < length; i++){
            grad(0) += (-mean_d+d(i))*(mean_x-x(i))/10e3;
            grad(1) += (-mean_d+d(i))*(mean_y-y(i))/10e3;
            grad_square(0) = grad(0)*grad(0);
            grad_square(1) = grad(1)*grad(1);
        }
        // biasd moment estimate
        s = beta1*s + (1-beta1)*grad;
        r = beta2*r + (1-beta2)*grad_square;
        // correct bias
        s_hat = s/(1-pow(beta1,iter));
        r_hat = r/(1-pow(beta2,iter));
        // update
        theta(0) -= learning_rate*s_hat(0)/(std::sqrt(r_hat(0))+10e-8);
        theta(1) -= learning_rate*s_hat(1)/(std::sqrt(r_hat(1))+10e-8);
    }
    float estimate_radius = std::sqrt(mean_d);
    
    return cv::Point3f(theta(0), theta(1), estimate_radius);
}


CircleDetector::CircleDetector(/* args */)
{
}

CircleDetector::~CircleDetector()
{
}







#endif