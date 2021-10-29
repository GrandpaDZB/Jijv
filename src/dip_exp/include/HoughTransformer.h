#ifndef HOUGH_TRANSFORMER
#define HOUGH_TRANSFORMER

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

using namespace std;
using namespace cv;

// ====================================================== parameters function ==============================================================
// using x,y and params except the last dim to compute the value of last dim
float param_func_line(float x, float y, float* params){
    return x*cos(params[0]) + y*sin(params[0]);
    
}

float param_func_circle(float x, float y, float* params){
    return sqrt((x-params[0])*(x-params[0])+(y-params[1])*(y-params[1]));
}


class HoughTransformer{
public:

    void scan_on_param_space(int dim, vector<int*> points, float** scan_area, float (*param_func)(float x, float y, float* params) , int* quanti_rate);
    void vector_sort(vector<float>* r);
    bool equal_intArray(int* A, int* B, int compare_len);

    float** build_hyperBlock(int dim);
    void free_hyperBlock(float** block, int dim);
    int* default_quanti_rate(int dim);

    void add_point(vector<int*>* points, int x, int y);
    void free_points(vector<int*>* points);

    void Hough_Line_Detect(vector<int*> points, float max_scan_r, int* quanti_rate, int iteration, float* params);
    void Hough_Circle_Detect(vector<int*> points, float max_a, float max_b, float max_r, float min_r, int* quanti_rate, int iteration, float* params);

    vector<vector<int*>> get_edgeSubsets(cv::Mat src, int min_len = 100);
    vector<vector<int>> find_link_neighbors(cv::Mat p, int x, int y);

    void Hough_Line_Detect_on_Image(cv::Mat src, float max_scan_r, int* quanti_rate, int iteration, vector<float*>* params, int min_len = 100);
    void Hough_Circle_Detect_on_Image(cv::Mat src, float max_a, float max_b, float max_r, float min_r, int* quanti_rate, int iteration, vector<float*>* params, int min_len = 100);


    HoughTransformer(/* args */);
    ~HoughTransformer();
};

void HoughTransformer::add_point(vector<int*>* points, int x, int y){
    int* tmp = new int[2];
    tmp[0] = x;
    tmp[1] = y;
    points->push_back(tmp);
    return;
}

void HoughTransformer::free_points(vector<int*>* points){
    for(int i = 0; i < points->size(); i++){
        delete[] points->at(i);
    }
    points->clear();
    return;
}

// The paramSpace would be quantified to 10^dim subspaces for default 
int* HoughTransformer::default_quanti_rate(int dim){
    int* quanti_rate = new int[dim];
    for(int i = 0; i < dim; i ++){
        quanti_rate[i] = 9;
    }
    return quanti_rate;
}

bool HoughTransformer::equal_intArray(int* A, int* B, int compare_len){
    for(int i = 0; i < compare_len; i ++){
        if(A[i] == B[i]){
            continue;
        }else{
            return false;
        }
    }
    return true;
}

float** HoughTransformer::build_hyperBlock(int dim){
    float** block = new float*[dim];
    for(int i = 0; i < dim; i ++){
        block[i] = new float[2];
    }
    return block;
}

void HoughTransformer::free_hyperBlock(float** block, int dim){
    for(int i = 0; i < dim; i ++){
        delete[] block[i];
    }
    delete[] block;
    return;
}

void HoughTransformer::vector_sort(vector<float>* r){
    int min = -1;
    for(int i = 0; i < r->size()-1; i++){
        min = i;
        for(int j = i+1; j < r->size(); j ++){
            if(r->at(j) < r->at(min)){
                min = j;
            }
        }
        if( i != min){
            float tmp = r->at(i);
            r->at(i) = r->at(min);
            r->at(min) = tmp;
        }
    }
    return;
}



/**
 * @brief: Scan the parameter hyperblock and vote for the most fit curve parameters.
 * @param {int} dim, the dimension of parameter space
 * @param {vector<int*>} points, 2D points detected on image
 * @param {float**} scan_area, like [[start1, end1], ..., [startn, endn]] where n is the dim
 * @param {HoughTransformer::func} func, it's the parameter equation of the shape you hope to detect. In this function, you need to use x,y,parameters except the last dim param to compute the last dim one.
 * @param {int*} quanti_rate, the number of subset you wish to divide the parameter space into. You may use HoughTransformer::default_quanti_rate to obtain
 * a default one, which is set to be 20 for each dim.
 * @return {float**} A smaller scan_area where the best params most likely stay on.
 */
void HoughTransformer::scan_on_param_space(int dim, vector<int*> points, float** scan_area, float (*param_func)(float x, float y, float* params) , int* quanti_rate){
    // build target_block
    float** target_block = this->build_hyperBlock(dim);
    int max_vote = 0;
    int* block = new int[dim];
    for(int i = 0; i < dim; i++){
        block[i] = 0;
    }

    // quantified space scanning
    while(!(this->equal_intArray(block, quanti_rate, dim-1))){
        // compute vote
        float* params = new float[dim];
        vector<float> last_dim_param;
        for(int i = 0; i < dim; i++){
            // choose the middle of the interval as the param
            float gamma = (((float)block[i]+0.5)/(quanti_rate[i]+1));
            params[i] = gamma*scan_area[i][1] + (1-gamma)*scan_area[i][0];
        }
        for(auto& point:points){
            last_dim_param.push_back((*param_func)(point[0], point[1], params));
        }
        this->vector_sort(&last_dim_param);
        // compare the max vote on last dim param
        int index = 0;
        for(int i = 0; i <= quanti_rate[dim-1]; i++){
            int vote = 0;
            float gamma1 = (float)i/(quanti_rate[dim-1]+1);
            float gamma2 = gamma1+(1.0/(quanti_rate[dim-1]+1));
            while(index < last_dim_param.size() && last_dim_param[index] < gamma1*scan_area[dim-1][1] + (1-gamma1)*scan_area[dim-1][0]){
                index++;
            }
            while(index < last_dim_param.size() && last_dim_param[index] > gamma1*scan_area[dim-1][1] + (1-gamma1)*scan_area[dim-1][0] && last_dim_param[index] < gamma2*scan_area[dim-1][1] + (1-gamma2)*scan_area[dim-1][0]){
                vote ++;
                index++;
            }
            if(vote > max_vote){
                max_vote = vote;
                for(int d = 0; d < dim-1; d ++){
                    float gamma3, gamma4;
                    if(block[d]-1>=0){
                        gamma3 = ((float)block[d]-1)/(quanti_rate[d]+1);
                    }else{
                        gamma3 = (float)block[d]/(quanti_rate[d]+1);
                    }
                    if(block[d]+2 <= quanti_rate[d]+1){
                        gamma4 = (((float)block[d]+2)/(quanti_rate[d]+1));
                    }else{
                        gamma4 = (((float)block[d]+1)/(quanti_rate[d]+1));
                    }
                    target_block[d][0] = gamma3*scan_area[d][1] + (1-gamma3)*scan_area[d][0];
                    target_block[d][1] = gamma4*scan_area[d][1] + (1-gamma4)*scan_area[d][0];
                }
                float unit_gamma = 1.0/(quanti_rate[dim-1]+1);
                if(i-1>=0){
                    gamma1 = (float)(i-1)/(quanti_rate[dim-1]+1);
                }else{
                    gamma1 = (float)i/(quanti_rate[dim-1]+1);
                }
                if(i+2<=quanti_rate[dim-1]+1){
                    gamma2 = (float)(i+2)/(quanti_rate[dim-1]+1);
                }else{
                    gamma2 = (float)(i+1)/(quanti_rate[dim-1]+1);
                }
                target_block[dim-1][0] = gamma1*scan_area[dim-1][1] + (1-gamma1)*scan_area[dim-1][0];
                target_block[dim-1][1] = gamma2*scan_area[dim-1][1] + (1-gamma2)*scan_area[dim-1][0];
            }
        }
        // update block
        block[0] += 1;
        for(int i = 0; i < dim; i++){
            if(block[i] > quanti_rate[i]){
                block[i] = 0;
                block[i+1] += 1;
            }
        }
    }
    for(int i = 0; i < dim; i++){
        scan_area[i][0] = target_block[i][0];
        scan_area[i][1] = target_block[i][1];
    }
    return;
}

void HoughTransformer::Hough_Line_Detect(vector<int*> points, float max_scan_r, int* quanti_rate, int iteration, float* params){
    
    float** scan_area = this->build_hyperBlock(2);
    scan_area[0][0] = 0;
    scan_area[0][1] = 3.1415926;
    scan_area[1][0] = 0;
    scan_area[1][1] = max_scan_r;

    float (*func)(float x, float y, float *param);
    func = &param_func_line;
    
    for(int iter = 0; iter < iteration; iter++){
        this->scan_on_param_space(2, points, scan_area, func, quanti_rate);
    }
    float theta = (scan_area[0][0]+scan_area[0][1])/2.0;
    float r = (scan_area[1][0]+scan_area[1][1])/2.0;
    params[0] = tan(3.1415926/2+theta);
    params[1] = r/sin(theta);
    return;
}

void HoughTransformer::Hough_Circle_Detect(vector<int*> points, float max_a, float max_b, float max_r, float min_r, int* quanti_rate, int iteration, float* params){
    
    float** scan_area = this->build_hyperBlock(3);
    scan_area[0][0] = 0;
    scan_area[0][1] = max_a;
    scan_area[1][0] = 0;
    scan_area[1][1] = max_b;
    scan_area[2][0] = min_r;
    scan_area[2][1] = max_r;

    float (*func)(float x, float y, float *param);
    func = &param_func_circle;
    
    for(int iter = 0; iter < iteration; iter++){
        this->scan_on_param_space(3, points, scan_area, func, quanti_rate);
    }
    params[0] = (scan_area[0][0]+scan_area[0][1])/2.0;
    params[1] = (scan_area[1][0]+scan_area[1][1])/2.0;
    params[2] = (scan_area[2][0]+scan_area[2][1])/2.0;
    return;
}

vector<vector<int>> HoughTransformer::find_link_neighbors(cv::Mat p, int x, int y){
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

vector<vector<int*>> HoughTransformer::get_edgeSubsets(cv::Mat src, int min_len){
    cv::Mat edge_img;
    src.copyTo(edge_img);
    vector<vector<int*>> edges;
    bool** visit = new bool*[edge_img.rows];
    for(int i = 0; i < edge_img.rows; i++){
        visit[i] = new bool[edge_img.cols];
        for(int j = 0; j < edge_img.rows; j++){
            visit[i][j] = false;
        }
    }

    for(int i = 0; i < edge_img.rows; i++){
        for(int j = 0; j < edge_img.cols; j++){
            if(visit[i][j] || edge_img.at<uint8_t>(i, j) != 255){ continue; }
            vector<int*> part;
            int* tmp = new int[2]; tmp[0] = i; tmp[1] = j;
            visit[i][j] = true;
            part.push_back(tmp);
            vector<vector<int>> neighbors = this->find_link_neighbors(edge_img, i, j);
            while(neighbors.size() != 0){
                bool is_valid = false;
                int a,b;
                for(auto& each:neighbors){
                    if(visit[each[0]][each[1]] == false){
                        is_valid = true;
                        a = each[0]; b = each[1];
                        break;
                    }
                }
                if(!is_valid){
                    break;
                }
                int* tmp = new int[2]; tmp[0] = a; tmp[1] = b;
                visit[tmp[0]][tmp[1]] = true;
                part.push_back(tmp);
                neighbors.clear();
                neighbors = this->find_link_neighbors(edge_img, tmp[0], tmp[1]);
            }
            if(part.size() > min_len){
                edges.push_back(part);
            }
        }
    }
    for(int i = 0; i < edge_img.rows; i++){
        delete[] visit[i];
    }
    delete[] visit;
    return edges;
}

void HoughTransformer::Hough_Line_Detect_on_Image(cv::Mat src, float max_scan_r, int* quanti_rate, int iteration, vector<float*>* params, int min_len){
    vector<vector<int*>> edges = this->get_edgeSubsets(src, min_len);
    for(int i = 0; i < edges.size(); i++){
        float* tmp_param = new float[2];
        this->Hough_Line_Detect(edges[i], max_scan_r, quanti_rate, iteration, tmp_param);
        params->push_back(tmp_param);
    }
    return;
}
void HoughTransformer::Hough_Circle_Detect_on_Image(cv::Mat src, float max_a, float max_b, float max_r, float min_r, int* quanti_rate, int iteration, vector<float*>* params, int min_len){
    vector<vector<int*>> edges = this->get_edgeSubsets(src, min_len);
    for(int i = 0; i < edges.size(); i++){
        float* tmp_param = new float[3];
        this->Hough_Circle_Detect(edges[i], max_a, max_b, max_r, min_r, quanti_rate, iteration, tmp_param);
        params->push_back(tmp_param);
    }
    return; 
}
    

HoughTransformer::HoughTransformer(/* args */){
}

HoughTransformer::~HoughTransformer()
{
}



#endif