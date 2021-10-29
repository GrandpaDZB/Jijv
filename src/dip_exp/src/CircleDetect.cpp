
#include<CircleDetector.h>

int main(int argc, char** argv){

    CircleDetector Detector;

    // the minimum length of sample array that you would regard a curve to be the right edge of a shape
    int min_len = 40;
    // the tolerant radius when using cluster method
    float tolerant_radius = 50;
    // min & max radius of target circles
    float min_radius = 30;
    float max_radius = 400;
    // [not important] searching kernel size when tracking the edge
    int searching_kernel_size = 5;
    // [not important] learning rate
    float learning_rate = 100;
    // [not important] learing iteration
    int max_iteration = 150;

    cv::Mat src = imread("/home/grandpadzb/github/Jijv/src/dip_exp/6.jpg");


    // ================================================= main code ======================================================
    vector<vector<cv::Point>> PointSets;
    Detector.find_RegionPoints(src, &PointSets, min_len, searching_kernel_size);

    vector<vector<cv::Point3f>> centers;
    centers.push_back(vector<cv::Point3f>());
    for(auto& each:PointSets){
        cv::Point3f new_center = Detector.Adam_optimizer(each, learning_rate, max_iteration);
        if(new_center.z < min_radius || new_center.z > max_radius){ continue; }
        if(centers[0].size() == 0){
            centers[0].push_back(new_center);
        }else{
            bool is_finish = false;
            for(auto& each:centers){
                for(auto& center:each){
                    if(pow(center.x-new_center.x,2)+pow(center.y-new_center.y,2)+pow(center.z-new_center.z,2) <= tolerant_radius*tolerant_radius){
                        each.push_back(new_center);
                        is_finish = true;
                        break;
                    }
                }
                if(is_finish){
                    break;
                }
            }
            if(!is_finish){
                centers.push_back(vector<cv::Point3f>());
                centers[centers.size()-1].push_back(new_center);
            }
        }
    }
    vector<cv::Point3d> circles;
    for(auto& each:centers){
        if(each.size() > 1){
            int mean_x = 0; float num_x = 0;
            int mean_y = 0; float num_y = 0;
            int mean_z = 0; float num_z = 0;
            for(auto& center:each){
                mean_x = (1.0/(num_x+1))*center.x + (num_x/(num_x+1))*mean_x; num_x++;
                mean_y = (1.0/(num_y+1))*center.y + (num_y/(num_y+1))*mean_y; num_y++;
                mean_z = (1.0/(num_z+1))*center.z + (num_z/(num_z+1))*mean_z; num_z++;
            }
            cv::circle(src, cv::Point(mean_y,mean_x), mean_z, cv::Scalar(255), 1);
            cv::circle(src, cv::Point(mean_y,mean_x), 1, cv::Scalar(255), 1);
        }
    }
    imshow("center", src);
    
    waitKey(0);
    destroyAllWindows();

    return 0;
}