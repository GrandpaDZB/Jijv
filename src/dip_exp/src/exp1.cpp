#include <iostream>
#include <HistAve.h>
using namespace std;


int main(int argc, char** argv){
    cv::VideoCapture camera(0);

    cv::Mat src;
    HistAve model;
    while(cv::waitKey(1) != (int)'q'){
        camera.read(src);
        cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
        model.import_img(src);
        model.process();
        cv::Mat concat_img;
        cv::hconcat(model.src_old, model.src, concat_img);
        cv::imshow("contrast", concat_img);
    }
    camera.release();
    cv::destroyAllWindows();

    return 0;
}