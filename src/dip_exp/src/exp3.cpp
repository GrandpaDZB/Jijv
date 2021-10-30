#include <CannyProcessor.h>
#include <opencv2/opencv.hpp>
#include <HoughTransformer.h>

using namespace cv;

int main(int argc, char** argv){

    CannyProcessor Processor;
    cv::Mat src_house = imread("/home/grandpadzb/github/Jijv/src/dip_exp/3.jpg");
    cv::Mat src_candy = imread("/home/grandpadzb/github/Jijv/src/dip_exp/6.jpg");

    cv::Mat plot_src_house;
    cv::Mat plot_src_candy;
    src_house.copyTo(plot_src_house);
    src_candy.copyTo(plot_src_candy);
    imshow("origin_house", plot_src_house);
    imshow("origin_candy", plot_src_candy);

    
    // preprocessing & gaussian bluring
    src_house = Processor.preprocess(src_house, 0.8);
    src_candy = Processor.preprocess(src_candy, 0.8);
    src_house = Processor.GaussianSmoothing(src_house, 3, 1.0);
    src_candy = Processor.GaussianSmoothing(src_candy, 3, 1.0);

    // sobel kernel convolution, computing Gradient Matrix
    gradient_matrix GradMatrix_house;
    gradient_matrix GradMatrix_candy;
    GradMatrix_house = Processor.SobelConv(src_house);
    GradMatrix_candy = Processor.SobelConv(src_candy);

    // NMS
    src_house = Processor.NMS(GradMatrix_house);
    src_candy = Processor.NMS(GradMatrix_candy);

    // double threshold & linking
    cv::Mat* dsrc_house = new cv::Mat[2];
    cv::Mat* dsrc_candy = new cv::Mat[2];
    Processor.DoubleThreshold(src_house, 20, 40, dsrc_house);
    Processor.DoubleThreshold(src_candy, 20, 40, dsrc_candy);
    src_house = Processor.Link(dsrc_house[0], dsrc_house[1], 3);
    src_candy = Processor.Link(dsrc_candy[0], dsrc_candy[1], 6);

    // display canny result
    imshow("canny_house", src_house);
    imshow("canny_candy", src_candy);

    HoughTransformer Transformer;

    vector<float*> params_house;
    cv::Mat plot_house;
    plot_house = cv::imread("/home/grandpadzb/github/Jijv/src/dip_exp/3.jpg");
    cv::resize(plot_house, plot_house, cv::Size(), 0.8, 0.8);
    int* quanti_rate_house = Transformer.default_quanti_rate(2);
    // hough line transform, with max_scan_r = 300, max_iteration = 3
    Transformer.Hough_Line_Detect_on_Image(src_house, 300, quanti_rate_house, 3, &params_house, 160);

    for(auto& each:params_house){
        cv::Point p1, p2;
        p1.y = 0;
        p1.x = each[1];
        if(each[0] >= 0){
            p2.x = plot_house.cols;
            p2.y = (p2.x-each[1])/each[0];
        }else{
            p2.x = 0;
            p2.y = -each[1]/each[0];
        }
        cv::line(plot_house, p1, p2, cv::Scalar(255,255,255), 1);
    }

    vector<float*> params_candy;
    cv::Mat plot_candy;
    plot_candy = cv::imread("/home/grandpadzb/github/Jijv/src/dip_exp/6.jpg");
    cv::resize(plot_candy, plot_candy, cv::Size(), 0.8, 0.8);
    int* quanti_rate_candy = Transformer.default_quanti_rate(3);

    // hough line transform, with max_scan_a = 200, with max_scan_b = 200, range of radiusc=[50,150], max_iteration = 4
    Transformer.Hough_Circle_Detect_on_Image(src_candy, 200, 200, 150, 30, quanti_rate_candy, 3, &params_candy, 140);

    
    for(auto& each:params_candy){
        cv::circle(plot_candy, cv::Point((int)each[0], (int)each[1]), (int)each[2], cv::Scalar(255,255,255), 1);
    }

    imshow("hough_house", plot_house);
    imshow("hough_candy", plot_candy);

    cv::waitKey(0);
    destroyAllWindows();
    
    return 0;
}