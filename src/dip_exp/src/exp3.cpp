#include <CannyProcessor.h>
#include <opencv2/opencv.hpp>
#include <HoughTransformer.h>

using namespace cv;

int main(int argc, char** argv){

    CannyProcessor Processor;

    // cv::Mat src = cv::imread("/home/grandpadzb/github/Jijv/src/dip_exp/2.jpg");
    // imshow("origin", src);

    // src = Processor.preprocess(src, 0.8);

    // src = Processor.GaussianSmoothing(src, 3, 1.0);

    // gradient_matrix GradMatrix;

    // GradMatrix = Processor.SobelConv(src);

    // src = Processor.NMS(GradMatrix);

    // cv::Mat* dsrc = new cv::Mat[2];
    
    // Processor.DoubleThreshold(src, 20, 50, dsrc);

    // src = Processor.Link(dsrc[0], dsrc[1]);

    // imshow("0", src);
    // imshow("1", dsrc[0]);
    // imshow("2", dsrc[1]);
    


    // cv::waitKey(0);

    // cv::destroyAllWindows();

    VideoCapture camera(0);
    cv::Mat src;
    while(waitKey(1) != (int)'q'){

        // cv::Mat src = imread("/home/grandpadzb/github/Jijv/src/dip_exp/4.jpg");
        camera.read(src);
        cv::Mat edge_img;
        cv::resize(src, src, cv::Size(), 0.5, 0.5);
        cv::Canny(src, edge_img ,80, 200);
        

        vector<float*> params;
        HoughTransformer Transformer;
        int* quanti_rate = Transformer.default_quanti_rate(2);

        // Transformer.Hough_Circle_Detect_on_Image(edge_img, 200,200, 150, 50, quanti_rate, 4, &params, 160);
        Transformer.Hough_Line_Detect_on_Image(edge_img, 300, quanti_rate, 3, &params, 140);

        cv::Mat plot;
        src.copyTo(plot);
        for(auto& each:params){
            cv::line(plot, cv::Point((int)each[1], 0), cv::Point((int)(each[0]*plot.rows+each[1]), (int)(plot.rows)), cv::Scalar(255,255,255), 1);
            //cv::circle(plot, cv::Point((int)each[1], (int)each[0]), (int)each[2], cv::Scalar(255,255,255), 1);
        }
        imshow("src", src);
        imshow("canny", edge_img);
        imshow("plot", plot);

    }
    destroyAllWindows();
    
    return 0;
}