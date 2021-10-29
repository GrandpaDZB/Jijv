#include <GaussianFilterSDomain.h>
#include <MorphologicalProcessor.h>


int main(int argc, char** argv){
    
    // 创建高斯滤波器和形态学操作器
    GaussianFilterSDomain GFilter;
    MorphologicalProcessor Processor;
    
    // 设置结构元素为3*3大小，值全是1的核
    delete[] Processor.kernel;
    Processor.kernel = Processor.full5_kernel;
    Processor.kernel_size = 5;
    
    // 设置高斯滤波器的核大小为3*3，标准差为5.0
    GFilter.kernel_size = 5;
    GFilter.sigma = 5.0;
    GFilter.build_kernel();

    cv::Mat src;
    cv::Mat src_Gaussian;
    cv::Mat src_dilation;
    cv::Mat src_erosion;

    VideoCapture camera(0);

    while (cv::waitKey(1) != (int)'q'){

        // 图片缩小
        camera.read(src);
        cv::resize(src, src, cv::Size(), 0.5, 0.5);
        imshow("src", src);
        
        // 转化为灰度图并进行高斯滤波
        cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
        src_Gaussian = GFilter.run(src);
        imshow("src_Gaussian", src_Gaussian);

        // 进行二值化，阈值为140
        cv::threshold(src, src, 140, 255, cv::THRESH_BINARY);
        imshow("src_Binary", src);

        Processor.src = src;

        // 进行膨胀和腐蚀操作
        src_dilation = Processor.dilation();
        src_erosion = Processor.erosion();

        imshow("src_dilation", src_dilation);
        imshow("src_erosion", src_erosion);
    }
    cv::destroyAllWindows();

    return 0;
}