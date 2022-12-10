#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void gaussianSmoothing1()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");
  
    // create filter kernel
    float gauss_data[25] = {1, 4, 7, 4, 1,
                            4, 16, 26, 16, 4,
                            7, 26, 41, 26, 7,
                            4, 16, 26, 16, 4,
                            1, 4, 7, 4, 1};
    cv::Mat kernel = cv::Mat(5, 5, CV_32F, gauss_data);

    double sum_element_kernel {0};

    // Divide each element of the kernel by the sum of all the values in the kernel.
    for(int i=0; i<kernel.rows; ++i)
    {
      const float* kernel_i = kernel.ptr<float>(i);
      for(int j=0; j<kernel.cols; ++j)
      {
        sum_element_kernel += kernel_i[j];
      }
    }

    for(int i=0; i<kernel.rows; ++i)
    {
      float* kernel_i = kernel.ptr<float>(i);
      for(int j=0; j<kernel.cols; ++j)
      {
        kernel_i[j] /= sum_element_kernel;
      }
    }
  
    // apply filter
    cv::Mat result;
    cv::filter2D(img, result, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // show result
    std::string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, result);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    gaussianSmoothing1();
}