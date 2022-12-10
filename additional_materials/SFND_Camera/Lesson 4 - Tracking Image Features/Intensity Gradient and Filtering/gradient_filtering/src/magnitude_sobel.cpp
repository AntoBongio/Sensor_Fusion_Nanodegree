#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <math.h>


std::string type2str(int type) {

    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    cv::Mat filtered_img;
    int filter_size = 5;
    int std_dev_x = 2;
    int sd_dev_y = 2;
    cv::GaussianBlur(imgGray, filtered_img, cv::Size(filter_size, filter_size), std_dev_x, sd_dev_y);

    // create filter kernels using the cv::Mat datatype both for x and y
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    float sobel_y[9] = {-1, -2, -1,
                         0,  0,  0, 
                        +1, +2, +1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

    // apply filter using the OpenCv function filter2D()
    cv::Mat result_x;
    cv::filter2D(filtered_img, result_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::Mat result_y;
    cv::filter2D(filtered_img, result_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // compute magnitude image based on the equation presented in the lesson 
    cv::Mat magnitude = imgGray.clone();

    std::cout << "Type: " << type2str(imgGray.type()) << std::endl;
    

    for(int i=0; i<magnitude.rows; ++i)
    {
        unsigned char* magnitude_i = magnitude.ptr<unsigned char>(i);
        const unsigned char* result_x_i = result_x.ptr<unsigned char>(i);
        const unsigned char* result_y_i = result_y.ptr<unsigned char>(i);

        for(int j=0; j<magnitude.cols; ++j)
        {
            magnitude_i[j] = std::sqrt(result_x_i[j]*result_x_i[j] + result_y_i[j]*result_y_i[j]);
        }
    }
    
    // // Other way
    // for (int r = 0; r < magnitude.rows; r++)
    // {
    //     for (int c = 0; c < magnitude.cols; c++)
    //     {
    //         magnitude.at<unsigned char>(r, c) = sqrt(pow(result_x.at<unsigned char>(r, c), 2) +
    //                                                  pow(result_y.at<unsigned char>(r, c), 2));
    //     }
    // }


    // show result
    std::string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, filtered_img);
    cv::waitKey(0); // wait for keyboard input before continuing

    windowName = "Magnitude";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}