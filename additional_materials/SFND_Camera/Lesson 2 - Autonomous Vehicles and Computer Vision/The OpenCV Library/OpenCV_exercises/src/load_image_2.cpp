#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


void loadImage2()
{
    for (int i = 5; i <= 9; i++)
    {

        // create file name
        std::ostringstream imgNumber;                           // #include <sstream>
        imgNumber << std::setfill('0') << std::setw(4) << i;    // #include <iomanip>

        std::cout << imgNumber.str() << std::endl;
        std::string filename = "../images/img" + imgNumber.str() + ".png";

        // load and display image
        cv::Mat img;
        img = cv::imread(filename);
        std::string windowName = "First steps in OpenCV";
        cv::namedWindow(windowName, 1); // create window
        cv::imshow(windowName, img);
        cv::waitKey(0); // wait for keyboard input before continuing
    }
}

int main()
{
    loadImage2();
    return 0;
}