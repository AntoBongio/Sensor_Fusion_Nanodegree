#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>


// void detect_corners(cv::Mat img, double k, std::vector<cv::KeyPoint>& key_points)
// {
//     int sw_size = 7; // sliding window size
//     int sw_dist = sw_size / 2; // Distance from the center of the sliding window

//     for(int i=sw_dist; i>img.rows - sw_dist - 1; ++i)
//     {
//         for(int j=sw_dist; j<img.cols - sw_dist - 1; ++j)
//         {
//             float maximum_value{0};
//             // For inside the window
//             for(int i_inside=i - sw_dist; i_inside<i + sw_dist; i_inside++)
//             {
//                 for(int j_inside = j - sw_dist; j_inside<j + sw_dist; j_inside++)
//                 {
//                     float current_value = img.at<float>(i_inside, j_inside);
//                     maximum_value = maximum_value < current_value ? current_value : maximum_value;
//                 }
//             }

//             // check wether current pixel is local maximum
//             if (img.at<float>(i, j) == maximum_value)
//                 // key_points.push_back(cv::Point2f(i, j));
//                 key_points.push_back(cv::KeyPoint(cv::Point2f(i, j)));
//         }
//     }

// }

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    std::string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    // TODO: Perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    std::vector<cv::KeyPoint> key_points;
    double maxOverlap = 0.0;

    for(size_t i = 0; i < dst_norm.rows; ++i)
    {
        for(size_t j = 0; j < dst_norm.cols; ++j)
        {
            float response = dst_norm.at<float>(i ,j);
            if(response > minResponse)
            {
                cv::KeyPoint new_key_point;
                new_key_point.pt = cv::Point2f(j, i);
                new_key_point.size = 2 * apertureSize;
                new_key_point.response = response;

                bool overlap{false};

                for(auto it=key_points.begin(); it != key_points.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(new_key_point, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        overlap = true;
                        if (new_key_point.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = new_key_point; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!overlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    key_points.push_back(new_key_point); // store new keypoint in dynamic list
                }
            }

        }
        
    }

    // visualize keypoints
    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 5);
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, key_points, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, visImage);
    cv::waitKey(0);


}

int main()
{
    cornernessHarris();
}