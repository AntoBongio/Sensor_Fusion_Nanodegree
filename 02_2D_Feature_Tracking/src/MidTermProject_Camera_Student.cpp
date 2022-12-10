/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

#include <queue>

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    std::string dataPath = "../";

    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = true;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    // ! AKAZE descriptor can be used only with AKAZE keypoints 

    std::vector<std::string> detector_list{"FAST" };
    std::vector<std::string> descriptor_list{ "ORB" }; 

    for(const auto& detectorType: detector_list)
    {
        for(const auto& descriptorType: descriptor_list)
        {
            dataBuffer.clear();
            std::cout << std::endl;
            std::cout << "Detector / Descriptor pair: [" << detectorType << ", " << descriptorType << "]" << std::endl;
            int total_matches {0};
            bool display_print{true};

            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                /* LOAD IMAGE INTO BUFFER */
                if(display_print)
                    std::cout << std::endl;
                    std::cout << "Img --> " << imgIndex << std::endl;

                // assemble filenames for current index
                std::ostringstream imgNumber;
                imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
                std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                // TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;

                if(dataBuffer.size() >= dataBufferSize)
                {
                    dataBuffer.push_back(frame);    // Add an element at the end (following the newest)
                    dataBuffer.pop_front(); // Remove the oldest (the first element)
                }
                else
                {
                    dataBuffer.push_back(frame);
                }

                /* DETECT IMAGE KEYPOINTS */
                // extract 2D keypoints from current image
                std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                
                // TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                // -> HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, FREAK, SIFT
                detect_keypoints(detectorType, keypoints, imgGray, false, display_print);


                // TASK MP.3 -> only keep keypoints on the preceding vehicle
                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);

                if (bFocusOnVehicle)
                {   
                    std::vector<cv::KeyPoint> new_keypoints;
                    for(auto it=keypoints.begin(); it!=keypoints.end(); ++it)
                    {
                        if(vehicleRect.contains(it->pt))
                        {
                            new_keypoints.push_back(*it);
                        }
                    }  
                    keypoints = new_keypoints;
                }

                // optional : limit number of keypoints (helpful for debugging and learning) 
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    std::cout << " NOTE: Keypoints have been limited!" << std::endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;

                /* EXTRACT KEYPOINT DESCRIPTORS */
                // TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                // -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, display_print);

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;


                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */
                    std::vector<cv::DMatch> matches;
                    std::string matcherType = "MAT_BF"; // MAT_BF, MAT_FLANN

                    std::string descriptor_set; // DES_BINARY, DES_HOG
                    if((descriptorType.compare("SIFT") == 0))
                        descriptor_set = "DES_HOG";
                    else
                        descriptor_set = "DES_BINARY";

                    std::string selectorType = "SEL_KNN";   // SEL_NN, SEL_KNN

                    // TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    // TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                    matches, descriptor_set, matcherType, selectorType);

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;
                    total_matches += matches.size();


                    // visualize matches between current and previous image
                    bVis = true;
                    if (bVis)
                    {
                        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        std::string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }

            } // eof loop over all images
            std::cout << "Total number of matches: " << total_matches << std::endl;
        }
    }

    return 0;
}