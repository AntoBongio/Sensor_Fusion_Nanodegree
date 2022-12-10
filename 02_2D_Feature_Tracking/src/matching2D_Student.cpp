#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);

    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        // Implement FLANN matching
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches

        // Filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }

    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, bool display_print)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if(display_print)
        std::cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << std::endl;
}


void detect_keypoints(std::string type, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool display_print)
{

    std::string detector_name;
    double t;
    std::vector<cv::Point2f> corners;

    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double k = 0.04;

    if(type.compare("SHITOMASI") == 0)
    {
        detector_name = "Shi-Tomasi";
        
        double minDistance = (1.0 - maxOverlap) * blockSize;
        int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

        double qualityLevel = 0.01; // minimal accepted quality of image corners
        
        t = (double)cv::getTickCount();
        cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

        // add corners to result vector
        for (auto it = corners.begin(); it != corners.end(); ++it)
        {
            cv::KeyPoint newKeyPoint;
            newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
            newKeyPoint.size = blockSize;
            keypoints.push_back(newKeyPoint);
        }

    }
    if(type.compare("HARRIS") == 0)
    {
        detector_name = "Harris";
        int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
        int minResponse = 100;

        t = (double)cv::getTickCount();
        cv::Mat dst, dst_norm, dst_norm_scaled;
        dst = cv::Mat::zeros(img.size(), CV_32FC1);
        cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        cv::convertScaleAbs(dst_norm, dst_norm_scaled);

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

                    for(auto it=keypoints.begin(); it != keypoints.end(); ++it)
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
                        keypoints.push_back(new_key_point); // store new keypoint in dynamic list
                    }
                }
            }
        }
    }
    if(type.compare("FAST") == 0)
    {
        detector_name = "FAST";
        int threshold = 10;
        bool non_maxima_suppresion = true;
        cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(threshold, non_maxima_suppresion);
        
        t = (double)cv::getTickCount();
        fast->detect(img, keypoints);
    }
    if(type.compare("BRISK") == 0)
    {
        detector_name = "BRISK";
        cv::Ptr<cv::FeatureDetector> brisk = cv::BRISK::create();

        t = (double)cv::getTickCount();
        brisk->detect(img, keypoints);
    }
    if(type.compare("ORB") == 0)
    {
        detector_name = "ORB";
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        
        t = (double)cv::getTickCount();
        orb->detect(img, keypoints);
    }
    if(type.compare("AKAZE") == 0)
    {
        detector_name = "AKAZE";
        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

        t = (double)cv::getTickCount();
        akaze->detect(img, keypoints);
    }
    if(type.compare("SIFT") == 0)
    {
        detector_name = "SIFT";
        cv::Ptr<cv::SiftFeatureDetector> sift = cv::SiftFeatureDetector::create();

        t = (double)cv::getTickCount();
        sift->detect(img, keypoints);
    }
    
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if(display_print)
        std::cout << detector_name << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = detector_name + " Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}