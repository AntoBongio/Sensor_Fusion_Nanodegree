
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// Associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{

    /*  
    Check which keypoint matches belong to the bounding box (check whether the corresponding keypoints 
    are within the region of interest in the camera image). Add these matches to a vector. 
    Problem: there will be outliers among the matches --> compute a robust mean of all the eucliean 
    distances between the keypoint matches and then remove those that are too far away from the mean.
    */

    std::vector<cv::DMatch> match_candidates;

    /* 
    I want to compute the distribution of the euclidean distsances. I have to compute:
    - Mean of the distribution
    - Standard deviation
    Then I can use this information to remove the outliers.
    */
    std::vector<double> euclidean_distance;
    double euclidean_distance_mean {0};
    double euclidean_distance_std {0};
    int count{0};

    // Check which keypoints matches belong to the bounding box (within the roi).
    for(auto it=kptMatches.begin(); it!=kptMatches.end(); ++it)
    {
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it->queryIdx);

        // IF they are inside roi, push to the vector as possible candidates.
        if(boundingBox.roi.contains(kpOuterCurr.pt) and boundingBox.roi.contains(kpOuterPrev.pt))
        {
            cv::Point2f diff = kpOuterCurr.pt - kpOuterPrev.pt;
            double distance = cv::sqrt(diff.x*diff.x + diff.y*diff.y);

            match_candidates.push_back(*it);
            euclidean_distance.push_back(distance);
            euclidean_distance_mean += distance;
            ++count;
        }
    }

    if(count > 0)
       euclidean_distance_mean /= count;

    // Eval standard deviation
    for(const auto& elem: euclidean_distance)
    {
        euclidean_distance_std += (elem - euclidean_distance_mean)*(elem - euclidean_distance_mean);
    }
    if(count > 1)
        euclidean_distance_std = std::sqrt(euclidean_distance_std/(count-1));
    else
        euclidean_distance_std = std::sqrt(euclidean_distance_std);

    std::vector<cv::DMatch> correct_matches;
    std::vector<cv::KeyPoint> correct_keypoints;


    for(const auto& match: match_candidates)
    {
        cv::KeyPoint kpOuterCurr = kptsCurr.at(match.trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(match.queryIdx);

        cv::Point2f diff = kpOuterCurr.pt - kpOuterPrev.pt;
        double distance = cv::sqrt(diff.x*diff.x + diff.y*diff.y);

        // 2sigma --> 95% of the points
        if((distance <= euclidean_distance_mean + 2*euclidean_distance_std) && (distance >= euclidean_distance_mean - 2*euclidean_distance_std))
        {
            correct_matches.push_back(match);
            correct_keypoints.push_back(kpOuterCurr);
        }
    }
    
    // std::cout << std::endl;
    // std::cout << "match_candidates: " << match_candidates.size() << ", correct_matches: " << correct_matches.size() << std::endl;
    // std::cout << std::endl;

    boundingBox.kptMatches = correct_matches;
    boundingBox.keypoints = correct_keypoints;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    std::vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    } // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios

    // // Mean version
    // double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    // double dT = 1 / frameRate;
    // TTC = -dT / (1 - meanDistRatio);

    // Median version
    std::sort(distRatios.begin(), distRatios.end());
    int med_index = std::floor(distRatios.size() / 2);

    double dT = 1 / frameRate;
    double medianDistRation = distRatios.size() % 2 == 0 ? (distRatios[med_index - 1] + distRatios[med_index])/2 : distRatios[med_index];

    TTC = -dT / (1 - medianDistRation);
    // std::cout << "Camera TTC: " << TTC << std::endl;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1/frameRate;
    double laneWidth = 4.0; // assumed width of the ego lane
    double minXPrev {0}; double minXCurr {0};
    double count {0};

    double minimum_XPrev{1e8};
    double minimum_XCurr{1e8};

    // Compute the mean of the x value of keypoints in lidarPointsPrev to mitigate the presence of outliers.
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            ++count;
            minXPrev += it->x;

            if(it->x < minimum_XPrev)
                minimum_XPrev = it->x;
        }
    }
    if(count == 0) count = 1;
    minXPrev = minXPrev / count;

    count = 0;
    // Compute the mean of the x value of keypoints in lidarPointsCurr to mitigate the presence of outliers.
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            ++count;
            minXCurr += it->x;

            if(it->x < minimum_XCurr)
                minimum_XCurr = it->x;
        }
    }
    if(count == 0) count = 1;
    minXCurr = minXCurr / count;

    // std::cout << "minXPrev: " << minXPrev << ", minXCurr: " << minXCurr << std::endl;
    // std::cout << "frameRate: " << frameRate << ", dT: " << 1/frameRate << std::endl;

    // compute TTC from both measurements
    TTC = minimum_XCurr * dT / (minimum_XPrev - minimum_XCurr);
    // std::cout << "TTC with minimum: " << TTC << std::endl;

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
    // std::cout << "TTC with mean: " << TTC << std::endl;
}

typedef std::pair<int, int> boxID_pair;
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // Reminder: prevFrame --> queryIdx / currFrame --> trainIdx
    std::map<boxID_pair, int> bb_matches;
    bool display = false;
    /*
    I have to verify if the previous BB contains the previous keypoint (prevFrame.keypoints[match.queryIdx]), while
    at the same time, the current BB contains the current keypoint (currFrame.keypoints[match.trainIdx]).
    If so, I can add 1 to the match counter between prev_bb_id and curr_bb_id.
    */

    /*
    The datastructure exploited is a map with a std::pair<int, int> as key (prev_boxID, curr_boxID)
    and the good matches inside both as value.
    */

    std::vector<int> prev_bb_IDs; // Needed for the second step of the algorithm
    
    for(const auto& match: matches)
    {
        // Check keypoint inside the previous bounding box
        for(const auto& prev_bb: prevFrame.boundingBoxes)
        {
            prev_bb_IDs.push_back(prev_bb.boxID);
            auto prev_keypoint = prevFrame.keypoints[match.queryIdx];

            if(prev_bb.roi.contains(prev_keypoint.pt))
            {
                auto curr_keypoint = currFrame.keypoints[match.trainIdx];

                // Check keypoint inside the current bounding box
                for(const auto& curr_bb: currFrame.boundingBoxes)
                {
                    if(curr_bb.roi.contains(curr_keypoint.pt))
                    {
                        // FOUND a good match
                        bb_matches[std::make_pair(prev_bb.boxID, curr_bb.boxID)] += 1;
                    }
                }
            }
        }
    }

    // Display map
    display = false;
    if(display)
    {
        std::cout << std::endl;
        std::cout << "bb_matches: ";
        for (const auto &entry: bb_matches)
        {
            auto key_pair = entry.first;
            std::cout << "{" << key_pair.first << "," << key_pair.second << "}, --> " << entry.second << std::endl;
        }
        std::cout << std::endl;
    }
    display = false;

    /*
    Iterate over the bb_ID in prevFrame. 
    Search in bb_matches for the pair with first id equale to bb_ID and max occurrences.
    Add this pair to bbBEstMatches. 
    */
   for(const auto& prev_bb_ID: prev_bb_IDs)
   {
    int max_occur = -1;
    int best_curr_bb_ID;

    for(auto it_matches=bb_matches.begin(); it_matches!=bb_matches.end(); ++it_matches)
    {
        if((prev_bb_ID == it_matches->first.first) && (it_matches->second > max_occur))
        {
            max_occur = it_matches->second;
            best_curr_bb_ID = it_matches->first.second;
        }
    }
    if(max_occur > 0)
        bbBestMatches[prev_bb_ID] = best_curr_bb_ID;
   }

}
