#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>


#include "dataStructures.h"
#include "structIO.hpp"

using namespace std;

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double &TTC)
{
    // auxiliary variables
    double dT = 0.1;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;

    // std::cout << std::endl;
    // std::cout << "lidarPointsPrev:" << std::endl;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        // std::cout << "(x, y, z): (" << it->x << ", " << it->y << ", " << it->z << ")" << std::endl; 
        if(it->y > -laneWidth/2 and it->y < laneWidth/2) // Check 3D point within ego lane
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }
    // std::cout << std::endl;

    // std::cout << "lidarPointsCurr:" << std::endl;
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        // std::cout << "(x, y, z): (" << it->x << ", " << it->y << ", " << it->z << ")" << std::endl; 
        if(it->y > -laneWidth/2 and it->y < laneWidth/2) // Check 3D point within ego lane
            minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }
    // std::cout << std::endl;

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

int main()
{

    std::vector<LidarPoint> currLidarPts, prevLidarPts;
    readLidarPts("../dat/C22A5_currLidarPts.dat", currLidarPts);
    readLidarPts("../dat/C22A5_prevLidarPts.dat", prevLidarPts);


    double ttc;
    computeTTCLidar(prevLidarPts, currLidarPts, ttc);
    cout << "ttc = " << ttc << "s" << endl;
}