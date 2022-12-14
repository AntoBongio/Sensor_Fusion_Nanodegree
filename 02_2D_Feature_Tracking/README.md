# Project: 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of this midterm project is to build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. It consists of four parts:

* Focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Integrate several keypoint detectors (HARRIS, FAST, ORB, ..) and compare them with regard to number of keypoints and speed. 
* Focus on descriptor extraction (FAST, BRISK, ORB, ..) and match them using brute force and the FLANN approaches. 
* Test the various algorithms in different combinations and compare them with regard to: descriptor accuracy, computational time. 

## Dependencies

* cmake >= 2.8
* make >= 4.1 (Linux)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## Local Installation

1. Clone the entire project github repo:

   ```sh
   git clone https://github.com/AntoBongio/Sensor_Fusion_Nanodegree.git
   ```

2. Execute the following commands in a terminal

   ```shell
   cd 02_2D_Feature_Tracking
   mkdir build && cd build
   cmake ..
   make
   ./2D_feature_tracking
   ```
