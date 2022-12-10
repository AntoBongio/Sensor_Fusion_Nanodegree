# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of this midterm project is to build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. It consists of four parts:

* Focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* Focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* Test the various algorithms in different combinations and compare them with regard to some performance measures. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
