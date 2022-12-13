# Project: 3D Object Tracking

<img src="images/course_code_structure.png" width="779" height="414" />

In this project, four major tasks have been addressed: 
1. Develop a way to match 3D objects over time by using keypoint correspondences. 
2. Compute the TTC based on Lidar measurements. 
3. Proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. Conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. 

## Dependencies for Running Locally
* cmake >= 2.8
* make >= 4.1 (Linux)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
  * Install Git LFS before cloning this Repo.
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
   cd 03_3D_Object_Tracking
   mkdir build && cd build
   cmake ..
   make
   ./3D_object_tracking
   ```
