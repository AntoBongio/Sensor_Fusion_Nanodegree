
# Udacity - Sensor Fusion Nanodegree

This Nanodegree program consists of four core courses:
- Lidar
- Camera
- Radar
- Kalman filters

Each course concludes with a project. [Nanodegree certificate](https://confirm.udacity.com/U5XUPGDV).

## [Project 1 - Lidar Obstacle Detection](https://github.com/AntoBongio/Sensor_Fusion_Nanodegree/tree/main/01_Lidar_Obstacle_Detection)

In this project, the main goal is to process a point clouds in order to detect
car and trucks on a narrow street using lidar. The detection pipeline consists
of filtering, segmentation, and clustering methods to extract the bounding
boxes surrounding the target objects. The segmentation and clustering methods
contains my own implmentation of RANSAC, KD-Tree, and Euclidean clustering
algorithms.

## [Project 2 - 2D Feature Tracking](https://github.com/AntoBongio/Sensor_Fusion_Nanodegree/tree/main/02_2D_Feature_Tracking)

This project was created to familiarize yourself with the OpenCV API that allows you to extract features, build feature descriptors and run feature matching algorithms. 
Feature Detectors: Shi-Tomasi, Harris, FAST, BRISK, ORB, AKAZE, SIFT
Feature Descriptors: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT


## [Project 3 - 3D Object Tracking (Lidar/Camera)](https://github.com/AntoBongio/Sensor_Fusion_Nanodegree/tree/main/03_3D_Object_Tracking)

This is the final project of the camera core course, the main idea is to combine both lidar and camera data for 3D object tracking. An object detection algorithm is used to first identify the region of interest (ROI) in the colour image. Then the associated keypoint correspondences are used to match 3D objects over time. The time-to-collision (TTC) can be estimated based on lidar 3D points and key point matching pairs within the ROIs.

## [Project 4 - Radar](https://github.com/AntoBongio/Sensor_Fusion_Nanodegree/tree/main/04_Radar)

The Radar sensor gives us the ability to estimate the position of a target as well as its velocity, proving to be complementary to Lidar. In the final project, the goal was to generate an artificial moving target, by assuming it is moving with the constant velocity model,  and to simulate a transmitted and received signal as what would be done in a radar module. The signal it is later exploited to calculate Range-Doppler Map (RDM) to ascertain the position and velocity of the target. Because the RDM is inherently noisy due to radar clutter, the CFAR is calculated in 2D in this RDM to finally detect where the object is and its speed.

## [Project 5 - Unscented Kalman Filter (Lidar/Radar)](https://github.com)


