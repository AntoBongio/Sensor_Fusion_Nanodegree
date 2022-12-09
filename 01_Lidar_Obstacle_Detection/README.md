# Sensor Fusion Self-Driving Car Course

## Project pipeline

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range.

To detect the obstacles in the environment, it is necessary to perform a few steps on the pcl. The following figure summarises the pipeline.

<img src="media/LIdar_project.png" width="600" height="350" />

#### Step 1: Filter PCL
Generate voxel grid with a resolution of 0.2 m and remove the roof from the pointcloud.

#### Step 2: Segment plane
Segment the filtered cloud into two parts, road and obstacles. 

#### Step 3: Clustering
Cluster the obstacle cloud based on the proximity of neighboring points. 

#### Step 4: Generate bounding boxes
Finally, place bounding boxes around the individual clusters. 

## Final result

<img src="media/project_result.gif" width="800" height="500" />

## Dependencies

* PCL - v1.10
* C++ v14
* gcc v9.4

## Local Installation

1. Install the latest version of PCL. You should be able to do all the classroom exercises and project with this setup.:

   ```sh
   sudo apt install libpcl-dev
   ```
   **Note** The library version of PCL being distributed by the apt repository for 18.04 and 20.04 are both older than v1.11. The following links have the information regarding the versions: [Bionic 18.04](https://www.ubuntuupdates.org/package/core/bionic/universe/updates/libpcl-dev) [Focal 20.04](https://www.ubuntuupdates.org/package/core/focal/universe/base/libpcl-dev)

You can either build PCL from source (for v1.11) or use the older version.

2. Clone this github repo:

   ```sh
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

3. Execute the following commands in a terminal

   ```shell
   cd Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```
