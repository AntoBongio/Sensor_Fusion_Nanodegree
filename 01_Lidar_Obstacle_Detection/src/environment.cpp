/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"



std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}



void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    bool render_plane = true;
    bool render_obstacle = false;
    bool render_cluster = false;
    bool render_box = true;

    // Generate Lidar object
    auto lidar = std::make_shared<Lidar>(cars, 0.0);

    // Scan the environment
    auto input_cloud = lidar->scan();

    // Render rays from the function in render.h
    // renderRays(viewer, lidar->position, input_cloud);
    // renderPointCloud(viewer, input_cloud, "Input cloud");

    // Create point processor
    auto point_processor = std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
    auto pointI_processor = std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_cloud = point_processor->SegmentPlane(input_cloud, 100, 0.2);
    
    if(render_plane)
        renderPointCloud(viewer, segmented_cloud.first, "plane cloud", Color(0, 1, 0));
    if(render_obstacle)
        renderPointCloud(viewer, segmented_cloud.second, "obstacle cloud", Color(1, 0, 0));


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor->Clustering(segmented_cloud.second, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_cluster)
        {
            // std::cout << "cluster size ";
            // point_processor->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstacle_cloud"+std::to_string(clusterId), colors[clusterId]);
        }
        
        if(render_box)
        {
            Box box = point_processor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
    
        ++clusterId;
    }
}

template <typename PointT> void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
                                          ProcessPointClouds<PointT>* point_processor,
                                          typename pcl::PointCloud<PointT>::Ptr input_cloud )
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool render_filtered = false;
    bool render_plane = true;
    bool render_obstacle = false;
    bool render_cluster = true;
    bool render_box = true;

    // Filter cloud
    auto filtered_cloud = point_processor->FilterCloud(input_cloud, 0.2 , Eigen::Vector4f (-15, -6, -2, 1), Eigen::Vector4f (20, 6, 5, 1));
    
    if(render_filtered)
        renderPointCloud(viewer, filtered_cloud, "filtered_cloud");

    // Segment cloud
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmented_cloud = point_processor->SegmentPlane(filtered_cloud, 100, 0.2);
    
    if(render_plane)
        renderPointCloud(viewer, segmented_cloud.first, "plane cloud", Color(0, 1, 0));
    if(render_obstacle)
        renderPointCloud(viewer, segmented_cloud.second, "obstacle cloud", Color(1, 0, 0));

    // Clustering
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters = point_processor->Clustering(segmented_cloud.second, 0.5, 20, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0), Color(1,0,1), Color(0,1, 1), Color(0.5,0,0), Color(0.5, 0, 0.5)};

    for(typename pcl::PointCloud<PointT>::Ptr cluster : cloudClusters)
    {
        if(render_cluster)
        {
            // std::cout << "cluster size ";
            // point_processor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstacle_cloud"+std::to_string(clusterId), colors[clusterId %7]);
        }
        
        if(render_box)
        {
            Box box = point_processor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
    
        ++clusterId;
    }
    
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = Side; // XY, FPS
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

     // Load cloud
    ProcessPointClouds<pcl::PointXYZI>* point_processor_I = new ProcessPointClouds<pcl::PointXYZI>();


    // pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = point_processor_I->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // cityBlock(viewer, point_processor_I, input_cloud);
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // }

    std::vector<boost::filesystem::path> stream = point_processor_I->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        input_cloud = point_processor_I->loadPcd((*streamIterator).string());
        cityBlock(viewer, point_processor_I, input_cloud);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}