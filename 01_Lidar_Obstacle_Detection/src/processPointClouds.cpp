// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "kdtree.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    // auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<PointT>>();

    // Create the filtering object and filter with resolution given by filterRes
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloud_filtered);

    // Crop the point cloud
    typename pcl::PointCloud<PointT>::Ptr cloud_region = boost::make_shared<pcl::PointCloud<PointT>>();

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    // Extract roof points
    typename pcl::PointCloud<PointT>::Ptr cloud_region_without_roof = boost::make_shared<pcl::PointCloud<PointT>>();
    
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -2.7, -2, 1));
    roof.setMax(Eigen::Vector4f (3.6, 2.7,  -.4, 1));
    roof.setNegative(true); // With this parameter, you can invert the filtering behavior
    roof.setInputCloud(cloud_region);
    roof.filter(*cloud_region_without_roof);

    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region_without_roof;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud = boost::make_shared<pcl::PointCloud<PointT>>();

    // Create a point cloud containing only the inliers
    for (const auto &elem : inliers->indices)
    {
        plane_cloud->points.push_back(cloud->points[elem]);
    }

    pcl::ExtractIndices<PointT> extract;
    // Remove the inliers from the point cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}

template <typename PointT>
struct Plane
{

    float A, B, C, D;

    Plane(float A, float B, float C, float D) : A{A}, B{B}, C{C}, D{D} {};

    inline float distance(const PointT &point)
    {
        return std::fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);
    }

    void print()
    {
        std::cout << "A: " << A << ", B: " << B << ", C: " << C << ", D: " << D << std::endl;
    }
};

template <typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // auto start_time = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    auto cloud_length = cloud->points.size();

    while (maxIterations--)
    {

        // Randomly sample 2 points
        std::unordered_set<int> inliers;

        while (inliers.size() < 3)
            inliers.insert(rand() % cloud_length);

        // Fit a line
        auto itr = inliers.begin();

        float x1, x2, x3, y1, y2, y3, z1, z2, z3;

        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        ++itr;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        ++itr;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

        Plane<PointT> plane(i, j, k, -(i * x1 + j * y1 + k * z1));

        // Measure distance between every point and fitted line
        for (int index = 0; index < cloud_length; ++index)
        {

            if (inliers.count(index) > 0)
                continue;

            auto point = cloud->points[index];
            float distance = plane.distance(point);

            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceTol)
                inliers.insert(index);
        }

        // Return indicies of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    // auto end_time = std::chrono::steady_clock::now();
    // auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // std::cout << "Ransac took: " << elapsed_time.count() << " milliseconds." << std::endl;

    return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    // auto startTime = std::chrono::steady_clock::now();

    // Create the segmentation object
    // pcl::SACSegmentation<PointT> seg;

    // Find inliers for the cloud.
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (maxIterations);
    // seg.setDistanceThreshold (distanceThreshold);

    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    std::unordered_set<int> inliers = Ransac<PointT>(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr plane_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud = boost::make_shared<pcl::PointCloud<PointT>>();

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            plane_cloud->points.push_back(point);
        else
            obstacle_cloud->points.push_back(point);
    }

    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(plane_cloud, obstacle_cloud);
}

template <typename PointT> 
void proximity(PointT point, int id, std::set<int>& visited, std::vector<int>& cluster, KdTree<PointT>* tree, float distanceTol)
{
	// Check point index as visited
	visited.insert(id);
	// Add point index to cluster
	cluster.push_back(id);
	// Search for newarby points
	auto nearby_points_od = tree->search(point, distanceTol);
	// Iterate through nearby points
	for (const auto& neighbor_id: nearby_points_od)
	{
		if(visited.count(neighbor_id))
			continue;
		
		proximity(point, neighbor_id, visited, cluster, tree, distanceTol);
	}
}

template <typename PointT>
std::vector<std::vector<int>> euclideanCluster(std::vector<PointT, Eigen::aligned_allocator<PointT>> points, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
	// Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::set<int> visited;

	for (int i=0; i<points.size(); ++i)
	{
		if(visited.count(i))
			continue;
		
		std::vector<int> cluster;
		proximity(points[i], i, visited, cluster, tree, distanceTol);

        if(cluster.size() > minSize and cluster.size() < maxSize)
		    clusters.push_back(cluster);
	}
 
	return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    // auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;


    /* ##########################################################################################*/

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // in meters
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster = boost::make_shared<typename pcl::PointCloud<PointT>>();

        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    /* ##########################################################################################*/

    // KdTree<PointT>* tree = new KdTree<PointT>();
    // int id = 0;

    // for(PointT point: cloud->points)
    // {
    //     tree->insert(point, id);
    //     ++id;
    // }
    // std::vector<std::vector<int>> cluster_indices = euclideanCluster<PointT>(cloud->points, tree, clusterTolerance, minSize, maxSize);

    // for (const auto &cluster : cluster_indices)
    // {
    //     typename pcl::PointCloud<PointT>::Ptr cloud_cluster = boost::make_shared<typename pcl::PointCloud<PointT>>();

    //     for (const auto &idx : cluster)
    //     {
    //         cloud_cluster->push_back((*cloud)[idx]);
    //     }

    //     cloud_cluster->width = cloud_cluster->points.size();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;

    //     clusters.push_back(cloud_cluster);
    // }

    /* ##########################################################################################*/


    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    // std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}