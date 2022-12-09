/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <math.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}


struct Line2D{

	float A, B, C;

	Line2D(float A, float B, float C) : A{A}, B{B}, C{C} {};

	inline float distance(const pcl::PointXYZ& point) {
		return std::fabs(A * point.x + B * point.y + C) / std::sqrt(A*A + B*B);
	}

	void print() {
		std::cout << "A: " << A << ", B: " << B << ", C: " << C << std::endl;
	}

};

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto start_time = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto cloud_length = cloud->points.size();

	while(maxIterations--) {

		// Randomly sample 2 points
		std::unordered_set<int> inliers;

		while(inliers.size() < 2)
			inliers.insert(rand() % cloud_length);

		// Fit a line
		auto itr = inliers.begin();

		float x1, x2, y1, y2;

		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		++itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		Line2D line(y1 - y2, x2 - x1, x1*y2 - x2*y1);		

		// Measure distance between every point and fitted line
		for(int index=0; index<cloud_length; ++index) {

			if (inliers.count(index) > 0) continue; 

			auto point = cloud->points[index];
			float distance = line.distance(point);

			// If distance is smaller than threshold count it as inlier
			if (distance <= distanceTol)
				inliers.insert(index);
		}

		// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}

	auto end_time = std::chrono::steady_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
	std::cout << "Ransac took: " << elapsed_time.count() << " nanoseconds." << std::endl;

	return inliersResult;
}

struct Plane{

	float A, B, C, D;

	Plane(float A, float B, float C, float D) : A{A}, B{B}, C{C}, D{D} {};

	inline float distance(const pcl::PointXYZ& point) {
		return std::fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A*A + B*B + C*C);
	}

	void print() {
		std::cout << "A: " << A << ", B: " << B << ", C: " << C << ", D: " << D << std::endl;
	}

};


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto start_time = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto cloud_length = cloud->points.size();

	while(maxIterations--) {

		// Randomly sample 2 points
		std::unordered_set<int> inliers;

		while(inliers.size() < 3)
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

		float i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1); 

		Plane plane(i, j, k, -(i*x1 + j*y1 + k*z1));		

		// Measure distance between every point and fitted line
		for(int index=0; index<cloud_length; ++index) {

			if (inliers.count(index) > 0) continue; 

			auto point = cloud->points[index];
			float distance = plane.distance(point);

			// If distance is smaller than threshold count it as inlier
			if (distance <= distanceTol)
				inliers.insert(index);
		}

		// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}

	auto end_time = std::chrono::steady_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "Ransac took: " << elapsed_time.count() << " milliseconds." << std::endl;

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
