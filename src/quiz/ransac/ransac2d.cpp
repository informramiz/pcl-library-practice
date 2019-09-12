/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <math.h>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

struct Line {
	int a;
	int b;
	int c;

	Line() {
		a = 0;
		b = 0;
		c = 0;
	}

	Line(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
		a = point1.y - point2.y;
		b = point2.x - point1.x;
		c = (point1.x * point2.y) - (point2.x * point1.y);
	}

	float distanceFromPoint(const pcl::PointXYZ& point) {
		return abs(a * point.x + b * point.y + c) / sqrtf(a * a + b * b);
	}
};

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int maxixumInliers = 0;

	// For max iterations 
	for (int i = 1; i <= maxIterations; ++i) {
		// Randomly sample subset and fit line

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	pcl::PointXYZ point1;
	point1.x = 2;
	point1.y = 3;
	point1.z = 0;
	pcl::PointXYZ point2;
	point2.x = 3;
	point2.y = 7;
	point2.z = 0;

	Line line(point1, point2);

	std::cout << "Line coefficients (a, b, c) = (" << line.a << ", " << line.b << ", " << line.c << ")" << std::endl;

	std::cout << "Distance from point: " << line.distanceFromPoint(point1) << std::endl;
	// // Create viewer
	// pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// // Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// // TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 0, 0);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	// for(int index = 0; index < cloud->points.size(); index++)
	// {
	// 	pcl::PointXYZ point = cloud->points[index];
	// 	if(inliers.count(index))
	// 		cloudInliers->points.push_back(point);
	// 	else
	// 		cloudOutliers->points.push_back(point);
	// }


	// // Render 2D point cloud with inliers and outliers
	// if(inliers.size())
	// {
	// 	renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  	// 	renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	// }
  	// else
  	// {
  	// 	renderPointCloud(viewer,cloud,"data");
  	// }
	
  	// while (!viewer->wasStopped ())
  	// {
  	//   viewer->spinOnce ();
  	// }
  	
}
