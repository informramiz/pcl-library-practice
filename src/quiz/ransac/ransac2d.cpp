/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <math.h>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include "../../plane.h"
#include "../../line.h"


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

std::unordered_set<int> CountInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Model<pcl::PointXYZ>& line, float distanceTolerance) {
	std::unordered_set<int> inliers;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (line.distanceFromPoint(cloud->points[i]) <= distanceTolerance) {
			inliers.insert(i);
		}
	}
	return inliers;
}

std::pair<std::unordered_set<int>, Line<pcl::PointXYZ> > LineRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	//COMMPLETED: Fill in this function
	int maxixumInliers = 0;
	Line<pcl::PointXYZ> planeWithMaxInliers;

	// For max iterations 
	for (int i = 1; i <= maxIterations; ++i) {
		// Randomly sample subset and fit line
		int randomPointIndex1 = rand() % cloud->points.size();
		int randomPointIndex2 = rand() % cloud->points.size();
		Line<pcl::PointXYZ> fittedPlane = Line<pcl::PointXYZ>(cloud->points[randomPointIndex1], cloud->points[randomPointIndex2]);
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		auto inliersForThisPlane = CountInliers(cloud, fittedPlane, distanceTol);
		if (inliersForThisPlane.size() > inliersResult.size()) {
			inliersResult = inliersForThisPlane;
			planeWithMaxInliers = fittedPlane;
		} 
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return std::pair<std::unordered_set<int>, Line<pcl::PointXYZ> >(inliersResult, planeWithMaxInliers);
}

std::pair<std::unordered_set<int>, Plane<pcl::PointXYZ> > PlaneRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	//COMMPLETED: Fill in this function
	int maxixumInliers = 0;
	Plane<pcl::PointXYZ> planeWithMaxInliers;

	// For max iterations 
	for (int i = 1; i <= maxIterations; ++i) {
		std::unordered_set<int> randomPointIndices;
		while (randomPointIndices.size() < 3) {
			int randomPointIndex = rand() % cloud->points.size();
			randomPointIndices.insert(randomPointIndex);
		}
		// Randomly sample subset and fit Plane
		std::unordered_set<int>::iterator iterator = randomPointIndices.begin();
		int randomPointIndex1 = *iterator;
		iterator++;
		int randomPointIndex2 = *iterator;
		iterator++;
		int randomPointIndex3 = *iterator;
		Plane<pcl::PointXYZ> fittedPlane = Plane<pcl::PointXYZ>(cloud->points[randomPointIndex1], cloud->points[randomPointIndex2], cloud->points[randomPointIndex3]);
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		auto inliersForThisPlane = CountInliers(cloud, fittedPlane, distanceTol);
		if (inliersForThisPlane.size() > inliersResult.size()) {
			inliersResult = inliersForThisPlane;
			planeWithMaxInliers = fittedPlane;
		} 
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return std::pair<std::unordered_set<int>, Plane<pcl::PointXYZ> >(inliersResult, planeWithMaxInliers);
}

void renderInliersAndOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::unordered_set<int>& inliers) {
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
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

void testLineRansac() {
	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	
	// COMPLETED: Change the max iteration and distance tolerance arguments for LineRansac function
	std::pair<std::unordered_set<int>, Line<pcl::PointXYZ> > inliersAndPlane = LineRansac(cloud, 50, 0.8);
	Line<pcl::PointXYZ> line = inliersAndPlane.second;
	std::unordered_set<int> inliers = inliersAndPlane.first;
	renderInliersAndOutliers(cloud, inliers);  	
}

void testPlaneRansac() {
	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// COMPLETED: Change the max iteration and distance tolerance arguments for LineRansac function
	std::pair<std::unordered_set<int>, Plane<pcl::PointXYZ> > inliersAndPlane = PlaneRansac(cloud, 50, 0.3);
	Plane<pcl::PointXYZ> plane = inliersAndPlane.second;
	std::unordered_set<int> inliers = inliersAndPlane.first;
	renderInliersAndOutliers(cloud, inliers);  	
}

void testPlaneModelCorrectness() {
	pcl::PointXYZ point1, point2, point3;
	point1.x = 2;
	point1.y = 3;
	point1.z = 0;

	point2.x = 4;
	point2.y = 5;
	point2.z = 0;

	point3.x = 10;
	point3.y = 15;
	point3.z = 0;

	Plane<pcl::PointXYZ> plane(point1, point2, point3);
	std::cout << "Plane(a, b, c, d) = (" << plane.a << ", " << plane.b << ", " << plane.c << ", " << plane.d << ")" << std::endl;
	std::cout << "Distance from Point: " << plane.distanceFromPoint(point1) << std::endl;
}

int main () {
	testPlaneRansac();
	// testLineRansac();
}
