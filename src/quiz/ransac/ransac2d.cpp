/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <math.h>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

class Model{
public:
	virtual float distanceFromPoint(const pcl::PointXYZ& point) const = 0;
};

struct Line: Model {
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

	float distanceFromPoint(const pcl::PointXYZ& point) const {
		return abs(a * point.x + b * point.y + c) / sqrtf(a * a + b * b);
	}
};

struct Plane: Model {
	int a;
	int b;
	int c;
	int d;

	Plane() {
		a = 0;
		b = 0;
		c = 0;
		d = 0;
	}

	Plane(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2, const pcl::PointXYZ& point3) {
		//a = (y2−y1)(z3−z1)−(z2−z1)(y3−y1)
		a = (point2.y - point1.y)*(point3.z - point1.z) - (point2.z - point1.z)*(point3.y - point1.y);
		//b = (z2-z1)(x3-x1)-(x2-x1)(z3-z1)
		b = (point2.z - point1.z)*(point3.x - point1.x) - (point2.x - point1.x)*(point3.z - point1.z);
		//c = (x2−x1)(y3−y1)−(y2−y1)(x3−x1)
		c = (point2.x - point1.x)*(point3.y-point1.y) - (point2.y - point1.y)*(point3.x-point1.x);
		// d = −(ix1+jy1+kz1)
		d = -(a*point1.x + b*point1.y + c*point1.z);
	}

	float distanceFromPoint(const pcl::PointXYZ& point) const {
		//d=∣A∗x+B∗y+C∗z+D∣/sqrt(A^2+B^2+C^2)
		return fabs(a*point.x + b*point.y + c*point.z + d) / sqrtf(a*a + b*b + c*c);
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

std::unordered_set<int> CountInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Model& line, float distanceTolerance) {
	std::unordered_set<int> inliers;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (line.distanceFromPoint(cloud->points[i]) <= distanceTolerance) {
			inliers.insert(i);
		}
	}
	return inliers;
}

std::pair<std::unordered_set<int>, Line> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	//COMMPLETED: Fill in this function
	int maxixumInliers = 0;
	Line lineWithMaxInliers;

	// For max iterations 
	for (int i = 1; i <= maxIterations; ++i) {
		// Randomly sample subset and fit line
		int randomPointIndex1 = rand() % cloud->points.size();
		int randomPointIndex2 = rand() % cloud->points.size();
		Line fittedLine = Line(cloud->points[randomPointIndex1], cloud->points[randomPointIndex2]);
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		auto inliersForThisLine = CountInliers(cloud, fittedLine, distanceTol);
		if (inliersForThisLine.size() > inliersResult.size()) {
			inliersResult = inliersForThisLine;
			lineWithMaxInliers = fittedLine;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return std::pair<std::unordered_set<int>, Line>(inliersResult, lineWithMaxInliers);

}

int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// COMPLETED: Change the max iteration and distance tolerance arguments for Ransac function
	std::pair<std::unordered_set<int>, Line> inliersAndLine = Ransac(cloud, 50, 0.8);
	Line line = inliersAndLine.second;
	std::unordered_set<int> inliers = inliersAndLine.first;

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
