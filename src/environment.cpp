/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void renderSegmentationResult(pcl::visualization::PCLVisualizer::Ptr& viewer,
std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentationResult) {
     //render both road/plane and obstacles
    renderPointCloud(viewer, segmentationResult.first, "obstacles", Color(1, 0, 0));
    renderPointCloud(viewer, segmentationResult.second, "plane Cloud", Color(0, 1, 0));
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // COMPLETED:: Create lidar sensor 
    Lidar* lidarPtr = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidarPtr->scan();
    //uncomment to render Lidar rays
    //renderRays(viewer, lidarPtr->position, cloud);
    // renderPointCloud(viewer, cloud, "My Point cloud");

    // COMPLETED:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentationResult = pointCloudProcessor.SegmentPlaneManual(cloud, 100, 0.2);
    // renderSegmentationResult(viewer, segmentationResult);

    //cluster the obstacles so that they can be worked on separately
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterClouds = pointCloudProcessor.Clustering(segmentationResult.first, 1.0, 3, 30);
    //define colors for each cluster (we are sure there will be 3 clusters in given example so only 3 colors)
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    //now let's render each cluster
    int clusterId = 0;
    for (typename pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud : clusterClouds) {
        std::cout << "Clustermake size: "; pointCloudProcessor.numPoints(clusterCloud);
        renderPointCloud(viewer, clusterCloud, "cluster-" + std::to_string(clusterId), colors[clusterId % 3]);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}