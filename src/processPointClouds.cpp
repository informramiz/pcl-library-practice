// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // COMPLETED: Create two new point clouds, one cloud with obstacles and other with segmented plane
    //create object to hold the extracted points for the plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(pcl::PointCloud<PointT>), obstacleCloud(pcl::PointCloud<PointT>);

    //create extracter
    pcl::ExtractIndices<PointT> extract;
    //give it input cloud
    extract.setInputCloud(cloud);

    //we want to extract inliers (plan points) first so set negative points to false to ignore them
    extract.setNegative(false);
    //set indices of points representing the plan
    extract.setIndices(*inliers);
    //extract plan points in `planeCloud`
    extract.filter(*planeCloud);

    //now extract the obstacle points by ignoring the positive points (inliers, plane points) and just picking 
    //negative points (obstacles). It is basically subtraction of plane from the main cloud
    extract.setNagative(true);
    extract.filter(*obstacleCloud);

    std::cout << "Points representing plane: " << planeCloud->width * planeCloud->height << std::endl;
    std::cout << "Points representing obstacles: " << obstacleCloud->width * obstacleCloud->height << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // COMPLETED:: Fill in this function to find inliers for the cloud.

    //coefficients object hold model coefficients [a, b, c, d]. (in this case, plan, ax + by + cz + d) 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    //to hold points that fit to the plane with error less than a threshold
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    //create segmentation object to perform segmentation on points
    pcl::SACSegmentation<PointT> seg;

    //set segmentation params
    //we want to fit a plan to detect road so set model type plan
    seg.setModelType(pcl::SACMODEL_PLANE);
    //we want to use RANSAC algorithm to fit the plan model
    seg.setMethodType(pcl::SAC_RANSAC);
    //As RANSAC is an iterative algorithm so we set the max iterations
    //the more the iterations the better results but slower 
    seg.setMaxIterations(maxIterations);
    //“distance threshold”, which determines how close a point must be to the model in order to be considered an inlier.
    seg.setDistanceThreshold(distanceThreshold);

    //optional, ask algorithm to optimize model (plan in this case) coefficients (a, b, c, d) as much possible
    seg.setOptimizeCoefficients(true);
    //give the input cloud
    seg.setInputCloud(cloud);

    //run the segmentation
    seg.segment(*inliers, *coefficients);

    std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    std::cout << "Model inliers: " << inliers->indices.size () << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}