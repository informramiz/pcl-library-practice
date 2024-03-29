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
std::unordered_set<int> ProcessPointClouds<PointT>::GetInliers(typename pcl::PointCloud<PointT>::Ptr cloud, const Model<PointT>& model, float distanceTolerance) {
    std::unordered_set<int> inliers;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (model.distanceFromPoint(cloud->points[i]) <= distanceTolerance) {
			inliers.insert(i);
		}
	}
	return inliers;
}

template<typename PointT>
std::pair<std::unordered_set<int>, Plane<PointT> > ProcessPointClouds<PointT>::PlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	//COMMPLETED: Fill in this function
	int maxixumInliers = 0;
	Plane<PointT> planeWithMaxInliers;

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
		Plane<PointT> fittedPlane = Plane<PointT>(cloud->points[randomPointIndex1], cloud->points[randomPointIndex2], cloud->points[randomPointIndex3]);
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		auto inliersForThisPlane = GetInliers(cloud, fittedPlane, distanceTol);
		if (inliersForThisPlane.size() > inliersResult.size()) {
			inliersResult = inliersForThisPlane;
			planeWithMaxInliers = fittedPlane;
		} 
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return std::pair<std::unordered_set<int>, Plane<PointT> >(inliersResult, planeWithMaxInliers);
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // COMPLETED: Create two new point clouds, one cloud with obstacles and other with segmented plane
    //create object to hold the extracted points for the plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>), obstacleCloud(new pcl::PointCloud<PointT>);

    //create extracter
    pcl::ExtractIndices<PointT> extract;
    //give it input cloud
    extract.setInputCloud(cloud);

    //we want to extract inliers (plan points) first so set negative points to false to ignore them
    extract.setNegative(false);
    //set indices of points representing the plan
    extract.setIndices(inliers);
    //extract plan points in `planeCloud`
    extract.filter(*planeCloud);

    //now extract the obstacle points by ignoring the positive points (inliers, plane points) and just picking 
    //negative points (obstacles). It is basically subtraction of plane from the main cloud
    extract.setNegative(true);
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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneManual(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //to hold points that fit to the plane with error less than a threshold
    std::pair<std::unordered_set<int>, Plane<PointT> > inliersAndPlane = PlaneRansac(cloud, maxIterations, distanceThreshold);
    std::unordered_set<int> inliersIndices = inliersAndPlane.first;
    std::cout << "Model inliers: " << inliersIndices.size () << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //Separate inliers for plane and outliers: Create two new point clouds, one cloud with obstacles and other with segmented plane
    //create object to hold the extracted points for the plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>), obstacleCloud(new pcl::PointCloud<PointT>);
    for(int pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++) {
        auto point = cloud->points[pointIndex];
        if (inliersIndices.count(pointIndex)) {
            planeCloud->points.push_back(point);
        } else {
            obstacleCloud->points.push_back(point);
        }
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstacleCloud, planeCloud);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //COMPLETED:: Fill in the function to perform euclidean clustering to group detected obstacles

    //build KD-tree that will be used by Euclidean clustering algorithm for searching
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    //create a vector to hold clusters points indices
    std::vector<pcl::PointIndices> clusterPointIndices;
    //create euclidean clustering algorithm object
    pcl::EuclideanClusterExtraction<PointT> euclideanClusterer;
    euclideanClusterer.setInputCloud(cloud);
    euclideanClusterer.setSearchMethod(tree);
    euclideanClusterer.setClusterTolerance(clusterTolerance);
    euclideanClusterer.setMinClusterSize(minSize);
    euclideanClusterer.setMaxClusterSize(maxSize);
    euclideanClusterer.extract(clusterPointIndices);

    //now that we have cluster point indices, let's convert them into point clound for each cluster
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (pcl::PointIndices pointIndices : clusterPointIndices) {
        typename pcl::PointCloud<PointT>::Ptr clusterPointCloud(new pcl::PointCloud<PointT>);
        for (int pointIndex : pointIndices.indices) {
            clusterPointCloud->points.push_back(cloud->points[pointIndex]);
        }
        clusterPointCloud->width = clusterPointCloud->points.size();
        clusterPointCloud->height = 1;
        clusterPointCloud->is_dense = true;

        clusters.push_back(clusterPointCloud);
    }

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