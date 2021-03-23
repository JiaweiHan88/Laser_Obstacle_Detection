// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
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
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudregion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(false);
    region.setMax(maxPoint);
    region.setMin(minPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudregion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(false);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudregion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudregion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudregion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudregion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Extract the inliers
    // Create the filtering object
    std::cerr << "start SeparateClouds" << std::endl;
    auto cloud1(new pcl::PointCloud<PointT>);
    auto cloud2(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud1);
    extract.setNegative(true);
    extract.filter(*cloud2);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud2, cloud1);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    std::cerr << "start segment" << std::endl;
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_Custom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacle(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_road(new pcl::PointCloud<PointT>);

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    PointT point1, point2, point3;
    int idx1;
    int idx2;
    int idx3;
    float a, b, c, d, dist;
    while (maxIterations--)
    {
        std::unordered_set<int> inliers_set;

        while (inliers_set.size() < 3)
            inliers_set.insert(rand() % (cloud->points.size()));

        auto iter = inliers_set.begin();
        idx1 = *iter;
        ++iter;
        idx2 = *iter;
        ++iter;
        idx3 = *iter;
        point1 = cloud->points[idx1];
        point2 = cloud->points[idx2];
        point3 = cloud->points[idx3];

        a = ((point2.y - point1.y) * (point3.z - point1.z)) - ((point2.z - point1.z) * (point3.y - point1.y));
        b = ((point2.z - point1.z) * (point3.x - point1.x)) - ((point2.x - point1.x) * (point3.z - point1.z));
        c = ((point2.x - point1.x) * (point3.y - point1.y)) - ((point2.y - point1.y) * (point3.x - point1.x));
        d = -(a * point1.x + b * point1.y + c * point1.z);

        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (inliers_set.count(i))
                continue;

            PointT testPoint;
            testPoint = cloud->points[i];

            dist = fabs((a * testPoint.x) + (b * testPoint.y) + (c * testPoint.z) + d) / sqrt((a * a) + (b * b) + (c * c));

            if (dist <= distanceThreshold)
                inliers_set.insert(i);
        }

        if (inliers_set.size() > inliersResult.size())
            inliersResult = inliers_set;
    }

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloud_road->points.push_back(point);
        else
            cloud_obstacle->points.push_back(point);
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloud_obstacle, cloud_road);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto &idx : it->indices)
            cloud_cluster->push_back((*cloud)[idx]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    return clusters;
}
template <typename PointT>
void ProcessPointClouds<PointT>::clusterrec(const typename pcl::PointCloud<PointT>::Ptr &cloud, int index, std::vector<int> &cluster, std::vector<bool> &processed, KdTree<PointT> *tree, float distanceTol)
{
    processed[index] = true;
    cluster.push_back(index);
    auto nearbyP = tree->search(cloud->points[index], distanceTol);
    for (int nearbyIndex : nearbyP)
    {
        if (processed[nearbyIndex] == false)
        {
            clusterrec(cloud, nearbyIndex, cluster, processed, tree, distanceTol);
        }
    }
}
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_Custom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    KdTree<PointT> *tree = new KdTree<PointT>;

    for (int i = 0; i < cloud->points.size(); i++)
        tree->insert(cloud->points[i], i);

    int it = 0;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //
    std::vector<std::vector<int>> cluster_indices;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(), false);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (processed[i] == false)
        {
            std::vector<int> cluster;
            clusterrec(cloud, i, cluster, processed, tree, clusterTolerance);
            cluster_indices.push_back(cluster);
        }
    }
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (std::vector<int> cluster : cluster_indices)
    {
        if (cluster.size() > minSize && cluster.size() < maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (std::vector<int>::const_iterator pit = cluster.begin(); pit != cluster.end(); ++pit)
                cloud_cluster->points.push_back(cloud->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
    }
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
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

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