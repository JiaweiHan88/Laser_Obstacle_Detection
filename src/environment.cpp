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

    // TODO:: Create lidar sensor
    auto lidar = new Lidar(cars, 0.0);
    auto lidarpcl = lidar->scan();
    //renderRays(viewer, lidar->position, lidarpcl);
    //renderPointCloud(viewer, lidarpcl, "ray");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processor.SegmentPlane(lidarpcl, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor.Clustering(segmentCloud.first, 1.0, 3, 100);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (auto cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        Box box = processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7.3, 1, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane_Custom(filterCloud, 30, 0.15);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_Custom(segmentCloud.first, 0.4, 5, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(0, 0, 1)};
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    for (auto cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 2]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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
std::vector<boost::filesystem::path>::iterator streamIterator;
std::vector<boost::filesystem::path> stream;
ProcessPointClouds<pcl::PointXYZI> *pointProcessorI;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    if (event.getKeySym() == "n" && event.keyDown())
    {
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
    }

    if (event.getKeySym() == "p" && event.keyDown())
    {
        if (streamIterator == stream.begin())
            streamIterator = stream.end();
        streamIterator--;
    }
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *)&viewer);
    //simpleHighway(viewer);
    pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    viewer->spin();
    // while (!viewer->wasStopped())
    // {

    //     // Clear viewer
    //     viewer->removeAllPointClouds();
    //     viewer->removeAllShapes();

    //     // Load pcd and run obstacle detection process
    //     inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    //     cityBlock(viewer, pointProcessorI, inputCloudI);

    //     streamIterator++;
    //     if (streamIterator == stream.end())
    //         streamIterator = stream.begin();

    //     viewer->spinOnce();
    // }
}