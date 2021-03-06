#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    //Create lidar sensor
    Lidar* lidar_sensor = new Lidar(cars, 0); 

    // Get rays from scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr rays = lidar_sensor->scan();

    // Uncomment to Render Rays
    // renderRays(viewer, lidar_sensor->position,rays);
    // renderPointCloud(viewer, rays, "Point Cloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>();

    // separate clouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = point_processor->SegmentPlane(rays, 100, 0.2);
    // Uncomment to render point clouds
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters)
    {
        std::cout << "cluster size ";
        point_processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;

        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* point_processor_I, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

    // renderPointCloud(viewer, inputCloud, "inputCloud");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = point_processor_I->FilterCloud(inputCloud, 0.5f, Eigen::Vector4f(-5, -7, -2, 1), Eigen::Vector4f(20, 7, 60, 1));
    

    // Segment Into Roads and Obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = point_processor_I->SegmentPlaneProject(filterCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "segmentCloud");


    // Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = point_processor_I->ClusteringProject(segmentCloud.second, 0.8, 10, 300); // 0.8, 10, 300


    // Find bounding boxes for the clusters
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters)
    {
        std::cout << "cluster size ";
        point_processor_I->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;

        Box box = point_processor_I->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

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

    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* point_processor_I = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = point_processor_I->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud_I;

	while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloud_I = point_processor_I->loadPcd((*streamIterator).string());
        cityBlock(viewer, point_processor_I, inputCloud_I);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}
