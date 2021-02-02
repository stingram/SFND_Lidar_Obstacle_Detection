/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
	int ind1, ind2, ind3;
	float A, B, C, D;
	float dist;
	pcl::PointXYZ point, p1, p2, p3;

	std::vector<int> temp_inliers;
	std::vector<int> max_inliers;

	// for random index selection
	int min = 0;
	int max = cloud->size() - 1;

	// For max iterations 
	for(int k=0;k<maxIterations;k++)
	{
		std::cout << "Iteration: " << k << "\n";
		temp_inliers.clear();
		// Randomly sample subset and fit line

		while(true)
		{
			ind1 = rand() % (max - min + 1) + min;
			ind2 = rand() % (max - min + 1) + min;
			ind3 = rand() % (max - min + 1) + min;
			if(ind1 != ind2 && ind1 != ind3 && ind2 != ind3)
			{
				break;
			}
		}

		p1 = cloud->at(ind1);
		p2 = cloud->at(ind2);
		p3 = cloud->at(ind3);

		// A = p1.x - p2.y;
		// B = p2.x - p1.x;
		// C = p1.x*p2.y - p2.x*p1.y;

		A = (p2.y-p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		B = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		C = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
		D = -1*(A*p1.x+B*p1.y+C*p1.z);

		std::cout << "A: " << A << ", B: " << B << ", C: " << C << ", D: " << D << "\n";

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(int i=0;i<cloud->size();i++)
		{
			point = cloud->at(i);
			// dist = std::abs(A*point.x + B*point.y + C)/std::sqrt(A*A + B*B + 0.000000001);
			dist = std::abs(A*point.x + B*point.y + C*point.z + D)/std::sqrt(A*A + B*B + C*C + 0.000000001);
			if(dist < distanceTol)
			{
				temp_inliers.push_back(i);
			}
		}
		// Check if the number of inliers from this line is greater than current max
		if(temp_inliers.size()>max_inliers.size())
		{
			// copy temp_inliers to max_inliers
			//max_inliers.clear();
			max_inliers.assign(temp_inliers.begin(), temp_inliers.end());
		}
	
	}
	// Return indicies of inliers from fitted line with most inliers
	for(const int i : max_inliers)
	{
		inliersResult.insert(i);
	}
	std::cout << "Num inliers: " << inliersResult.size() << "\n";
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.2);

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
