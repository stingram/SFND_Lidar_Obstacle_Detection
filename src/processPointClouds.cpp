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

    // Voxel grid point reduction and region based filtering
    typename pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_box(new pcl::PointCloud<PointT>());
    
   
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    // Crop box
    typename pcl::CropBox<PointT>::Ptr crop_box(new pcl::CropBox<PointT>());
    crop_box->setMin(minPoint);
    crop_box->setMax(maxPoint);
    crop_box->setInputCloud(cloud_filtered);
    crop_box->filter(*cloud_box);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_box;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obst_cloud(new pcl::PointCloud<PointT> ());  
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT> ());


    pcl::ExtractIndices<PointT> extract;

    

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    extract.setNegative(true);
    extract.filter(*obst_cloud);

    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // Find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    
    // Optional
    seg.setOptimizeCoefficients (true);
    
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacProject(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	int ind1, ind2, ind3;
	float A, B, C, D;
	float dist;
	PointT point, p1, p2, p3;

	std::vector<int> temp_inliers;
	std::vector<int> max_inliers;

	// for random index selection
	int min = 0;
	int max = cloud->size() - 1;

	// For max iterations 
	for(int k=0;k<maxIterations;k++)
	{
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

		A = (p2.y-p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		B = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		C = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
		D = -1*(A*p1.x+B*p1.y+C*p1.z);

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		for(int i=0;i<cloud->size();i++)
		{
			point = cloud->at(i);
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
			max_inliers.assign(temp_inliers.begin(), temp_inliers.end());
		}
	
	}
	// Return indicies of inliers from fitted line with most inliers
	for(const int i : max_inliers)
	{
		inliersResult.insert(i);
	}
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneProject(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliers = RansacProject(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new typename pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudInliers,cloudOutliers);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Use cluster indices to build vector of point clouds
    // cluster_indices is a vector of point_indices
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->push_back((*cloud)[*pit]);
        }
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximityProject(const typename pcl::PointCloud<PointT>::Ptr cloud, int id, typename pcl::PointCloud<PointT>::Ptr cluster,
					std::map<int, bool>& visited, KdTree<PointT>*& tree, float distanceTol)
{
	if(!visited[id])
	{
		visited[id] = true;
		// cluster->indices.push_back(id);
		// PointT point = cloud->at(id);
        PointT point = cloud->at(id);
        cluster->push_back(point); 

		pcl::PointIndices::Ptr nearby = tree->search(point, distanceTol);

		for(std::vector<int>::const_iterator it = nearby->indices.begin(); it != nearby->indices.end(); ++it)
		{
			proximityProject(cloud, *it, cluster, visited, tree, distanceTol);
		}

	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanClusterProject(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::map<int, bool> visited;

	for(int i=0; i<cloud->size(); i++)
	{
		if(!visited[i])
		{
			// create cluster
			typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
			proximityProject(cloud, i, cluster, visited, tree, distanceTol);
			if(cluster->size()>=5)
				clusters.push_back(cluster);
		}
	}
 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringProject(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT>* tree = new KdTree<PointT>;
   
    for (int i=0; i < cloud->size(); i++) 
    	tree->insert(cloud->at(i),i); 
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cluster_indices = euclideanClusterProject(cloud, tree, clusterTolerance);

    return cluster_indices;
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
