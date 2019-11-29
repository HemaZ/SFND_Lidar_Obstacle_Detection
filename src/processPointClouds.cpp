// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/filters/extract_indices.h>

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
    typename pcl::PointCloud<PointT>::Ptr fcloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vgrid;
    vgrid.setInputCloud(cloud);
    vgrid.setLeafSize(filterRes, filterRes, filterRes);
    vgrid.filter(*fcloud);

    typename pcl::PointCloud<PointT>::Ptr filterRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(fcloud);
    region.filter(*filterRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filterRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    for(int i:inliers->indices){
        planeCloud->points.push_back(cloud->points[i]);
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	while(maxIterations--){
		std::unordered_set<int> inliers;
		while(inliers.size()<3){
			// p2rand = rand()%cloud->points.size();
			inliers.insert(rand()%cloud->points.size());
		}
		auto itr = inliers.begin();
		float x1, y1, x2, y2, x3, y3, z1, z2, z3;
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		float i, j, k;
		i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float A, B, C, D;

		A = i;
		B = j;
		C = k;
		D = -(i*x1+j*y1+k*z1);

		for(int index = 0; index<cloud->size(); index++){
			float x, y, z;
			if (inliers.count(index)>0)
				continue;
			x = cloud->points[index].x;
			y = cloud->points[index].y;
			z = cloud->points[index].z;
			float d = fabs(A*x+B*y+C*z+D) / sqrt(A*A+ B*B+C*C);
			if (d<=distanceTol){
				inliers.insert(index);
			}
		}
		if(inliers.size()>inliersResult.size()){
			inliersResult = inliers;
		}
	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    // pcl::SACSegmentation<PointT> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (distanceThreshold);
    // seg.setMaxIterations(maxIterations);
    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);

    std::unordered_set<int> inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    if (inliers.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

void Prox(int pidx, const std::vector<std::vector<float>>& points , std::vector<int> &cluster, KdTree* tree, std::unordered_set<int> &processed, float distanceTol){
	processed.insert(pidx);
	cluster.push_back(pidx);
	std::vector<int> nearby = tree->search(points[pidx], distanceTol);
	for(int idx: nearby){
		if(processed.count(idx)==0){
			Prox(idx, points, cluster, tree, processed, distanceTol);
		}
	}

}


std::vector<std::vector<int>> eCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::unordered_set<int> processed;
	std::vector<std::vector<int>> clusters;
	for(int i =0; i<points.size(); i++){
		if(processed.count(i)!=0){
			continue;
		}
		std::vector<int> cluster;
		Prox(i, points, cluster, tree, processed, distanceTol);
		
		clusters.push_back(cluster);

	}
	
	
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterscloud;
    std::vector<std::vector<float>> points;
    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->size(); i++){
        PointT point = cloud->points[i];
        std::vector<float> fpoint = {point.x, point.y, point.z};
        tree->insert(fpoint,i); 
        points.push_back(fpoint);
    } 
    std::vector<std::vector<int>> clusters = eCluster(points, tree, clusterTolerance);

    int clusterId = 0;
    for(std::vector<int> cluster : clusters)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice: cluster){
            PointT point;
            point.x = points[indice][0];
            point.y = points[indice][1];
            point.z = points[indice][2];
  			clusterCloud->points.push_back(point);

          }
  		clusterscloud.push_back(clusterCloud);
  		++clusterId;
  	}




    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (cloud);
    
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance); // 2cm
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(cluster_indices);
    // for(pcl::PointIndices getIndices: cluster_indices){
    //     typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    //     for(int index: getIndices.indices){
    //         cluster ->points.push_back(cloud->points[index]);
    //     }
    //     cluster->width = cluster->points.size();
    //     cluster->height = 1;
    //     cluster -> is_dense = true;
    //     clusters.push_back(cluster);
    // }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusterscloud;
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