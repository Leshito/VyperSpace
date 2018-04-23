#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include "shape_detection.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
typedef pcl::PointXYZRGBA PointT;

void shapeDetect(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	// Get the model, if present.
	//Cylinder
	//-Objects
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());

	//-Datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

	//-Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	//-Create segmentation object
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setNormalDistanceWeight(0.1);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.setModelType(pcl::SACMODEL_CYLINDER);

	//-Check if cylinder exists
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	if (inliers_cylinder->indices.size() != 0){
		std::cout << "Cylinder" << std::endl;
		return;
	}
/*
	//Sphere
	//-Set segmentation parameters
	seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);

	//-Check if sphere exists
	pcl::PointIndices::Ptr shapeIndices(new pcl::PointIndices);
	seg.segment(*shapeIndices, *coefficients_cylinder);
	if (shapeIndices->indices.size() != 0){
		std::cout << "Sphere" << std::endl;
		return;
	}
*/

	//Sphere
	//-Objects
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;

	//-Set segmentation parameters
	segmentation.setInputCloud(cloud);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	segmentation.setMaxIterations(1000);
	//segmentation.setRadiusLimits(0.1,0.15);

	//-Datasets
	pcl::PointIndices::Ptr shapeIndices(new pcl::PointIndices);

	//-Check if sphere exists
	segmentation.setModelType(pcl::SACMODEL_SPHERE);
	segmentation.segment(*shapeIndices, *coefficients);
	if (shapeIndices->indices.size() != 0){
		std::cout << "Sphere" << std::endl;
		return;
	}

/*
	//Cylinder model
	segmentation.setModelType(pcl::SACMODEL_CYLINDER);
	segmentation.segment(*shapeIndices, *coefficients);
	if (shapeIndices->indices.size() != 0){
		std::cout << "Cylinder" << std::endl;
		return;
	}
*/
	std::cout << "Undetermined" << std::endl;
}
