#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <string>
#include "shape_detection.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

typedef pcl::PointXYZRGBA PointT;

void shapeDetect(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	std::string shape = "Undetermined";

	std::vector<int> inliersSph;
	/*std::vector<int> inliersCyl;

        // created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
	  model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (cloud));

	//Sphere
	pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransacSph (model_s);
	ransacSph.setDistanceThreshold (.01);
	ransacSph.computeModel();
	ransacSph.getInliers(inliersSph);

	if(inliersSph.size() != 0)
		//shape = "Sphere";
	std::cout << "Sphere inliers (ransac): " << inliersSph.size() << std::endl;*/
	
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
		//shape = "Sphere";
		//std::cout << "Sphere inliers (segment): " << shapeIndices->indices.size() << std::endl;
		//return;
	}

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
	seg.setDistanceThreshold(0.12);//originally 0.01 //was 0.1, maybe too much now?
	seg.setNormalDistanceWeight(0.1);
	seg.setRadiusLimits(0, 0.5);//originally 0.1
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.setModelType(pcl::SACMODEL_CYLINDER);

	//-Check if cylinder exists
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	if (inliers_cylinder->indices.size() != 0){
		//shape = "Cylinder";
		//std::cout << "Cylinder inliers (segment): " << inliers_cylinder->indices.size()<< std::endl;
		//return;
	}

//compare which model has more inliers, if either
	if(shapeIndices->indices.size() > inliers_cylinder->indices.size() || shapeIndices->indices.size() == inliers_cylinder->indices.size())
		shape = "Sphere";
	else if(inliers_cylinder->indices.size() > inliersSph.size())
		shape = "Cylinder";
	




        //
        //std::cout << "Sphere Inliers: " << inliersSph.size() << std::endl;
        ////std::cout << "Cylinder Inliers: " << inliersCyl.size() << std::endl;
        //

	std::cout << shape << std::endl;
	//std::cout << "Undetermined" << std::endl;
}
