#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

void getDimensions(
 	float plane[],
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
	float heightWidth[])
{
	pcl::PointXYZRGBA farthest;
	pcl::PointXYZRGBA nearest;
	float max = -9999, min = 9999;
	float a = plane[0];
	float b = plane[1];
	float c = plane[2];
	float d = plane[3];


	for(const pcl::PointXYZRGBA pt : cloud->points)
	{	if(!(std::isnan(pt.x)))
		{
			float dist = std::abs(a*pt.x + b*pt.y + c*pt.z + d)
	 				    /std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));

			if (dist < min)
			{
				min = dist;
				nearest = pt;
			}
			else if (dist > max)
			{
				max = dist;
				farthest = pt;
			}
		}		
	}


	heightWidth[0] = max - min;

	if(min - 0.01 <= 0)
	{
		heightWidth[0] += 0.01;
	}

	pcl::PointXYZRGBA mins;
	pcl::PointXYZRGBA maxs;

	pcl::getMinMax3D(*cloud, mins, maxs);

	heightWidth[1] = maxs.x - mins.x;

	
}

void getDimensionsTransform(
 	float plane[],
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
	float heightWidth[])
{
	float a = plane[0];
	float b = plane[1];
	float c = plane[2];
	float d = plane[3];

	Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;
	floor_plane_normal_vector[0] = a;
    floor_plane_normal_vector[1] = b;
    floor_plane_normal_vector[2] = c;

    xy_plane_normal_vector[0] = 0.0;
    xy_plane_normal_vector[1] = 0.0;
    xy_plane_normal_vector[2] = 1.0;

    rotation_vector = (xy_plane_normal_vector.cross(floor_plane_normal_vector)).normalized();

	float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));
    
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  	Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    transform_1.rotate (Eigen::AngleAxisf (theta, rotation_vector));
  	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);

  	pcl::PCDWriter writer;
  	writer.write<pcl::PointXYZRGBA> ("transform.pcd", *transformed_cloud, false);
	pcl::PointXYZRGBA mins;
	pcl::PointXYZRGBA maxs;

	pcl::getMinMax3D(*transformed_cloud, mins, maxs);
	heightWidth[0] = maxs.z - mins.z;
	heightWidth[1] = maxs.x - mins.x;


}
