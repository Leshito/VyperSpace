#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>


int planeSeg(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objects, float planeCoe[])
{

	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Get the plane model, if present.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
	segmentation.setInputCloud(cloud);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
	segmentation.segment(*planeIndices, *coefficients);

	planeCoe[0] = coefficients->values[0];
	planeCoe[1] = coefficients->values[1];
	planeCoe[2] = coefficients->values[2];
	planeCoe[3] = coefficients->values[3];

	if (planeIndices->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl;
	else
	{
		// Copy the points of the plane to a new cloud.
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(planeIndices);
		//extract.setNegative(true);
		extract.filter(*plane);
		
		// Retrieve the convex hull.
		pcl::ConvexHull<pcl::PointXYZRGBA> hull;
		hull.setInputCloud(plane);
		// Make sure that the resulting hull is bidimensional.
		hull.setDimension(2);
		hull.reconstruct(*convexHull);

		// Redundant check.
		if (hull.getDimension() == 2)
		{
			// Prism object.
			pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
			prism.setInputCloud(cloud);
			prism.setInputPlanarHull(convexHull);
			// First parameter: minimum Z value. Set to 0.01m, segments objects lying on the plane (can be negative).
			// Second parameter: maximum Z value, set to 1m. Tune it according to the height of the objects you expect.
			prism.setHeightLimits(0.01f, 1.0f);
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

			prism.segment(*objectIndices);

			// Get and show all points retrieved by the hull.
			extract.setIndices(objectIndices);
			extract.filter(*objects);
		}
		else std::cout << "The chosen hull is not planar." << std::endl;
	}
	
}