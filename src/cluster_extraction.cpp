#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "cluster_extraction.h"
#include <fstream>
#include <string>
#include <iostream>



int
clusterExtraction(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                  std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > *clusters)
{


  pcl::PCDWriter writer;

  std::string line;
  std::string tok;
  char *tokens;
  int boxParams[2] = {0, 0};

  std::ifstream inconfig;
  inconfig.open("configuration.txt");
  if (inconfig.is_open())
  {
    getline(inconfig, line);
    getline(inconfig, line);
    getline(inconfig, line);
    getline(inconfig, line);
    getline(inconfig, line);
    getline(inconfig, line);
    std::cout << line << std::endl;
    std::stringstream stream(line);

    int it=0;
    while (stream)
    {
      stream >> boxParams[it];
      it++;
    }
    inconfig.close();
  }
  //boxParams contents
	// [0]  [1]
	//minSize maxSize

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance (0.005); // 1cm

  ec.setMinClusterSize (boxParams[0]);
  ec.setMaxClusterSize (boxParams[1]);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int numOfClusters = cluster_indices.size();

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();

    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::stringstream ss;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_cluster, false);
    clusters->push_back(cloud_cluster);
    j++;
  }

  return (0);
}
