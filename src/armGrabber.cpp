#include <iostream>
#include <Eigen/Geometry>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <boost/make_shared.hpp>
#include <stdio.h>
#include "voxel_grid.h"
#include "plane_seg.h"
#include "cluster_extraction.h"
#include "passthrough.h"
#include "shape_detection.h"
#include "correspondence_grouping.h"
#include <vector>
#include "outputPoints.h"

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
armGrabber(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn)
{

  int i = 0;
  const char *dir_path="/home/saul/Documents/PCL_workspace/VyperSpace/hand_clusters/";
  int numOfFiles=4;
  //std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > arm_clusters;
  std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > clusters;

  pcl::PointCloud<pcl::PointXYZRGBA>  tempCloud = pcl::PointCloud<pcl::PointXYZRGBA>(*cloudIn);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  cloud = tempCloud.makeShared();

  pcl::PCDWriter writer;

  if(cloud->size() > 0)
  {
    passthroughfilter(cloud, cloud);
  }
  else
  {
    printf("ArmGrabber: No data at passthrough filter.\n");
    return NULL;
  }

  if(cloud->size() > 0)
  {
    writer.write<pcl::PointXYZRGBA> ("arm_passthrough.pcd", *cloud, false);
    voxelfilter(cloud, cloud);

  }
  else
  {
    printf("ArmGrabber: No data at voxel filter.\n");
    return NULL;
  }


  float planeCoe[] = {0, 0, 0, 0};

  if(cloud->size() > 0)
  {
    writer.write<pcl::PointXYZRGBA> ("arm_voxel.pcd", *cloud, false);
    planeSeg(cloud, cloud, planeCoe);

  }
  else
  {
    printf("ArmGrabber: No data at plane segmentation.\n");
    return NULL;
  }

  if(cloud->size() > 0)
  {
    writer.write<pcl::PointXYZRGBA> ("arm_plane_seg.pcd", *cloud, false);
    clusterExtraction(cloud, &clusters);

    int s = clusters.size();

    if (s == 0)
    {
      printf("ArmGrabber: No clusters found.\n");
      return NULL;
    }
    else
      return clusters.front()->makeShared();



 }


}
