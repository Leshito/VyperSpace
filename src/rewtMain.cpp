#include <iostream>
#include <Eigen/Geometry>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
#include "voxel_grid.h"
#include "plane_seg.h"
#include "cluster_extraction.h"
#include "passthrough.h"
#include "shape_detection.h"
#include "correspondence_grouping.h"
#include <vector>
#include "outputPoints.h"

int rewtMain(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn)
{

  int i = 0;
  const char *dir_path="/home/luish/Schoolz/SD/VyperSpace/hand_clusters/";
  int numOfFiles=4;
  std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > arm_clusters;
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
    printf("No data at passthrough filter.\n");
    return 0;
  }

  if(cloud->size() > 0)
  {
    writer.write<pcl::PointXYZRGBA> ("passthrough.pcd", *cloud, false);
    voxelfilter(cloud, cloud);
    
  }
  else
  {
    printf("No data at voxel filter.\n");
    return 0;
  }


  float planeCoe[] = {0, 0, 0, 0};

  if(cloud->size() > 0)
  {
    writer.write<pcl::PointXYZRGBA> ("voxel.pcd", *cloud, false);
    planeSeg(cloud, cloud, planeCoe);
    
  }
  else
  {
    printf("No data at plane segmentation.\n");
    return 0;
  }
  
  if(cloud->size() > 0)
  {
    writer.write<pcl::PointXYZRGBA> ("plane_seg.pcd", *cloud, false);
    clusterExtraction(cloud, &clusters);

    int s = clusters.size();

    if (s == 0)
    {
      printf("No clusters found.\n");
      return 0;
    }

    /*printf("Saul start\n");
    int flag=0;

    for(int j=0; j<s;j++)
    {
      for(int i=0; i<numOfFiles;i++)
      {
        if(findModel(arm_clusters[i], clusters[j])==1)
        {
          clusters.erase(clusters.begin()+j);
          flag=1;
          break;
        }
      }
      if(flag==1)
        break;
    }

    printf("Saul end\n");*/

  }
  else
  {
    printf("No data at cluster extraction.\n");
    return 0;
  }
 
  int s = clusters.size();

  if (s == 0)
  {
    printf("No clusters to process.\n");
    return 0;
  }

  shapeDetect(clusters[0]);
 
  float heightWidth[] = {0, 0};
  
  printf("Via Distance From Plane\n");
  getDimensions(planeCoe, clusters[0], heightWidth);
  printf("height: %f inches, width: %f inches\n", 39.37007874*heightWidth[0], 39.37007874*heightWidth[1]);

  printf("Via Transform\n");
  getDimensionsTransform(planeCoe, clusters[0], heightWidth);
  printf("height: %f inches, width: %f inches\n", 39.37007874*heightWidth[0], 39.37007874*heightWidth[1]);

}