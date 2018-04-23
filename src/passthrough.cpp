
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int passthroughfilter (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.2);

  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName ("x");
  //trim                right left
  pass.setFilterLimits (-.15, .155);
  pass.filter (*cloud_filtered);


  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName ("y");
  //trim                bottom  top
  pass.setFilterLimits (-.3, .05);
  pass.filter (*cloud_filtered);


  return (0);
}
