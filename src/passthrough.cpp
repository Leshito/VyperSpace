
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <fstream>
#include <string>

int passthroughfilter (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
  std::string line;
  std::string tok;
  char *tokens;
  float boxParams[6] = {0, 0, 0, 0, 0, 0};

  std::ifstream inconfig;
  inconfig.open("configuration.txt");
  if (inconfig.is_open())
  {
    getline(inconfig, line);
    getline(inconfig, line);
    getline(inconfig, line);
    //std::cout << line << std::endl;
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
	// [0]  [1] [2]  [3]  [4]  [5]
	//xmax xmin ymax ymin zmax zmin


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (boxParams[5], boxParams[4]);

  pass.filter (*cloud_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName ("x");
  //trim                right left
  pass.setFilterLimits (boxParams[1], boxParams[0]);
  pass.filter (*cloud_filtered);


  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName ("y");
  //trim                bottom  top
  pass.setFilterLimits (boxParams[3], boxParams[2]);
  pass.filter (*cloud_filtered);


  return (0);
}
