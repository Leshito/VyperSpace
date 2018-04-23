#ifndef __RANSAC
#define __RANSAC 2018

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);

int ransac(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final, char ** argv, int argc);
#endif
