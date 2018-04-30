#ifndef __ARMGRABBER
#define __ARMGRABBER 2018

//int armGrabber(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr *arm);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr armGrabber(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn );
#endif
