#ifndef CLUSTER_EXTRACTION_H
#define CLUSTER_EXTRACTIN_H 2018

int clusterExtraction(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > *clusters);

#endif
