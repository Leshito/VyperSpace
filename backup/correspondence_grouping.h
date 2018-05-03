#ifndef __CORRESPONDENCE_GROUPING
#define __CORRESPONDENCE_GROUPING 2018

typedef pcl::PointXYZRGBA PointType;
void showHelp (char *filename);
void parseCommandLine (int argc, char *argv[]);
double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud);
int findModel (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene  );
#endif
