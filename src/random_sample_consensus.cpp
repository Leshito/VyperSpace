#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "random_sample_consensus.h"
#include "shape_detection.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int ransac(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr finalCloud, char ** argv, int argc)
{
  //retrieve arguments
  char* pcdFile=argv[2];

  std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA> (cloud));
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (cloud));
  if(pcl::console::find_argument (argc, argv, "-f") >= 0)
  {
    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  {
    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud, inliers, *finalCloud);
  //Detect shape of object in scene
  shapeDetect(finalCloud);

  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }*/

    //pcl::PCDWriter writer;
    //writer.write ("/home/luish/Schoolz/SD/voxelTest/table_scene_lms400_downsampled.pcd", *cloudFiltered,
    //      false);

    
    /*
    writer.write ("/home/dylan/PCL_TEST_GROUP/3/Project/vvProject/finalCloud.pcd", *finalCloud,
          false);
    */



  return 0;
 }
