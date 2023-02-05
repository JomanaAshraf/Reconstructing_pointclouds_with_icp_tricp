#include "../nanoflann.hpp"
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Ground Truth Visualization"));
  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer->addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

  viewer->setBackgroundColor(0, 0, 0); // Setting background to a dark grey
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  return (viewer);
}

int main() 
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::io::loadPCDFile ("../build/fountain_pc2.pcd", *source_cloud); 
  pcl::visualization::PCLVisualizer::Ptr viewer_tricp;
  viewer_tricp = rgbVis(source_cloud);
  while (!viewer_tricp->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer_tricp->spinOnce ();
  }
  return 0;
} 