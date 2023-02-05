#include "nanoflann.hpp"
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace Eigen;
using namespace std;
using namespace std::chrono;
using namespace nanoflann;

const size_t dim = 3; 
using PC_type = Matrix<float, Eigen::Dynamic, dim>; 
using one_col = Matrix<float, Eigen::Dynamic, 1>; 

pcl::visualization::PCLVisualizer::Ptr ground_truth_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
                      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Ground Truth Visualization"));
  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer->addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 0, 255, 255); 
  viewer->addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");


  viewer->addCoordinateSystem (1.0, "cloud", 0);
  viewer->setBackgroundColor(0, 0, 0); // Setting background to a dark grey
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr view_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,const std::string& title)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (title));
  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer->addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 255, 255, 255); 
  viewer->addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");


  viewer->addCoordinateSystem (1.0, "cloud", 0);
  viewer->setBackgroundColor(0, 0, 0); // Setting background to a dark grey
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  return (viewer);
}

double MSE_icp(vector<float> distances)
{
  double error=0;

  for (int i=0;i<distances.size();i++)
  {
    error+=distances[i];
  }
  // std::cout<<err<<std::endl;
  error /= distances.size();
  return error;
}

PC_type icp(PC_type source_pc,PC_type transformed_pc,int max_iter, Eigen::Matrix4f initial_transformation,
            std::string output_file )
{

  auto start = high_resolution_clock::now();
  Eigen::Matrix3f rotation_gt;
  Eigen::Vector3f translation_gt;
  rotation_gt=initial_transformation.block<3,3>(0,0);
  translation_gt=initial_transformation.block<3,1>(0,3);
 
  using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor< PC_type, dim , nanoflann::metric_L2>; 
  my_kd_tree_t tree_index(3 ,std::cref(transformed_pc));
  tree_index.index->buildIndex();

  int iters_icp;
  double error;
  double prev_error=0.0;
  double tolerance = 0.00001;
  Eigen::Matrix3f Rot_total = Eigen::Matrix3f::Identity();
  Eigen::Vector3f trans_total = Eigen::Vector3f::Zero();
  for (iters_icp=0; iters_icp<max_iter;iters_icp++)
  {
    nanoflann::KNNResultSet<float> result(1);
    Eigen::Vector3f query;
    vector<float> distances;

    Eigen::MatrixXf ordered_pc = source_pc;
    for (int i = 0; i < source_pc.rows(); ++i) 
    {
      size_t closest_index;
      float closest_sqdist;
      // Query point:
      query= source_pc.row(i);
      result.init(& closest_index, & closest_sqdist);
      tree_index.index->findNeighbors(result, &query.x(),nanoflann::SearchParams());
      ordered_pc.row(i) = transformed_pc.row(closest_index);
      distances.push_back(closest_sqdist);
    }
   
    PC_type mean_src;
    PC_type mean_target;
    PC_type an;
    PC_type bn;
    PC_type H;
    PC_type rotation;
    one_col translation;
    mean_src.resize(1,Eigen::NoChange);
    mean_target.resize(1,Eigen::NoChange);
    H.resize(3,Eigen::NoChange);
    an.resize(source_pc.rows(),Eigen::NoChange);
    bn.resize(source_pc.rows(),Eigen::NoChange);
    rotation.resize(3,Eigen::NoChange);
    translation.resize(3,Eigen::NoChange);

    mean_src=source_pc.colwise().mean();
    mean_target=ordered_pc.colwise().mean();
    for(int i=0; i<source_pc.rows();i++)
    {
      an.row(i)=source_pc.row(i)-mean_src;
      bn.row(i)=ordered_pc.row(i)-mean_target;
    }
    
  
    H=an.transpose()*bn;
    JacobiSVD<PC_type> svd(H, ComputeFullU | ComputeFullV);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    rotation=V*U.transpose();
    translation=mean_target.transpose()- rotation*mean_src.transpose();
    
    Rot_total*=rotation;
    trans_total+=translation;

    Eigen::Matrix4f icp_transformation;
    icp_transformation.block<3,3>(0,0)=rotation;
    icp_transformation.block<3,1>(0,3)=translation;

    for(int i=0; i < source_pc.rows(); i++)
    {
      source_pc.row(i) = (rotation * source_pc.row(i).transpose() + translation).transpose();
    }
    
    error = MSE_icp(distances); 
    if (abs(prev_error - error) < tolerance) 
    {
      break;
    }
    prev_error = error;
  }
  auto stop = high_resolution_clock::now();

  duration<double> time =(stop - start);
  double time_seconds = time.count();
  std::cout << "-----------------------------------" << std::endl;
  std::cout << "--------------- ICP ---------------" << std::endl;
  std::cout << "Iterations of ICP: " << iters_icp << std::endl;
  std::cout << "Time = " <<time_seconds << " s" << std::endl;
  std::cout << "MSE_icp: " << error << std::endl;
  std::cout << "Rotation matrix: " << endl << Rot_total << std::endl;
  std::cout << "Translation matrix: " << trans_total << std::endl;
  std::cout << "rotation error: " << std::acos(((Rot_total.transpose() * rotation_gt).trace() - 1) / 2) * 180.0 / M_PI << std::endl;
  std::cout << "Translation error: " << std::sqrt((trans_total - translation_gt).squaredNorm()) << std::endl;
  
  //saving data in text file
  std::ofstream txt_file;
  // File Open
  txt_file.open(output_file +"_icp.txt");

  // Write to the file
  txt_file << "Iterations of ICP: " << iters_icp << std::endl;
  txt_file <<"Time = " <<time_seconds<< " s" << std::endl;
  txt_file <<"MSE_icp: " << error << std::endl;
  txt_file << "rotation error: " << std::acos(((Rot_total.transpose() * rotation_gt).trace() - 1) / 2) * 180.0 / M_PI<< std::endl;
  txt_file <<"Translation error: " << std::sqrt((trans_total - translation_gt).squaredNorm()) << std::endl;
  // File Close
  txt_file.close();

  return (source_pc);
}

double MSE_tricp (int number_of_points, std::vector<int> sorted_index, std::vector<float> sqdist)
 {
  double error = 0.;
  for (int i = 0; i < number_of_points; i++) {
      error += sqdist[sorted_index[i]];
  }
  error /= number_of_points;
  return error;
}

PC_type tricp(PC_type source_pc,PC_type transformed_pc,int max_iter, Eigen::Matrix4f initial_transformation,
              std::string output_file )
{
  auto start = high_resolution_clock::now();
  Eigen::Matrix3f rotation_gt;
  Eigen::Vector3f translation_gt;
  rotation_gt=initial_transformation.block<3,3>(0,0);
  translation_gt=initial_transformation.block<3,1>(0,3);
 
  using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor< PC_type, dim , nanoflann::metric_L2>; 
  my_kd_tree_t tree_index(3 ,std::cref(transformed_pc));
  tree_index.index->buildIndex();

  int iters_tricp;
  double error;
  double prev_error=0.0;
  double tolerance = 0.00001;
  float matching_factor=0.6;
  Eigen::Matrix3f Rot_total = Eigen::Matrix3f::Identity();
  Eigen::Vector3f trans_total = Eigen::Vector3f::Zero();
  for (iters_tricp=0; iters_tricp<max_iter;iters_tricp++)
  { // Find NN
    std::vector<size_t> indices;
    std::vector<float> sqdist;
    for (int i = 0; i < source_pc.rows(); i++)
    {
      // Query point:
      vector<float> query(3);
      for (int j = 0; j < 3; j++) 
      {
        query[j] = source_pc(i, j);
      }
      // do a knn search
      nanoflann::KNNResultSet<float> result(1);
      size_t closest_index;
      float closest_sqdist;
      result.init(&closest_index, &closest_sqdist);
      tree_index.index->findNeighbors(result, &query[0], nanoflann::SearchParams());
      indices.push_back(closest_index);
      sqdist.push_back(closest_sqdist);
    }
    std::vector<int> sorted_index(indices.size());
    std::iota(sorted_index.begin(), sorted_index.end(), 0);
    std::sort(sorted_index.begin(), sorted_index.end(),
    [&](int A, int B) -> bool {return sqdist[A] < sqdist[B];});
    int number_of_points=matching_factor*double(sorted_index.size());
    PC_type new_source_pc;
    PC_type new_transformed_pc;
    new_source_pc.resize(number_of_points,Eigen::NoChange);
    new_transformed_pc.resize(number_of_points,Eigen::NoChange);
    for (int i=0; i<number_of_points;i++)
    {
     new_source_pc.row(i)= source_pc.row(sorted_index[i]);
     new_transformed_pc.row(i)=transformed_pc.row(indices[sorted_index[i]]);
    }

    PC_type mean_src;
    PC_type mean_target;
    PC_type an;
    PC_type bn;
    PC_type H;
    PC_type rotation;
    one_col translation;
    mean_src.resize(1,Eigen::NoChange);
    mean_target.resize(1,Eigen::NoChange);
    H.resize(3,Eigen::NoChange);
    an.resize(new_source_pc.rows(),Eigen::NoChange);
    bn.resize(new_source_pc.rows(),Eigen::NoChange);
    rotation.resize(3,Eigen::NoChange);
    translation.resize(3,Eigen::NoChange);

    mean_src=new_source_pc.colwise().mean();
    mean_target=new_transformed_pc.colwise().mean();
    for(int i=0; i<new_source_pc.rows();i++)
    {
      an.row(i)=new_source_pc.row(i)-mean_src;
      bn.row(i)=new_transformed_pc.row(i)-mean_target;
    }
  
    H=an.transpose()*bn;
    JacobiSVD<PC_type> svd(H, ComputeFullU | ComputeFullV);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    rotation=V*U.transpose();
    translation=mean_target.transpose()- rotation*mean_src.transpose();
 
    Rot_total*=rotation;
    trans_total+=translation;

    for(int i=0; i < source_pc.rows(); i++)
    {
      source_pc.row(i) = (rotation * source_pc.row(i).transpose() + translation).transpose();
    }
    error = MSE_tricp(number_of_points, sorted_index, sqdist); 
     if (abs(prev_error - error) < tolerance) 
    {
      break;
    }
    prev_error = error;
  }
  auto stop = high_resolution_clock::now();
  duration<double> time =(stop - start);
  double time_seconds = time.count();
  std::cout << "-------------------------------------" << std::endl;
  std::cout << "--------------- TRICP ---------------" << std::endl;
  std::cout << "Iterations of tric: " << iters_tricp << std::endl;
  std::cout << "Time = " <<time_seconds << " s" << std::endl;
  std::cout << "MSE_tricp: " << error << std::endl;
  std::cout << "Rotation matrix: " << endl << Rot_total << std::endl;
  std::cout << "Translation matrix: " << trans_total << std::endl;
  std::cout << "Angular rotation error: " << std::acos(((Rot_total.transpose() * rotation_gt).trace() - 1) / 2) * 180.0 / M_PI << std::endl;
  std::cout << "Translation error: " << std::sqrt((trans_total - translation_gt).squaredNorm()) << std::endl;
  
  //saving data in text file
  std::ofstream txt_file;
  // File Open
  txt_file.open(output_file +"_tricp.txt");

  // Write to the file
  txt_file << "Iterations of TRICP: " << iters_tricp << std::endl;
  txt_file <<"Time = " <<time_seconds<< " s" << std::endl;
  txt_file <<"MSE_tricp: " << error << std::endl;
  txt_file << "rotation error: " << std::acos(((Rot_total.transpose() * rotation_gt).trace() - 1) / 2) * 180.0 / M_PI<< std::endl;
  txt_file <<"Translation error: " << std::sqrt((trans_total - translation_gt).squaredNorm()) << std::endl;
  // File Close
  txt_file.close();

  return (source_pc);
}

int main(int argc, char** argv) 
{
	if (argc < 7) {
    std::cerr << "Usage: " << argv[0] << " path_of_pc1 path_of_pc2 rotation_angle translation max_itererations output_filename" << std::endl;
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
 
  pcl::io::loadPCDFile(argv[1], *source_cloud);
  pcl::io::loadPCDFile(argv[2], *transformed_cloud);

  // Create an identity matrix
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  // Define a rotation matrix
  float theta = atof(argv[3])* M_PI / 180.0; // The angle of rotation in radians
  transformation (0,0) = std::cos(theta);
  transformation (0,1) = -sin(theta);
  transformation (1,0) = sin(theta);
  transformation (1,1) = std::cos(theta);
  // Define a translation of 2.5 meters on the x axis.
  transformation (0,3) = atof(argv[4]);
 
  int max_iter= atoi(argv[5]);
  std::string output_file = argv[6];


  // Executing the transformation
  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transformation); 

  PC_type source_pc;
  PC_type transformed_pc;
  source_pc.resize(source_cloud->size(),Eigen::NoChange);
  transformed_pc.resize(transformed_cloud->size(),Eigen::NoChange);

  for(int i = 0; i<source_pc.rows();++i)
  {
    source_pc.row(i) = Eigen::Vector3f(source_cloud->at(i).x,source_cloud->at(i).y,source_cloud->at(i).z);
  }

  for(int i = 0; i<transformed_pc.rows();++i)
  {
    transformed_pc.row(i) = Eigen::Vector3f(transformed_cloud->at(i).x,transformed_cloud->at(i).y
                                    ,transformed_cloud->at(i).z);
  }

  // Visualization for the point clouds
  pcl::visualization::PCLVisualizer::Ptr viewer_gt;
  viewer_gt = ground_truth_vis(source_cloud,transformed_cloud);

  // Executing ICP
  PC_type new_source_icp;
  new_source_icp.resize(source_pc.rows(),Eigen::NoChange);
  new_source_icp=icp(source_pc,transformed_pc,max_iter,transformation,output_file);
  // Convert the obtained matrix from ICP to point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_icp (new pcl::PointCloud<pcl::PointXYZ> ());
  for(int i=0; i < new_source_icp.rows(); i++){
    new_cloud_icp->push_back(pcl::PointXYZ(new_source_icp(i, 0), new_source_icp(i, 1), new_source_icp(i, 2)));
  }
  new_cloud_icp->width = new_cloud_icp->size (); new_cloud_icp->height = 1; new_cloud_icp->is_dense = true;

  // Visualization for ICP
  pcl::visualization::PCLVisualizer::Ptr viewer_icp;
  viewer_icp = view_vis(transformed_cloud,new_cloud_icp,"ICP Visulization");


  // Executing TRICP
  PC_type new_source_tricp;
  new_source_tricp.resize(source_pc.rows(),Eigen::NoChange);
  new_source_tricp=tricp(source_pc,transformed_pc,max_iter,transformation,output_file);
  // Convert the obtained matrix from TRICP to point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_tricp (new pcl::PointCloud<pcl::PointXYZ> ());
  for(int i=0; i < new_source_icp.rows(); i++){
    new_cloud_tricp->push_back(pcl::PointXYZ(new_source_tricp(i, 0), new_source_tricp(i, 1), new_source_tricp(i, 2)));
  }
  new_cloud_tricp->width = new_cloud_tricp->size (); new_cloud_tricp->height = 1; new_cloud_tricp->is_dense = true;

  // Visualization for TRICP 
  pcl::visualization::PCLVisualizer::Ptr viewer_tricp;
  viewer_tricp = view_vis(transformed_cloud,new_cloud_tricp,"TRICP Visulization");

 
  while (!viewer_tricp->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer_tricp->spinOnce ();
  }
  return 0;
} 