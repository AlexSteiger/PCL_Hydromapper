#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> //for getMinMax3D

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/surface/concave_hull.h>

// greedy projection:
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("../Samples_PCD/Spundwand.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud
  pcl::PCDWriter writer;
  writer.write ("1_Input_Cloud.pcd", *cloud, false);  
  
  // Some informations about input Cloud
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  std::cout << "Dimensions of Input (";
  std::cout << cloud->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;

  // std::cout << "\nIntervall for filtering: " << std::endl;
  // std::cout << "[ " << maxPt.z-2 << " | " << maxPt.z-1 << " ]" << std::endl;
  
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (maxPt.z-1.5, maxPt.z-1);   //only data points between get through
  pass.setFilterLimitsNegative (false);
  pass.filter (*cloud_filtered);
  writer.write ("2_Cloud_filtered.pcd", *cloud_filtered, false);    // save File
  
  // Some informations about filtered Cloud
  pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
  std::cout << "\nDimensions of filtered Cloud (";
  std::cout << cloud_filtered->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
 
  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  
  // Project
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  writer.write ("3_Cloud_projected.pcd", *cloud_projected, false);    //save File
 
  // Concave Hull
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.05);  //limits the size of the hull semements, smaller -> more Detail
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
  writer.write ("4_Concave_Hull.pcd", *cloud_hull, false);
  
  // greedy projection
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_projected);
  n.setInputCloud (cloud_projected);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_projected, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.1); //original 0.025

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (500); //original 100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Saving and viewing the result
  pcl::io::saveVTKFile("GreedyProjectionMesh.vtk", triangles);
  pcl::io::savePolygonFileSTL("GreedyProjectionMesh.stl", triangles);
  
  return (0);
}

