#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>  // for savePolygonFileSTL
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>  //movingLeastSquares

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("../../Samples_PCD/Spundwand.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud
  
    // Statistical outlier removal
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (60);
  sor.setStddevMulThresh (2);
  sor.setNegative (false);
  sor.filter (*cloud_filtered);
  std::cout << cloud.points.size() - cloud_filtered->points.size () << " outliers where removed. ";
  std::cout << cloud_filtered->points.size() << " points left" << std::endl;
  
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;
  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  // Set parameters
  mls.setInputCloud (cloud_filtered);
  mls.setPolynomialOrder (3);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  // Reconstruct
  mls.process (mls_points);
  pcl::io::savePCDFile ("MLS_out.pcd", mls_points);  

//   // Normal estimation*
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
//   tree2->setInputCloud (cloud_filtered);
//   n.setInputCloud (cloud_filtered);
//   n.setSearchMethod (tree2);
//   n.setKSearch (20); //original 20
//   n.compute (*normals);
//   //* normals should not contain the point normals + surface curvatures
// 
//   // Concatenate the XYZ and normal fields*
//   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//   pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
//   //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
  tree3->setInputCloud (mls_points);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (1); //original 0.025

  // Set typical values for the parameters   M_PI/4: 45° | M_PI/18: 10° | 2*M_PI/3: 120°
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100); //original 100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (mls_points);
  gp3.setSearchMethod (tree3);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Saving and viewing the result
  pcl::io::saveVTKFile("GreedyProjectionMesh.vtk", triangles);
  pcl::io::savePolygonFileSTL("GreedyProjectionMesh.stl", triangles);
  // Finish
  return (0);
}
