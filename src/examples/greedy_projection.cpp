#include <pcl/point_types.h>
#include <pcl/common/common.h>  //for getMinMax3D
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>  // for savePolygonFileSTL
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h> // for removeNaNFromPointCloud
#include <pcl/surface/mls.h>  //movingLeastSquares

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("../../Samples_PCD/Spundwand.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  std::cout << "Input: " << cloud->points.size() << " points" << std::endl;
  //* the data should be available in cloud
  
  
//   // Create a KD-Tree
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//   // Output has the PointNormal type in order to store the normals calculated by MLS
//   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointNormal>);
//   // Init object (second point type is for the normals, even if unused)
//   pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//   mls.setComputeNormals (true);
//   // Set parameters
//   mls.setInputCloud (cloud);
//   mls.setPolynomialOrder (3);
//   mls.setSearchMethod (tree);
//   mls.setSearchRadius (0.1);
//   // Reconstruct
//   mls.process (*cloud_smoothed);
  
  // MovingLeastSquares without computing normals and KdTree
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setComputeNormals (false);
  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (3);
  mls.setSearchRadius (0.1);
  // Reconstruct
  mls.process (*cloud_smoothed);
  pcl::io::savePCDFile ("GP1_cloud_smoothed.pcd", *cloud_smoothed);
  
  // Some informations about cloud_smoothed Cloud
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud_smoothed, minPt, maxPt); 
  std::cout << "Dimensions of cloud_smoothed (";
  std::cout << cloud_smoothed->points.size () << " data points):" << std::endl;
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
  
  
  // Statistical outlier removal
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud_smoothed);
  sor2.setMeanK (60);
  sor2.setStddevMulThresh (2);
  sor2.setNegative (false);
  sor2.filter (*cloud_filtered);
  std::cout << cloud_smoothed->points.size() - cloud_filtered->points.size () << " outliers where removed. ";
  std::cout << cloud_filtered->points.size() << " points left" << std::endl;
  pcl::io::savePCDFile ("GP2_cloud_filtered.pcd", *cloud_filtered); 

  
  // Remove NaNs from PointCloud
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, indices);
  std::cout << cloud_filtered->points.size() << " now points left" << std::endl; 
  
  
  // greedy projection needs Normals
  // Normal estimation*
  std::cout << "starting normal estimation for greedy projection" << std::endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
  tree2->setInputCloud (cloud_filtered);
  n.setInputCloud (cloud_filtered);
  n.setSearchMethod (tree2);
  n.setRadiusSearch (10);  //alternative to setKSerach()
  //n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud_filtered + normals

  
  std::cout << "starting greedy projection for greedy projection" << std::endl;
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
  tree3->setInputCloud (cloud_with_normals); //Input: cloud_with_normals

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.5);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);
  
  
  // Get result
  gp3.setInputCloud (cloud_with_normals);  //Input: cloud_with_normals
  gp3.setSearchMethod (tree3);
  mls.setSearchRadius (0.1);
  gp3.reconstruct (triangles);
  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  
  pcl::io::saveVTKFile ("GreedyProjectionMesh.vtk",triangles);
  return (0);
}
