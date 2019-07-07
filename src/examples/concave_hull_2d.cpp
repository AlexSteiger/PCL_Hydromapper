// http://pointclouds.org/documentation/tutorials/hull_2d.php
// calculate a simple 2D hull polygon (concave or convex) for a set of points supported by a plane.

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

//   reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
//   // Build a filter to remove spurious NaNs
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pass.setInputCloud (cloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0, 1.1);
//   pass.filter (*cloud_filtered);
//   std::cerr << "PointCloud after filtering has: "
//             << cloud_filtered->points.size () << " data points." << std::endl;
// 
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   std::cout << "1" << std::endl;
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   // Create the segmentation object
//   std::cout << "2" << std::endl;
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   // Optional
//   std::cout << "3" << std::endl;
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   std::cout << "4" << std::endl;
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setDistanceThreshold (0.1);
//   std::cout << "5" << std::endl;
//   seg.setInputCloud (cloud_filtered);
//   std::cout << "6" << std::endl;
//   seg.segment (*inliers, *coefficients);
//   std::cout << "7" << std::endl;
//   std::cerr << "PointCloud after segmentation has: "
//             << inliers->indices.size () << " inliers." << std::endl;
//   
//             
//   // Project the model inliers
//   pcl::ProjectInliers<pcl::PointXYZ> proj;
//   proj.setModelType (pcl::SACMODEL_PLANE);
//   proj.setIndices (inliers);
//   proj.setInputCloud (cloud_filtered);
//   proj.setModelCoefficients (coefficients);
//   proj.filter (*cloud_projected);
//   std::cerr << "PointCloud after projection has: "
//             << cloud_projected->points.size () << " data points." << std::endl;
//   pcl::PCDWriter writer;
//   writer.write ("proj.pcd", *cloud_projected, false);
//             
            
  reader.read ("../../Samples_PCD/quader_dense.pcd", *cloud_projected);
  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.07);  //limits the size of the hull semements, smaller -> more Detail
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("Concave_Hull_2d.pcd", *cloud_hull, false);

  return (0);
}
