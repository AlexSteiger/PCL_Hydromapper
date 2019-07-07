#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> //for getMinMax3D

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("../../Samples_PCD/Spundwand.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  std::cout << "Input: " << cloud->points.size () << " data points." << std::endl;
  //* the data should be available in cloud

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  std::cout << "Min x: " << minPt.x << " | Max x: " << maxPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << " | Max y: " << maxPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << " | Max z: " << maxPt.z << std::endl;
  
  std::cout << "Intervall for filtering: " << std::endl;
  std::cout << "[ " << maxPt.z-2 << " | " << maxPt.z-1 << " ]" << std::endl;
  
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (maxPt.z-1.1, maxPt.z-1);   //only data points between get through
  pass.setFilterLimitsNegative (false);
  pass.filter (*cloud_filtered);
  
  std::cout << "Output: " << cloud_filtered->points.size () << " data points." << std::endl;
  pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
  
 
  
  //save File
  pcl::PCDWriter writer;
  writer.write ("Filtered_Pass_Through_Filter.pcd", *cloud_filtered, false);

 
  return (0);
}
