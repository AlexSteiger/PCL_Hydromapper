
#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <vector>


int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;

  reader.read ("../../Samples_PCD/KugelInWuerfel.pcd", *cloud);
  std::cout << "Input has: " << cloud->points.size () << " data points." << std::endl;

  std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
  model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

  
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);

  ransac.setDistanceThreshold (0.1);
  ransac.computeModel();
  ransac.getInliers(inliers);

  
  //copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
  std::cout << "Output has: " << final->points.size () << " data points." << std::endl;
  
   pcl::PCDWriter writer;
   writer.write ("random_sample_consensus_sphere_out.pcd", *final, false);

    return 0;
}
