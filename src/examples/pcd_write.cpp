#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 50000;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);


    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        
    if (0) //Sphere
    {
      cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      cloud.points[i].z =  - sqrt( 1 - (cloud.points[i].x * cloud.points[i].x)
                                        - (cloud.points[i].y * cloud.points[i].y));
    }
    else if (0) //Sphere in Würfel
    {
      cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)  //wenn i/5 ein rundes Ergebniss ist, dann..
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)
        cloud.points[i].z =  sqrt( 1 - (cloud.points[i].x * cloud.points[i].x)
                                      - (cloud.points[i].y * cloud.points[i].y));
      else
        cloud.points[i].z =  - sqrt( 1 - (cloud.points[i].x * cloud.points[i].x)
                                        - (cloud.points[i].y * cloud.points[i].y));
    }
    else if (0)  //Plane in Würfel
    {
      cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
        cloud.points[i].z = -1 * (cloud.points[i].x + cloud.points[i].y);
                                    - (cloud.points[i].y * cloud.points[i].y);
    }
    
        else if (1)  //Plane
    {
      cloud.points[i].x = 1024 * rand () / ((RAND_MAX + 1.0f)/0.01);
      cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

    }
    else  //Würfel
    {
    cloud.points[i].x =  rand () / ((RAND_MAX + 1.0f)/2);   //0.0 to 2
    cloud.points[i].y =  rand () / (RAND_MAX + 1.0f);       //0.0 to 1
    cloud.points[i].z =  rand () / (RAND_MAX + 1.0f);       //0.0 to 1
    }
    }

  
  pcl::io::savePCDFileASCII ("out.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
    if (i > 10) break;
  }
  return (0);
}
