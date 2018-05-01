#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


  //Read files
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
  }

  std::cout << "Loaded :\n"
            << argv[1]
            << "size: "
            << cloud->width * cloud->height
            << std::endl;
            

  std::cout << "/////////////// Starting Histogram Classifier ///////////////" << std::endl;

  // Step 1: find the minimum z and maximum z from cloud
  float min_z = 9999999999, max_z = -9999999999;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    if (cloud->points[i].z < min_z)
      min_z = cloud->points[i].z;
    if (cloud->points[i].z > max_z)
      max_z = cloud->points[i].z;
  }

  // to prevent border conditions, I expand the min and max by a small value
  min_z -= 0.01;
  max_z += 0.01;


  // Step 2: create histogram (5 intervals)
  float interval_size = (max_z - min_z)/10.0;
  int hist[10] = {0};
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    hist[int((cloud->points[i].z-min_z)/interval_size)] += 1;
  }

  // Print Hist
  std::cout << "Hist: \n";
  int i;
  for(i=0;i<10;i++){
    std::cout << hist[i] << " ";
  }
  std::cout << std::endl;

}