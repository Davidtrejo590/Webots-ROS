#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>


/**
 * OBJECT DETECT CALLBACK
 */
void objectDetectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  // CREATE A NEW POINT CLOUD FROM (PCL)
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 2000;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  int j = 0;
  void* p = (void*)(&msg-> data[0]);                                            // POINTER TO FIRST DATA IN THE POINT CLOUD 2


  // WALKING THROUGH THE POINT CLOUD
  for(size_t i = 0; i < msg->width * msg->height; i++){
    float x = *((float*)(p + 0));
    float y = *((float*)(p + 4));
    float z = *((float*)(p + 8));
    p += msg -> point_step;

    // FILTERING DATA & FILLING NEW POINT CLOUD
    if( (isinf(x) or isinf(y) or isinf(z)) != true){
      if ( (x > 1.0 or x < -1.0) and (y > -1.5) and (z > 2.0 or z < -2.0)){
        cloud.points[j].x = x;
        cloud.points[j].y = y;
        cloud.points[j].z = z;
        j++;
      }
    } 
  }

  // K MEANS
  // pcl::Kmeans real(cloud.points.size(), 3);
  // real.setClusterSize(3);
  // for (size_t i = 0; i < cloud.points.size(); i++){
  //   std::vector<float> data(3);
  //   data[0] = cloud.points[i].x;
  //   data[1] = cloud.points[i].y;
  //   data[2] = cloud.points[i].z;
  //   real.addDataPoint(data);
  // }

  // real.kMeans();
  // pcl::Kmeans::Centroids centroids = real.get_centroids();
  // std::cout << "-----START-----" << std::endl;
  // for (size_t i = 0; i < centroids.size(); i++){
  //   // CENTROIDS
  //   std::cout << "x: " << centroids[i][0] << "  y: " << centroids[i][1] << "  z: " << centroids[i][2] << std::endl;
  // }
  // std::cout << "-----END-----" << std::endl;

}

/*   
 * MAIN FUNCTION
 */
int main(int argc, char **argv)
{
  std::cout << "OBJECT DETECT NODE..." << std::endl;
  ros::init(argc, argv, "object_detect");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/point_cloud", 10, objectDetectCallback);
  ros::spin();

  return 0;
}

