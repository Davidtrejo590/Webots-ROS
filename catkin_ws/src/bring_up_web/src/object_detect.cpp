#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * OBJECT DETECT CALLBACK
 */
void objectDetectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // GET EACH POINT IN THE POINT CLOUD
  void* p = (void*)(&msg-> data[0]);
  for(size_t i = 0; i < msg->width*msg->height; i++){
    float x = *((float*)(p + 0));
    float y = *((float*)(p + 4));
    float z = *((float*)(p + 8));
    p += msg -> point_step;

    std::cout << x << "-" << y << "-" << z << "-" << std::endl;
  }

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



// // GET THE POINT CLOUD
//   sensor_msgs::PointCloud point_cloud;
//   sensor_msgs::convertPointCloud2ToPointCloud(msg, point_cloud);

//   // std::cout << "SIZE" << point_cloud.points.size() << std::endl;
//   // 144000 - 

  
//   // FILTER POINTS
//   for(int i = 0; i < point_cloud.points.size(); i++){
//     if( (isinf(point_cloud.points[i].x) || isinf(point_cloud.points[i].y) || isinf(point_cloud.points[i].z)) != true){
//       std::cout << point_cloud.points[i] << std::endl;
//     }
//   }