#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/point_cloud_conversion.h> 
#include "typeinfo"
#include <cmath>


/**
 * OBJECT DETECT CALLBACK
 */
void objectDetectCallback(const sensor_msgs::PointCloud2& msg)
{

  // GET THE POINT CLOUD
  sensor_msgs::PointCloud point_cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(msg, point_cloud);

  // std::cout << "SIZE" << point_cloud.points.size() << std::endl;
  // 144000 - 

  
  // FILTER POINTS
  for(int i = 0; i < point_cloud.points.size(); i++){
    if( (isinf(point_cloud.points[i].x) || isinf(point_cloud.points[i].y) || isinf(point_cloud.points[i].z)) != true){
      std::cout << point_cloud.points[i] << std::endl;
    }
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