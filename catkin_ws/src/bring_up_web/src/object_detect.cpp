#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <fstream>




/**
 * OBJECT DETECT CALLBACK
 */
void objectDetectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

  // FILE
  // std::ofstream pc;
  // pc.open("output.csv");
  // pc << "x,y,z" << std::endl;


  // int k = 3;
  // srand(time(NULL));

  // // GENERATE INITIAL CENTROIDS
  // std::vector<std::vector<double>> centroids;
  // // NUMBER OF CENTROIDS ( K )
  // for(int i = 0; i < k; i++){
  //   std::vector<double> points;
  //   for(int j = 0; j < 3; j++){
  //     points.push_back(rand());
  //   }

  //   centroids.push_back(points);
  // }


  // // DISPLAYING CENTROIDS
  // std::cout << "-----START-----" << std::endl;
  // for(int i = 0; i < centroids.size(); i++){
  //   for(int j = 0; j < centroids[i].size(); j++){
  //     std::cout << centroids[i][j] << " ";
  //   }
  //   std::cout << std::endl;

  // }
  // std::cout << "-----END-----" << std::endl;
  


  int points = 0;
  void* p = (void*)(&msg-> data[0]);                                            // POINTER TO FIRST DATA IN THE POINT CLOUD 2
  // WALKING THROUGH THE POINT CLOUD
  for(size_t i = 0; i < msg->width * msg->height; i++){
    float x = *((float*)(p + 0));
    float y = *((float*)(p + 4));
    float z = *((float*)(p + 8));
    p += msg -> point_step;

    // FILTER POINT CLOUD
    if( (isinf(x) or isinf(y) or isinf(z)) != true){
      if ( (x > 1.0 or x < -1.0) and (y > -1.5) and (z > 2.0 or z < -2.0)){
        // std::cout << x << " " << y << " " << z << std::endl;
        points++;
        // pc << x << "," << y << "," << z << std::endl;

        // APPLY K MEANS ALGORITH TO EACH POINT


      }
    } 
  }

  // pc.close();

  // std::cout << "POINT CLOUD SIZE: " << points << std::endl;
  

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
