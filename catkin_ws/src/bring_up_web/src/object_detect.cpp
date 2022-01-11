#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <bits/stdc++.h>



// GET MIN INDEX 
int get_min_index(std::vector<double> data){
    auto it = std::find(data.begin(), data.end(), *min_element(data.begin(), data.end()));
        if(it != data.end()){
            int index = it - data.begin();
            return index;
        }
}


// GENERATE CENTROIDS
std::vector<std::vector<double>> generate_centroids(std::vector<std::vector<double>> dataset, int k){

    std::vector<double> x, y, z;
    double min_x, min_y, min_z, max_x, max_y, max_z;

    for(int i = 0; i < dataset.size(); i++){
        x.push_back(dataset[i][0]);
        y.push_back(dataset[i][1]);
        z.push_back(dataset[i][2]);
    }
    min_x = *min_element(x.begin(), x.end());
    min_y = *min_element(y.begin(), y.end());
    min_z = *min_element(z.begin(), z.end());

    max_x = *max_element(x.begin(), x.end());
    max_y = *max_element(y.begin(), y.end());
    max_z = *max_element(z.begin(), z.end());


    // DEFINE SRAND
    srand(time(NULL));
    // GENERATE INITIAL CENTROIDS
    std::vector<std::vector<double>> centroids;                                             // INITIAL CENTROIDS
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < k; i++){
        std::vector<double> points;
        // for(int j = 0; j < 3; j++){
        // points.push_back((rand() % 35) + 1);
        // }
        points.push_back((rand() % int(max_x)) + min_x);
        points.push_back((rand() % int(max_y)) + min_y);
        points.push_back((rand() % int(max_z)) + min_z);

        centroids.push_back(points);
    }

    return centroids;
}

// CALCULATE CENTROIDS
std::vector<int> calulate_centroids(std::vector<std::vector<double>> pc, std::vector<std::vector<double>> c){

    std::vector<int> indices;                                                           // INDICES
    double distance = 0.0;

    std::vector<std::vector<std::vector<double>>> clusters;

    for(int i = 0; i < pc.size(); i++){
        std::vector<double> distances;

        for(int j = 0; j < c.size(); j++){
                // CALCULATE DISTANCE d(centroid, point)
                distance = sqrt( pow((c[j][0] - pc[i][0]), 2) + pow((c[j][1] - pc[i][1]), 2) + pow((c[j][2] - pc[i][2]), 2));
                distances.push_back(distance);
                // std::cout << "[" << pc[i][0] << "," << pc[i][1] << "," << pc[i][2] << "]" << "," << "[" << c[j][0] << "," << c[j][1] << "," << c[j][2] << "]" << "," << distance << "," << j <<  std::endl;
        }

        // GET INDEX OF MIN DISTANCE 
        int indice = get_min_index(distances);
        indices.push_back(indice);
    }

    return indices;
}





/**
 * OBJECT DETECT CALLBACK
 */
void objectDetectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){  

  std::vector<std::vector<double>> point_cloud;


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
        std::vector<double> point;
        point.push_back(x);
        point.push_back(y);
        point.push_back(z);
        point_cloud.push_back(point);
      }
    }
  }


  // INIT KMEANS
  std::vector<std::vector<double>> initial_centroids;
  std::vector<int> indices;
  int k = 3;                                                                              // NUMBER OF CLUSTERS

  initial_centroids = generate_centroids(point_cloud, k);                                 // GENERATE INITIAL CENTROIDS
  indices = calulate_centroids(point_cloud, initial_centroids);                           // CLUSTER INDEXES
  

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
