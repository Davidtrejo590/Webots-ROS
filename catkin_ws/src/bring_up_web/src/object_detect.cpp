#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <math.h>
#include <bits/stdc++.h>


/* FUNCTION TO GET THE INDEX OF MINIMUN ELEMENT */
int get_min_index(std::vector<double> data){
    auto it = std::find(data.begin(), data.end(), *min_element(data.begin(), data.end()));
        if(it != data.end()){
            int index = it - data.begin();
            return index;
        }
}

/* GENERATE RANDOMLY INITIAL CENTROIDS */
std::vector<std::vector<double>> generate_centroids(std::vector<std::vector<double>> dataset, int k){

    int size = dataset.size();
    std::vector<double> x, y, z;
    x.resize(size);
    y.resize(size);
    z.resize(size);
    double min_x, min_y, min_z, max_x, max_y, max_z;

    for(int i = 0; i < dataset.size(); i++){
        x[i] = dataset[i][0];
        y[i] = dataset[i][1];
        z[i] = dataset[i][2];
    }

    // GET THE MIN AND MAX OF EACH COMPONENT (X, Y, Z)
    min_x = *min_element(x.begin(), x.end());
    min_y = *min_element(y.begin(), y.end());
    min_z = *min_element(z.begin(), z.end());

    max_x = *max_element(x.begin(), x.end());
    max_y = *max_element(y.begin(), y.end());
    max_z = *max_element(z.begin(), z.end());


    // DEFINE SRAND
    srand(time(NULL));
    // GENERATE INITIAL CENTROIDS
    std::vector<std::vector<double>> initial_centroids(k);                                          // INITIAL CENTROIDS
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < k; i++){
        std::vector<double> point = {(rand() % int(max_x)) + min_x, (rand() % int(max_y)) + min_y, (rand() % int(max_z)) + min_z };
        initial_centroids[i] = point;
    }

    return initial_centroids;                                                                        // RETURN K INITIAL CENTROIDS
}

/* CALCULATE CENTROIDS - ASIGN EACH POINT IN THE NEAREST CLUSTER */
std::vector<std::vector<double>> calulate_centroids(std::vector<std::vector<double>> pc, std::vector<std::vector<double>> c){

    std::vector<double> p = {0.0, 0.0, 0.0};
    int m_size = c.size();                                                                  // M

    // SET OF COUNTERS
    std::vector<int> counters;
    counters.resize(m_size);

    // SET OF NEW CENTROIDS
    std::vector<std::vector<double>> new_centroids;
    new_centroids.resize(m_size);

    // INITIALIZE WHIT M VECTORS {0.0, 0.0, 0.0}
    for(int i = 0; i < new_centroids.size(); i++){
        new_centroids[i] = p;
    }


    // CALCULATE DISTANCE FOR EACH POINT 
    for(int i = 0; i < pc.size(); i++){
        std::vector<double> distances;                                                      // VECTOR OF DISTANCES WHIT M SIZE
        distances.resize(m_size); 
        for(int j = 0; j < distances.size(); j++){
                // CALCULATE EUCLIDIAN DISTANCE
                double distance = sqrt( pow((c[j][0] - pc[i][0]), 2) + pow((c[j][1] - pc[i][1]), 2) + pow((c[j][2] - pc[i][2]), 2));
                distances[j] = distance;                                                    // STORE EACH DISTANCE
        }

        int j_idx = get_min_index(distances);                                               // FIND J INDEX
        for(int k = 0; k < new_centroids[j_idx].size(); k++){
            new_centroids[j_idx][k] += pc[i][k];                                            // NEW_CENTROIDS[j] += CLOUD[I]
        }
        counters[j_idx]++;                                                                  // COUNTERS[J] ++ 
    }


    // COMPUTE NEW CENTROIDS
    for(int j = 0; j < new_centroids.size(); j++ ){
        for(int k = 0; k < new_centroids[j].size(); k++){
            // IF THE CURRENT CLUSTER HAVE POINTS
            if(new_centroids[j][0] != 0.0 or new_centroids[j][1] != 0.0 or new_centroids[j][2] != 0.0){
                new_centroids[j][k] /= counters[j];                                         // NEW_CENTROIDS[J] /= COUNTERS[J]
            }
        }
    }

    return new_centroids;                                                                   // RETURN NEW CENTROIDS
}

/* CALCULATE THE DISTANCE BETWEEN EACH CENTROID d(OLD_CENTROID, NEW_CENTROID)*/
double compare_centroids(std::vector<std::vector<double>> nc, std::vector<std::vector<double>> oc ){
    double total_distance = 0.0;

    for(int i = 0; i < nc.size(); i++){
        // EUCLEDIAN DISTANCE d(OLD_CENTROID, NEW_CENTROID)
        double distance = sqrt( 
            pow((nc[i][0]- oc[i][0]), 2) + pow((nc[i][1]- oc[i][1]), 2) + pow((nc[i][2]- oc[i][2]), 2) 
            );
        total_distance += distance;                                                           // SUM OF EACH DISTANCE
    }

    return total_distance;                                                                    // RETURN THE SUM OF DISTANCES
}



/* KMEANS FUNCTION */
std::vector<std::vector<double>> kmeans(std::vector<std::vector<double>> point_cloud){

    std::vector<std::vector<double>> initial_centroids;                                 // INITAL CENTROIDS
    std::vector<std::vector<double>> new_centroids;                                     // CENTROIDS CALCULATED
    int k = 8;                                                                          // NUMBER OF CLUSTERS
    int attemps = 0;
    int max_attemps = 100;
    double total_distance = 0.0;
    double tol = 0.1;

    initial_centroids = generate_centroids(point_cloud, k);                             // GENERATE INITIAL CENTROIDS
    new_centroids = calulate_centroids(point_cloud, initial_centroids);                 // CALCULATE NEW CENTROIDS
    total_distance = compare_centroids(new_centroids, initial_centroids);               // COMPUTE TOTAL DISTANCE BETWEEN INITAL & NEW CENTROIDS

    do{
        std::vector<std::vector<double>> centroids(new_centroids);                      // CENTROIDS <- NEW CENTROIDS
        new_centroids = calulate_centroids(point_cloud, centroids);                     // RECOMPUTE CENTROIDS
        total_distance = compare_centroids(new_centroids, centroids);                   // RECOMPUTE TOTAL DISTANCE
        attemps += 1;
    }while(total_distance > tol and attemps < max_attemps);

    return new_centroids;                                                               // RETURN THE CURRENT CENTROIDS
    
}



/*
 * OBJECT DETECT CALLBACK
*/
void objectDetectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    
    std::vector<std::vector<double>> point_cloud;                                       // POINT CLOUD FILTERED

    void* p = (void*)(&msg-> data[0]);                                                  // POINTER TO FIRST DATA IN THE POINT CLOUD 2
    // WALKING THROUGH THE POINT CLOUD
    for(size_t i = 0; i < msg->width * msg->height; i++){
        float x = *((float*)(p + 0));
        float y = *((float*)(p + 4));
        float z = *((float*)(p + 8));
        p += msg -> point_step;

        // FILL POINT CLOUD
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

    // CLUSTERING
    std::vector<std::vector<double>> current_centroids;                                  // ACTUAL CENTROIDS
    current_centroids = kmeans(point_cloud);                                             // APPLY KMEANS


    // DISPLAYING NEW CENTROIDS
    std::cout << "----------" << std::endl;
    for(int i = 0; i < current_centroids.size(); i++){
        for(int j = 0; j < current_centroids[i].size(); j++){
            if(current_centroids[i][j] != 0.0){
                std::cout << current_centroids[i][j] << " ";
            }
        }
        std::cout << std::endl;
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
