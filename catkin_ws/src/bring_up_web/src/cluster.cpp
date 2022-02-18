#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <math.h>
#include <bits/stdc++.h>
#include <limits>


// GLOBAL VARIABLES
std::vector<int> counters;                                  // SET OF COUNTERS
std::vector<std::vector<float>> new_centroids;              // SET OF NEW CENTROIDS  

//MESSAGE 
ros::Publisher pub_poses;


/* GENERATE RANDOMLY INITIAL CENTROIDS */
std::vector<std::vector<float>> generate_centroids(int k){

    float min = 0.0;
    float max = 20.0;

    // DEFINE SRAND
    srand(time(NULL));
    // GENERATE INITIAL CENTROIDS
    std::vector<std::vector<float>> initial_centroids;                                                     // INITIAL CENTROIDS
    initial_centroids.resize(k);
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < initial_centroids.size(); i++){
        std::vector<float> point = { (rand() % int(max)) + min, (rand() % int(max)) + min, (rand() % int(max)) + min };
        initial_centroids[i] = point;
        
    }

    return initial_centroids;

}

/* CALCULATE CENTROIDS - ASIGN EACH POINT IN THE NEAREST CLUSTER */
void calculate_distance(float x, float y, float z, std::vector<std::vector<float>> centroids){

    // CALCULATE DISTANCE 
    double min_dist = std::numeric_limits<double>::infinity();                              // MIN_DISTANCE
    int j_idx = 0;                                                                          // J INDEX

    for(int i = 0; i < centroids.size(); i++){
        // CALCULATE EUCLIDIAN DISTANCE
        double distance = sqrt( 
            pow((centroids[i][0] - x), 2) + 
            pow((centroids[i][1] - y), 2) + 
            pow((centroids[i][2] - z), 2)
        );
        if(distance < min_dist){
            min_dist = distance;
            j_idx = i;
        }
    }

    new_centroids[j_idx][0] += x;                                      // NEW_CENTROIDS[J] += CLOUD[I] - X
    new_centroids[j_idx][1] += y;                                      // NEW_CENTROIDS[J] += CLOUD[I] - Y
    new_centroids[j_idx][2] += z;                                      // NEW_CENTROIDS[J] += CLOUD[I] - Z
    counters[j_idx]++;                                                                      // COUNTERS[J] ++ 

}

void compute_new_centroids(){

    for(int j = 0; j < new_centroids.size(); j++ ){
        // SKIP IF THE CURRENT CLUSTER HAVE NOT POINTS
        if(counters[j] == 0){
            continue;
        }
        new_centroids[j][0] /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - X
        new_centroids[j][1] /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - Y
        new_centroids[j][2] /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - Z
    }
}



/* CALCULATE THE DISTANCE BETWEEN EACH CENTROID d(OLD_CENTROID, NEW_CENTROID)*/
float compare_centroids( std::vector<std::vector<float>> nc, std::vector<std::vector<float>> oc ){

    float total_distance = 0.0;

    for(int i = 0; i < nc.size(); i++){
        // EUCLEDIAN DISTANCE d(OLD_CENTROID, NEW_CENTROID)
        double distance = sqrt( 
            pow((nc[i][0] - oc[i][0]), 2) +
            pow((nc[i][1] - oc[i][1]), 2) + 
            pow((nc[i][2] - oc[i][2]), 2) 
            );
        total_distance += distance;                                                             // SUM OF EACH DISTANCE
    }

    return total_distance;                                                                      // RETURN THE SUM OF DISTANCES
}

/*
 * OBJECT DETECT CALLBACK
*/
void objectDetectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    
    int k = 3;                                                                          // CLUSTERS
    float total_distance = 10.0;
    float tol = 0.1;
    std::vector<std::vector<float>> initial_centroids;                                  // INITIAL CENTROIDS
    std::vector<float> point = {0.0, 0.0, 0.0};                                         // AUX VECTOR
    
    counters.resize(k);                                                                 // INIT FOR KMEANS
    new_centroids.resize(k);
    
    // INITIALIZE WHIT M VECTORS {0.0, 0.0, 0.0}
    for(int i = 0; i < new_centroids.size(); i++){
        new_centroids[i] = point;
        counters[i] = 0;
    }
    
    
    initial_centroids = generate_centroids(k);

    void* p = (void*)(&msg-> data[0]);                                                  // POINTER TO FIRST DATA IN THE POINT CLOUD 2
    // WALKING THROUGH THE POINT CLOUD
    for(size_t i = 0; i < msg->width * msg->height; i++){
        float x = *((float*)(p + 0));
        float y = *((float*)(p + 4));
        float z = *((float*)(p + 8));
        p += msg -> point_step;

        // FILL POINT CLOUD
        if( (isinf(x) == false) and (isinf(y) == false) and (isinf(z) == false) ){
            if ( y > -1.0 ){
                // CALCULATE DISTANCE
                calculate_distance(x, y, z, initial_centroids);
            }
        }
    }

    // COMPUTE NEW CENTROIDS
    compute_new_centroids();

    // COMPARE OLD AND NEW CENTROIDS
    total_distance = compare_centroids(initial_centroids, new_centroids);

    for(int i = 0; i < counters.size(); i++){
        counters[i] = 0;
    }


    // MESSAGE
    geometry_msgs::PoseArray current_centroids;                                  // ACTUAL CENTROIDS
    current_centroids.poses.resize(new_centroids.size());
    current_centroids.header.frame_id = "lidar_link";
    // VECTOR CENTROIDS TO POSEARRAY
    for(int i = 0; i < new_centroids.size(); i++){
        current_centroids.poses[i].position.x = new_centroids[i][0];
        current_centroids.poses[i].position.y = new_centroids[i][1];
        current_centroids.poses[i].position.z = new_centroids[i][2];

    }

    pub_poses.publish(current_centroids);


}

/*   
 * MAIN FUNCTION
 */
int main(int argc, char **argv)
{
    std::cout << "OBJECT DETECT NODE..." << std::endl;
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle n;

    // PUBLISHERS
    pub_poses = n.advertise<geometry_msgs::PoseArray>("/centroid_pose", 10);
    // SUBCRIBERS
    ros::Subscriber sub = n.subscribe("/point_cloud", 10, objectDetectCallback);

    ros::spin();


    return 0;
}
