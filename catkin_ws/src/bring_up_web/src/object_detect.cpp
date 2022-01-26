#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <algorithm>
#include <ctime>
#include <vector>
#include <iostream>
#include <math.h>
#include <bits/stdc++.h>
#include <limits>

std::vector<std::vector<double>> initial_centroids = {
    { -3.64229929,  -0.55005622,  -6.23897093 },
    {  3.79652465,  -0.35820951,  -0.04962473 },
    {  3.32333617,  -0.58985401, -14.47182993 },
    { -3.03360501,  -0.47881233,  22.09920644 },
    {  4.61459438,  -0.60176526, -28.79273614 },
    { -3.74919798,  -0.60729853, -19.39469276 },
    { -3.68494749,  -0.58626728, -35.58342223 },
    { -4.00155359,  -0.52482279,  -5.27777102 }
};

// std::vector<std::vector<double>> initial_centroids;

//MESSAGE 
ros::Publisher pub_poses;

/* GENERATE RANDOMLY INITIAL CENTROIDS */
std::vector<std::vector<double>> generate_centroids(int k){

    double min = -10.0;
    double max = 10.0;

    // DEFINE SRAND
    srand(time(NULL));
    // GENERATE INITIAL CENTROIDS
    std::vector<std::vector<double>> initial_centroids;                                                     // INITIAL CENTROIDS
    initial_centroids.resize(k);
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < initial_centroids.size(); i++){
        std::vector<double> point = { (rand() % int(max)) + min, (rand() % int(max)) + min, (rand() % int(max)) + min };
        initial_centroids[i] = point;
        
    }

    return initial_centroids;

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
        double min_dist = std::numeric_limits<double>::infinity();                              // MIN_DISTANCE
        int j_idx = 0;                                                                          // J INDEX

        for(int j = 0; j < m_size; j++){
                // CALCULATE EUCLIDIAN DISTANCE
                double distance = sqrt( pow((c[j][0] - pc[i][0]), 2) + pow((c[j][1] - pc[i][1]), 2) + pow((c[j][2] - pc[i][2]), 2));
                if(distance < min_dist){
                    min_dist = distance;
                    j_idx = j;
                }
        }

        new_centroids[j_idx][0] += pc[i][0];                                      // NEW_CENTROIDS[J] += CLOUD[I] - X
        new_centroids[j_idx][1] += pc[i][1];                                      // NEW_CENTROIDS[J] += CLOUD[I] - Y
        new_centroids[j_idx][2] += pc[i][2];                                      // NEW_CENTROIDS[J] += CLOUD[I] - Z
        counters[j_idx]++;                                                                      // COUNTERS[J] ++ 
    }


    // COMPUTE NEW CENTROIDS
    for(int j = 0; j < new_centroids.size(); j++ ){
        if(counters[j] == 0){
            continue;
        }
        new_centroids[j][0] /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - X
        new_centroids[j][1] /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - Y
        new_centroids[j][2] /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - Z
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

/* CALCULATE THE DISTANCE FOR EACH CENTROID  */
std::vector<std::vector<double>> centroid_distance(std::vector<std::vector<double>> c){

    for(int i = 0; i < c.size(); i++){
        for(int j = 0; j < c.size(); j++){
            double distance = sqrt( 
                pow((c[i][0]- c[j][0]), 2) + pow((c[i][1]- c[j][1]), 2) + pow((c[i][2]- c[j][2]), 2) 
            );
            if( distance != 0.0 and distance < 2.0 ){
                c[i][0] = ( c[i][0] + c[j][0] ) / 2;
                c[i][1] = ( c[i][1] + c[j][1] ) / 2;
                c[i][2] = ( c[i][2] + c[j][2] ) / 2;
            }
        }
    }

    return c;

}



/* KMEANS FUNCTION */
geometry_msgs::PoseArray kmeans(std::vector<std::vector<double>> point_cloud){

    geometry_msgs::PoseArray actual_centroids;
    std::vector<std::vector<double>> new_centroids;                                     // CENTROIDS CALCULATED
    int k = 8;                                                                          // NUMBER OF CLUSTERS
    int attemps = 0;
    int max_attemps = 100;
    double total_distance = 0.0;
    double tol = 0.1;

    new_centroids = calulate_centroids(point_cloud, initial_centroids);                 // CALCULATE NEW CENTROIDS
    total_distance = compare_centroids(new_centroids, initial_centroids);               // COMPUTE TOTAL DISTANCE BETWEEN INITAL & NEW CENTROIDS

    while (total_distance > tol)
    {
        std::vector<std::vector<double>> centroids(new_centroids);                      // CENTROIDS <- NEW CENTROIDS
        new_centroids = calulate_centroids(point_cloud, centroids);                     // RECOMPUTE CENTROIDS
        total_distance = compare_centroids(new_centroids, centroids);                   // RECOMPUTE TOTAL DISTANCE
        attemps += 1;
    }

    // CHECK DISTANCE BETWEEN CENTROIDS
    new_centroids = centroid_distance(new_centroids);

    // PASS TO POSE ARRAY
    actual_centroids.poses.resize(new_centroids.size());

    for(int i = 0; i < new_centroids.size(); i++){
        actual_centroids.poses[i].position.x = new_centroids[i][0];
        actual_centroids.poses[i].position.y = new_centroids[i][1];
        actual_centroids.poses[i].position.z = new_centroids[i][2];
    }


    return actual_centroids;                                                            // RETURN THE CURRENT CENTROIDS
    
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
            if( (x > 1.0 or x < -1.0) and (y > -0.75) ){
                std::vector<double> point = {x, y, z};
                point_cloud.push_back(point);
            }
        }
    }

    // CLUSTERING
    geometry_msgs::PoseArray centroids;                                                 // ACTUAL CENTROIDS
    centroids = kmeans(point_cloud);                                                    // APPLY KMEANS
    centroids.header.frame_id = "lidar_link";
    pub_poses.publish(centroids);                                                       // PUBLISH CENTROIDS
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
    pub_poses = n.advertise<geometry_msgs::PoseArray>("/object_pose", 10);
    ros::Subscriber sub = n.subscribe("/point_cloud", 10, objectDetectCallback);
    ros::spin();

    return 0;
}
