#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <math.h>
#include <bits/stdc++.h>
#include <limits>

//MESSAGE 
ros::Publisher pub_poses;


/* GENERATE RANDOMLY INITIAL CENTROIDS */
geometry_msgs::PoseArray generate_centroids(std::vector<std::vector<float>>& dataset, int k){

    int size = dataset.size();
    std::vector<float> x, y, z;
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
    geometry_msgs::PoseArray initial_centroids;                                                     // INITIAL CENTROIDS
    initial_centroids.poses.resize(k);
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < k; i++){
        initial_centroids.poses[i].position.x = (rand() % int(max_x)) + min_x;
        initial_centroids.poses[i].position.y = (rand() % int(max_y)) + min_y;
        initial_centroids.poses[i].position.z = (rand() % int(max_z)) + min_z;
    }

    return initial_centroids;                                                                        // RETURN K INITIAL CENTROIDS
}

/* CALCULATE CENTROIDS - ASIGN EACH POINT IN THE NEAREST CLUSTER */
geometry_msgs::PoseArray calulate_centroids(std::vector<std::vector<float>>& pc, geometry_msgs::PoseArray& c){

    int m_size = c.poses.size();                                                    // M

    // SET OF COUNTERS
    std::vector<int> counters;
    counters.resize(m_size);

    // SET OF NEW CENTROIDS
    geometry_msgs::PoseArray new_centroids;                                         // NEW CENTROIDS
    new_centroids.poses.resize(m_size);

    // INITIALIZE WHIT M VECTORS {0.0, 0.0, 0.0}
    for(int i = 0; i < new_centroids.poses.size(); i++){
        new_centroids.poses[i].position.x = 0.0;
        new_centroids.poses[i].position.y = 0.0;
        new_centroids.poses[i].position.z = 0.0;
    }


    // CALCULATE DISTANCE FOR EACH POINT 
    for(int i = 0; i < pc.size(); i++){
        double min_dist = std::numeric_limits<double>::infinity();                              // MIN_DISTANCE
        int j_idx = 0;                                                                          // J INDEX

        for(int j = 0; j < m_size; j++){
                // CALCULATE EUCLIDIAN DISTANCE
                double distance = sqrt( 
                    pow((c.poses[j].position.x - pc[i][0]), 2) + 
                    pow((c.poses[j].position.y - pc[i][1]), 2) + 
                    pow((c.poses[j].position.z - pc[i][2]), 2)
                );
                if(distance < min_dist){
                    min_dist = distance;
                    j_idx = j;
                }
        }

        new_centroids.poses[j_idx].position.x += pc[i][0];                                      // NEW_CENTROIDS[J] += CLOUD[I] - X
        new_centroids.poses[j_idx].position.y += pc[i][1];                                      // NEW_CENTROIDS[J] += CLOUD[I] - Y
        new_centroids.poses[j_idx].position.z += pc[i][2];                                      // NEW_CENTROIDS[J] += CLOUD[I] - Z
        counters[j_idx]++;                                                                      // COUNTERS[J] ++ 
    }


    // COMPUTE NEW CENTROIDS
    for(int j = 0; j < new_centroids.poses.size(); j++ ){
        // SKIP IF THE CURRENT CLUSTER HAVE NOT POINTS
        if(counters[j] == 0){
            continue;
        }
        new_centroids.poses[j].position.x /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - X
        new_centroids.poses[j].position.y /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - Y
        new_centroids.poses[j].position.z /= counters[j];                                       // NEW CENTROIDS[J] /= COUNTERS[J] - Z
    }

    return new_centroids;                                                                       // RETURN NEW CENTROIDS
}

/* CALCULATE THE DISTANCE BETWEEN EACH CENTROID d(OLD_CENTROID, NEW_CENTROID)*/
double compare_centroids(geometry_msgs::PoseArray& nc, geometry_msgs::PoseArray& oc ){
    double total_distance = 0.0;

    for(int i = 0; i < nc.poses.size(); i++){
        // EUCLEDIAN DISTANCE d(OLD_CENTROID, NEW_CENTROID)
        double distance = sqrt( 
            pow((nc.poses[i].position.x - oc.poses[i].position.x), 2) +
            pow((nc.poses[i].position.y - oc.poses[i].position.y), 2) + 
            pow((nc.poses[i].position.z - oc.poses[i].position.z), 2) 
            );
        total_distance += distance;                                                             // SUM OF EACH DISTANCE
    }

    return total_distance;                                                                      // RETURN THE SUM OF DISTANCES
}



/* KMEANS FUNCTION */
geometry_msgs::PoseArray kmeans(std::vector<std::vector<float>>& point_cloud){

    geometry_msgs::PoseArray initial_centroids;                                         // INITIAL CENTROIDS
    geometry_msgs::PoseArray new_centroids;                                             // CENTROIDS CALCULATED
    int k = 3;                                                                          // NUMBER OF CLUSTERS
    int attemps = 0;
    int max_attemps = 100;
    double total_distance = 0.0;
    double tol = 0.1;

    initial_centroids = generate_centroids(point_cloud, k);                             // GENERATE INITIAL CENTROIDS
    new_centroids = calulate_centroids(point_cloud, initial_centroids);                 // CALCULATE NEW CENTROIDS
    total_distance = compare_centroids(new_centroids, initial_centroids);               // COMPUTE TOTAL DISTANCE BETWEEN INITAL & NEW CENTROIDS

    do{
        geometry_msgs::PoseArray centroids = new_centroids;                             // CENTROIDS <- NEW CENTROIDS
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
    
    std::vector<std::vector<float>> point_cloud;                                       // POINT CLOUD FILTERED

    void* p = (void*)(&msg-> data[0]);                                                  // POINTER TO FIRST DATA IN THE POINT CLOUD 2
    // WALKING THROUGH THE POINT CLOUD
    for(size_t i = 0; i < msg->width * msg->height; i++){
        float x = *((float*)(p + 0));
        float y = *((float*)(p + 4));
        float z = *((float*)(p + 8));
        p += msg -> point_step;

        // FILL POINT CLOUD
        if( (isinf(x) == false) and (isinf(y) == false) and (isinf(z) == false) ){
            if ( (x > 1.0 or x < -1.0) and (y > -2.5) and (z > 2.0 or z < -2.0) ){
                std::vector<float> point = {x, y, z};
                point_cloud.push_back(point);
                // std::cout << x << " " << y << " " << z << std::endl;
                // CLUSTER HERE

            }
        }
    }

    // CLUSTERING
    geometry_msgs::PoseArray current_centroids;                                  // ACTUAL CENTROIDS
    current_centroids = kmeans(point_cloud);                                             // APPLY KMEANS
    current_centroids.header.frame_id = "lidar_link";

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


// PUBLICAR LOS CENTROIDES CALCULADOS DESDE EL CALLBACK EN UN POSE ARRAY -- DONE
// ELIMINAR FOR'S NO NECESARIOS -- DONE
// REVISAR VECTORES DONDE SE USA RESIZE -- DONE
// CLUSTERIZAR DESDE EL FILTRADO DE LA NUBE DE PUNTOS
// GENERAR CENTROIDES INICIALES AL INICIO DEL CALLBACK
// ELIMINAR EL VECTOR DE DISTANCES -- INICIAR UNA VARIABLE MIN_DISTANCIA EN INF -- DONE
// REVISAR EL AREA DE INTERES DE LA NUBE DE PUNTOS

