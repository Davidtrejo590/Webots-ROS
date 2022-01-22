#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <ctime>
#include <vector>
#include <iostream>
#include <math.h>
#include <bits/stdc++.h>

std::vector<std::vector<double>> initial_centroids = {
    { -3.24854239, -0.78178249,  21.68897923},
    {  5.23484374,  -1.10132963, -27.92289582},
    {  3.28875793,  -0.4766275,   -8.91994962},
    {  3.87006921,  -0.37626108,   6.56706054},
    { 10.89367012,  -1.22822929,  24.94426386},
    { -6.5163362,   -1.36696403, -26.10448287},
    { -6.54936334,  -1.22340166,  25.60631742},
    { -4.05076476,  -0.42877155,  -7.2448349 },
};


//MESSAGE 
ros::Publisher pub_poses;

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
    std::vector<std::vector<double>> initial_centroids;                                          // INITIAL CENTROIDS
    initial_centroids.resize(k);
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < initial_centroids.size(); i++){
        initial_centroids[i] = {(rand() % int(max_x)) + min_x, (rand() % int(max_y)) + min_y, (rand() % int(max_z)) + min_z };
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



/* KMEANS FUNCTION */
geometry_msgs::PoseArray kmeans(std::vector<std::vector<double>> point_cloud){

    geometry_msgs::PoseArray actual_centroids;
    // std::vector<std::vector<double>> initial_centroids;                                 // INITAL CENTROIDS
    std::vector<std::vector<double>> new_centroids;                                     // CENTROIDS CALCULATED
    int k = 8;                                                                          // NUMBER OF CLUSTERS
    int attemps = 0;
    int max_attemps = 100;
    double total_distance = 0.0;
    double tol = 0.1;

    // initial_centroids = generate_centroids(point_cloud, k);                             // GENERATE INITIAL CENTROIDS

    new_centroids = calulate_centroids(point_cloud, initial_centroids);                 // CALCULATE NEW CENTROIDS
    total_distance = compare_centroids(new_centroids, initial_centroids);               // COMPUTE TOTAL DISTANCE BETWEEN INITAL & NEW CENTROIDS

    while (total_distance > tol)
    {
        std::vector<std::vector<double>> centroids(new_centroids);                      // CENTROIDS <- NEW CENTROIDS
        new_centroids = calulate_centroids(point_cloud, centroids);                     // RECOMPUTE CENTROIDS
        total_distance = compare_centroids(new_centroids, centroids);                   // RECOMPUTE TOTAL DISTANCE
        attemps += 1;
    }

    // std::cout << attemps << std::endl;

   
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
            std::vector<double> point = {x, y, z};
            point_cloud.push_back(point);
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


// PUBLICAR LOS CENTROIDES CALCULADOS DESDE EL CALLBACK EN UN POSE ARRAY -- DONE
// ELIMINAR FOR'S NO NECESARIOS -- DONE
// REVISAR VECTORES DONDE SE USA RESIZE -- DONE
// CLUSTERIZAR DESDE EL FILTRADO DE LA NUBE DE PUNTOS
// GENERAR CENTROIDES INICIALES AL INICIO DEL CALLBACK
// ELIMINAR EL VECTOR DE DISTANCES -- INICIAR UNA VARIABLE MIN_DISTANCIA EN INF -- DONE
// REVISAR EL AREA DE INTERES DE LA NUBE DE PUNTOS


/* 
    * COSAS QUE PUEDEN ESTAR AFECTANDO EL CLUSTERING
    * LAS LECTURAS DEL LIDAR VELODYNE CAMBIAN BASTANTE RESPECTO A LAS ANTERIORES
    * LA OBTENCIÓN DE LOS CENTROIDES INICIALES DE MANERA ALEATORIA HACE QUE LOS CENTROIDES FINALES SE MUEVAN
    * EL LIDAR SICK SOLO DA PUNTOS FRONTALES EN 180 GRADOS
    * HACER UNA MEJOR ELECCIÓN DE LOS CENTROIDES INCIALES
    * 


*/