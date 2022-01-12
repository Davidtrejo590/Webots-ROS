#include <iostream>
#include <ctime>
#include <vector> 
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>
#include <bits/stdc++.h>


int get_min_index(std::vector<double> data){
    auto it = std::find(data.begin(), data.end(), *min_element(data.begin(), data.end()));
        if(it != data.end()){
            int index = it - data.begin();
            return index;
        }
}

// GENERATE CENTROIDS
std::vector<std::vector<double>> generate_centroids(std::vector<std::vector<double>> dataset, int k){

    int size = dataset.size();

    std::vector<double> x(size), y(size), z(size);
    double min_x, min_y, min_z, max_x, max_y, max_z;


    for(int i = 0; i < dataset.size(); i++){
        x[i] = dataset[i][0];
        y[i] = dataset[i][1];
        z[i] = dataset[i][2];
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
    std::vector<std::vector<double>> centroids(k);                                             // INITIAL CENTROIDS
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < k; i++){
        std::vector<double> point = {(rand() % int(max_x)) + min_x, (rand() % int(max_y)) + min_y, (rand() % int(max_z)) + min_z };
        // points.push_back((rand() % int(max_x)) + min_x);
        // points.push_back((rand() % int(max_y)) + min_y);
        // points.push_back((rand() % int(max_z)) + min_z);
        centroids[i] = point;
    }

    return centroids;
}


// GET POINT CLOUD
std::vector<std::vector<double>> get_data(){
    std::vector<std::vector<double>> point_cloud;
    std::string line;

    std::ifstream file("output.csv");

    while(getline(file, line)){
        std::vector<double> p;
        std::stringstream lineStream(line);
        std::string bit;
        double x, y, z;
        getline(lineStream, bit, ',');
        x = std::stof(bit);
        getline(lineStream, bit, ',');
        y = std::stod(bit);
        getline(lineStream, bit, '\n');
        z = std::stod(bit);
        
        p.push_back(x);
        p.push_back(y);
        p.push_back(z);

        point_cloud.push_back(p);

    }

    return point_cloud;
}

// CALCULATE CENTROIDS
std::vector<std::vector<double>> calulate_centroids(std::vector<std::vector<double>> pc, std::vector<std::vector<double>> c){

    std::vector<double> p = {0.0, 0.0, 0.0};
    int m_size = c.size();

    // COUNTERS
    std::vector<int> counters(m_size);
    // counters.resize(c.size());
    // NEW CENTROIDS
    std::vector<std::vector<double>> new_centroids(m_size);
    // new_centroids.resize(c.size());

    // INICIALIZAR VENTORES EN 0
    for(int i = 0; i < new_centroids.size(); i++){
        new_centroids[i] = p;
    }


    for(int i = 0; i < pc.size(); i++){
        std::vector<double> distances(m_size);
        for(int j = 0; j < distances.size(); j++){
                // CALCULATE DISTANCE
                double distance = sqrt( pow((c[j][0] - pc[i][0]), 2) + pow((c[j][1] - pc[i][1]), 2) + pow((c[j][2] - pc[i][2]), 2));
                // distances.push_back(distance);
                distances[j] = distance;
                // std::cout << "[" << pc[i][0] << "," << pc[i][1] << "," << pc[i][2] << "]" << "," << "[" << c[j][0] << "," << c[j][1] << "," << c[j][2] << "]" << "," << distance << "," << j <<  std::endl;
        }

        int indice = get_min_index(distances);                                                // INDICE J
        // std::cout << pc[i][0] << " " << pc[i][1] << " " << pc[i][2] << " "<< indice << std::endl;
        for(int k = 0; k < new_centroids[indice].size(); k++){
            new_centroids[indice][k] += pc[i][k];
        }
        counters[indice]++;



    }

    // for(int i = 0; i < counters.size(); i++){
    //     std::cout << counters[i] << " ";
    // }

    // std::cout <<  std::endl;
    // std::cout << "-----" <<  std::endl;
    // std::cout << new_centroids[0][0] << " " << new_centroids[0][1] << " " << new_centroids[0][2] <<  std::endl;
    // std::cout << new_centroids[1][0] << " " << new_centroids[1][1] << " " << new_centroids[1][2] <<  std::endl;
    // std::cout << new_centroids[2][0] << " " << new_centroids[2][1] << " " << new_centroids[2][2] <<  std::endl;
    // std::cout << "-----" <<  std::endl;

    for(int j = 0; j < new_centroids.size(); j++ ){
        for(int k = 0; k < new_centroids[j].size(); k++){
            if(new_centroids[j][0] != 0.0 or new_centroids[j][1] != 0.0 or new_centroids[j][2] != 0.0){
                new_centroids[j][k] /= counters[j];
            }
        }
    }

    // for(int i = 0; i < new_centroids.size(); i++){
    //     for(int j = 0; j < new_centroids[i].size(); j++){
    //     std::cout << new_centroids[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    return new_centroids;
}

// COMPARE CENTROIDS

double compare_centroids(std::vector<std::vector<double>> nc, std::vector<std::vector<double>> oc ){
    double total_distance = 0.0;

    for(int i = 0; i < nc.size(); i++){
        double distance = sqrt( pow((nc[i][0]- oc[i][0]), 2) + pow((nc[i][1]- oc[i][1]), 2) + pow((nc[i][2]- oc[i][2]), 2) );
        // std::cout << nc[i][0] << " "  << nc[i][1] << " " << nc[i][2] << " / " << oc[i][0] << " " << oc[i][1] << oc[i][2] << " " << distance << std::endl;
        total_distance += distance;
        // std::cout << total_distance << std::endl;
    }

    return total_distance;
}


std::vector<std::vector<double>> kmeans(std::vector<std::vector<double>> point_cloud){
    std::vector<std::vector<double>> initial_centroids;
    std::vector<std::vector<double>> new_centroids;
    // std::vector<std::vector<double>> point_cloud;
    int k = 8;                                                                              // NUMBER OF CLUSTERS
    int attemps = 0;
    int max_attemps = 100;
    double total_distance = 0.0;
    double tol = 0.1;

    // point_cloud = get_data();                                                                        // GET POINT CLOUD                                              
    initial_centroids = generate_centroids(point_cloud, k);                                          // GENERATE INITIAL CENTROIDS
    // for(int i = 0; i < initial_centroids.size(); i++){
    //     for(int j = 0; j < initial_centroids[i].size(); j++){
    //     std::cout << initial_centroids[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    new_centroids = calulate_centroids(point_cloud, initial_centroids);
    total_distance = compare_centroids(new_centroids, initial_centroids);

    do{
        std::vector<std::vector<double>> centroids(new_centroids);
        new_centroids = calulate_centroids(point_cloud, centroids);
        total_distance = compare_centroids(new_centroids, centroids);
        attemps += 1;
    }while(total_distance > tol and attemps < max_attemps);

    return new_centroids;
    
}

int main(){

    std::vector<std::vector<double>> current_centroids;
    std::vector<std::vector<double>> point_cloud;
    point_cloud = get_data();                                                                        // GET POINT CLOUD                                              
    current_centroids = kmeans(point_cloud);



    // std::vector<std::vector<double>> initial_centroids;
    // std::vector<std::vector<double>> new_centroids;
    // std::vector<std::vector<double>> point_cloud;
    // int k = 8;                                                                              // NUMBER OF CLUSTERS
    // int attemps = 0;
    // int max_attemps = 100;
    // double total_distance = 0.0;
    // double tol = 0.1;

    // point_cloud = get_data();                                                                        // GET POINT CLOUD                                              
    // initial_centroids = generate_centroids(point_cloud, k);                                          // GENERATE INITIAL CENTROIDS
    // // for(int i = 0; i < initial_centroids.size(); i++){
    // //     for(int j = 0; j < initial_centroids[i].size(); j++){
    // //     std::cout << initial_centroids[i][j] << " ";
    // //     }
    // //     std::cout << std::endl;
    // // }
    // new_centroids = calulate_centroids(point_cloud, initial_centroids);
    // total_distance = compare_centroids(new_centroids, initial_centroids);

    // do{
    //     std::vector<std::vector<double>> centroids(new_centroids);
    //     new_centroids = calulate_centroids(point_cloud, centroids);
    //     total_distance = compare_centroids(new_centroids, centroids);
    //     attemps += 1;
    // }while(total_distance > tol and attemps < max_attemps);

    for(int i = 0; i < current_centroids.size(); i++){
        for(int j = 0; j < current_centroids[i].size(); j++){
        std::cout << current_centroids[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}



// DISPLAYING CENTROIDS
    // for(int i = 0; i < initial_centroids.size(); i++){
    //     for(int j = 0; j < initial_centroids[i].size(); j++){
    //     std::cout << initial_centroids[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // DISPLAYING POINT CLOUD
    // for(int i = 0; i < pc.size(); i++){
    //     for(int j = 0; j < pc[i].size(); j++){
    //     std::cout << pc[i][j] << " ";
    //     }
    //     std::cout << std::endl;

    // }





// // START KMEANS 
    // double distance = 0.0;
    // int counter = 0;
    // std::vector<int> idx;                                                           // INDICES
    // for(int i = 0; i < point_cloud.size(); i++){
    //     double x, y, z;
    //     x = point_cloud[i][0];
    //     y = point_cloud[i][1];
    //     z = point_cloud[i][2];
    //     double aux = 1000.0;

    //     for(int j = 0; j < centroids.size(); j++){
    //             double xc, yc, zc;
    //             xc = centroids[j][0];
    //             yc = centroids[j][1];
    //             zc = centroids[j][2];

    //             // std::cout << xc << " " <<  yc << " " << zc << std::endl;
    //             // CALCULATE DISTANCE
    //             distance = sqrt( pow((xc - x), 2) + pow((yc - y), 2) + pow((zc - z), 2));
    //             // std::cout << distance << std::endl;
    //             if(distance < aux){
    //                 aux = distance;
    //                 // std::cout << "MIN" << distance << std::endl;
    //                 counter = j;
    //             }
    //     }
    //     idx.push_back(counter);
    //     // std::cout << "MIN" << aux << std::endl;

    // }

    // for(int i = 0; i < idx.size(); i++){
    //     std::cout << idx[i] << std::endl;
    // }






//     std::cout << "-----" << std::endl;
//   for(int i = 0; i < initial_centroids.size(); i++){
//         for(int j = 0; j < initial_centroids[i].size(); j++){
//         std::cout << initial_centroids[i][j] << " ";
//         }
//         std::cout << std::endl;
//   }

//   std::cout << "INDEX" << std::endl;
//   for(int i = 0; i < indices.size(); i++){
//         std::cout << indices[i] << " ";
//     }
