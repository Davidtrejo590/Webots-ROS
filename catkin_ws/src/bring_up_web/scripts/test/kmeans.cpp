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
        points.push_back((rand() % int(max_x)) + min_x);
        points.push_back((rand() % int(max_y)) + min_y);
        points.push_back((rand() % int(max_z)) + min_z);

        centroids.push_back(points);
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
std::vector<int> calulate_centroids(std::vector<std::vector<double>> pc, std::vector<std::vector<double>> c){

    std::vector<int> idx;                                                           // INDICES
    double distance = 0.0;
    int counter = 0;

    std::vector<std::vector<std::vector<double>>> clusters;

    
    for(int i = 0; i < pc.size(); i++){
        std::vector<double> distances;

        for(int j = 0; j < c.size(); j++){
                // CALCULATE DISTANCE
                distance = sqrt( pow((c[j][0] - pc[i][0]), 2) + pow((c[j][1] - pc[i][1]), 2) + pow((c[j][2] - pc[i][2]), 2));
                distances.push_back(distance);
                // std::cout << "[" << pc[i][0] << "," << pc[i][1] << "," << pc[i][2] << "]" << "," << "[" << c[j][0] << "," << c[j][1] << "," << c[j][2] << "]" << "," << distance << "," << j <<  std::endl;
        }

        int indice = get_min_index(distances);
        idx.push_back(indice);
        

    }

    return idx;
}





int main(){

    std::vector<std::vector<double>> initial_centroids;
    std::vector<int> indices;
    std::vector<std::vector<double>> pc;
    int k = 3;                                                                              // NUMBER OF CLUSTERS

    pc = get_data();                                                                        // GET POINT CLOUD                                              
    initial_centroids = generate_centroids(pc, k);                                              // GENERATE INITIAL CENTROIDS
    indices = calulate_centroids(pc, initial_centroids);
    
    for(int i = 0; i < indices.size(); i++){
        std::cout << indices[i] << " ";
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