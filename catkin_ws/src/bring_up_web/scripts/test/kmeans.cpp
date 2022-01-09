#include <iostream>
#include <ctime>
#include <vector> 
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>



int main(){

    // DEFINE SRAND
    srand(time(NULL));
    // GENERATE INITIAL CENTROIDS
    int k = 3;
    std::vector<std::vector<double>> centroids;                                             // INITIAL CENTROIDS
    // NUMBER OF CENTROIDS ( K )
    for(int i = 0; i < k; i++){
        std::vector<double> points;
        for(int j = 0; j < 3; j++){
        points.push_back((rand() % 35) + 1);
        }

        centroids.push_back(points);
    }


    // DISPLAYING CENTROIDS
    // for(int i = 0; i < centroids.size(); i++){
    //     for(int j = 0; j < centroids[i].size(); j++){
    //     std::cout << centroids[i][j] << " ";
    //     }
    //     std::cout << std::endl;

    // }
    

    // GET THE POINT CLOUD
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

        // std::cout << x << y << z << std::endl;
        
        p.push_back(x);
        p.push_back(y);
        p.push_back(z);

        point_cloud.push_back(p);

    }

    // START KMEANS 
    double distance = 0.0;
    int counter = 0;
    for(int i = 0; i < point_cloud.size(); i++){
        double x, y, z;
        x = point_cloud[i][0];
        y = point_cloud[i][1];
        z = point_cloud[i][2];

        for(int j = 0; j < centroids.size(); j++){
                double xc, yc, zc;
                xc = centroids[j][0];
                yc = centroids[j][1];
                zc = centroids[j][2];

                // std::cout << xc << " " <<  yc << " " << zc << std::endl;
                // CALCULATE DISTANCE
                distance = sqrt( pow((xc - x), 2) + pow((yc - y), 2) + pow((zc - z), 2));
                counter++;
                // std::cout << distance << std::endl;

                
        }

    }

    std::cout << counter << std::endl;




    

    return 0;
}
