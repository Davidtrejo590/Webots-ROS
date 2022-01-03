#include <iostream>
#include <ctime>
#include <vector> 
#include <list>

int main(){

    // DEFINE SRAND
    srand(time(NULL));


    // CREATE LIST TO STORE DATASET
    std::list<std::vector<int>> centroids;


    for(int j = 0; j < 8; j++){
        // CREATE VECTOR TO STORE PPOINT
        std::vector<int>  point;
        for (int i = 0; i < 3; i++){
            point.push_back( 1 + rand() % 6);        
        }
        centroids.push_back(point);
    }


    for(const auto &vec : centroids){
        for(const auto &v: vec)
            std::cout << v << ", ";
            std::cout << "- ";
    }
        
    
    

    return 0;
}
