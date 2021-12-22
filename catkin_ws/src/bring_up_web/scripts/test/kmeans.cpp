#include <iostream>
#include <ctime>



int main(){
    // std::cout << "Hello World";

    srand(time(NULL));


    int point [3];
    

    for(int i=0; i < 3; i++){
        point[i] = (rand() % 20) + 1;
    }

    for(int i = 0; i < 3; i++){
        std::cout << point[i] << std::endl;
    }


    return 0;
}
