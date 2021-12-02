// inlcude iostream and string libraries
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <time.h>

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Missing required arguments: filename num_bodies" << std::endl;
        return 1;
    }

    std::string filename = argv[1];

    std::ofstream outputFile;
    outputFile.open(filename, std::ios::out);
    
    int num_bodies = atoi(argv[2]);

    std::string name;
    std::string color;
    float mass;
    float rad;
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;

    char colors[] = "rgb";

    for (int counter = 0; counter <  num_bodies; counter++)
    {
        name = "body-" + std::to_string(counter);
        color = colors[rand() % 3];
        mass = rand() % 1000 + 1; // generate a random number between 1 and 1000
        rad = rand() % 100 + 1; // generate a random number between 1 and 100
        pos_x = rand() % 1000000000 + 1; // generate a random number between 1 and 1000000000
        pos_y = rand() % 1000000000 + 1; // generate a random number between 1 and 1000000000
        vel_x = rand() % 10000; // generate a random number between 1 and 10000
        vel_y = rand() % 10000; // generate a random number between 1 and 10000
        
        // write new body to output file
        outputFile << name << "," << color << "," << mass << "," << rad << "," << pos_x << "," << pos_y << "," << vel_x << "," << vel_y << std::endl;
    }
    
    outputFile.close();
    return 0;   
}