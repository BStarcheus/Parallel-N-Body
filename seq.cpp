#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include "body.cpp"

const double GRAVITY = 0.000000000066742;

void readInitStateFile(std::string filename,
                       std::vector<std::string> &name,
                       std::vector<float> &mass,
                       std::vector<float> &rad,
                       std::vector<float> &pos_x,
                       std::vector<float> &pos_y,
                       std::vector<float> &vel_x,
                       std::vector<float> &vel_y) {
    std::string elem;
    float m, r, px, py, vx, vy;

    std::ifstream file(filename);

    while (std::getline(file, elem, ',')) {
        name.push_back(elem);

        std::getline(file, elem, ',');
        mass.push_back(std::stof(elem));

        std::getline(file, elem, ',');
        rad.push_back(std::stof(elem));

        std::getline(file, elem, ',');
        pos_x.push_back(std::stof(elem));

        std::getline(file, elem, ',');
        pos_y.push_back(std::stof(elem));

        std::getline(file, elem, ',');
        vel_x.push_back(std::stof(elem));

        std::getline(file, elem);
        vel_y.push_back(std::stof(elem));
    }
}

double distance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double calcForce(Body x, Body y){
    double gForce = (GRAVITY * (x.mass * y.mass)) / (pow(distance(x.pos_x, x.pos_y, y.pos_x, y.pos_y), 2));
    return gForce;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Missing required filename argument" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    
    std::vector<std::string> name;
    std::vector<float> mass;
    std::vector<float> rad;
    std::vector<float> pos_x;
    std::vector<float> pos_y;
    std::vector<float> vel_x;
    std::vector<float> vel_y;

    // Load initial state of bodies into separate vectors for each type of data
    readInitStateFile(filename, name, mass, rad, pos_x, pos_y, vel_x, vel_y);

    std::vector<Body> bodies;
    // Populate body vector
    for (int i = 0; i < name.size(); i++) {
        bodies.push_back(Body(name[i], mass[i], rad[i], pos_x[i], pos_y[i], vel_x[i], vel_y[i]));
    }
    
    std::vector<std::vector<double>> forceMatrix;

    // Iterate through each pair of bodies and calculate the force they are exerting on each other.
    forceMatrix.resize(bodies.size());  // set # of columns (x)
    for (int x = 0; x < bodies.size(); x++)
    {
        forceMatrix[x].resize(bodies.size());  // set # of rows in each column (y)
        for (int y = 0; y < bodies.size(); y++)
        {
            // Don't bother calculating the force between an object and itself...
            if(x != y){
                forceMatrix[x][y] = calcForce(bodies[x], bodies[y]);
            }
            else{
                forceMatrix[x][y] = 0;
            }
            std::cout << forceMatrix[x][y] << std::endl; // Printing to make sure this is calculating force correctly.
        }
    }

    return 0;
}