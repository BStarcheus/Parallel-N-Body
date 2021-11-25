#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include "body.cpp"

const double GRAVITY = 0.000000000066742;

struct float2 {
    float x;
    float y;
};

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

double velocity(float d, float t) 
{
    return d / t;
}

double distance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// F = G * m1 * m2 * r / (||r|| ^3)
// Returns each force component, x and y
float2 calcForce(Body a, Body b) {
    double gForce = (GRAVITY * a.mass * b.mass) / (pow(distance(a.pos_x, a.pos_y, b.pos_x, b.pos_y), 3));
    float2 f = {gForce * (b.pos_x - a.pos_x), gForce * (b.pos_y - a.pos_y)};
    return f;
}

void calcAccelerations(std::vector<std::vector<double> > &accelMatrix_x, 
                       std::vector<std::vector<double> > &accelMatrix_y, 
                       std::vector<Body> &bodies) {
    // Iterate through each pair of bodies and calculate the acceleration
    for (int x = 0; x < bodies.size(); x++)
    {
        for (int y = x+1; y < bodies.size(); y++) // each iter will store calculate for x,y and y,x
        {
            Body a = bodies[x];
            Body b = bodies[y];
            float2 f = calcForce(a, b);

            // Store the acceleration of x in [x][y]
            accelMatrix_x[x][y] = f.x / a.mass;
            accelMatrix_y[x][y] = f.y / a.mass;
            // Store the acceleration of y in [y][x]
            accelMatrix_x[y][x] = f.x / b.mass;
            accelMatrix_y[y][x] = f.y / b.mass;
            
            // Printing to make sure this is calculating force correctly.
            // std::cout << accelMatrix_x[x][y] << std::endl; 
            // std::cout << accelMatrix_y[x][y] << std::endl;
            // std::cout << accelMatrix_x[y][x] << std::endl;
            // std::cout << accelMatrix_y[y][x] << std::endl;
        }
    }
}

void integrateStep(std::vector<std::vector<double> > &accelMatrix_x, 
                   std::vector<std::vector<double> > &accelMatrix_y, 
                   std::vector<Body> &bodies,
                   int deltaTime) {
    
    calcAccelerations(accelMatrix_x, accelMatrix_y, bodies);

    for (int x = 0; x < bodies.size(); x++) {
        Body a = bodies[x];

        for (int y = 0; y < bodies.size(); y++) {
            // Update velocity
            a.vel_x += accelMatrix_x[x][y] * deltaTime;
            a.vel_y += accelMatrix_y[x][y] * deltaTime;
        }
        // Update position
        a.pos_x += a.vel_x * deltaTime;
        a.pos_y += a.vel_y * deltaTime;
        
        // Testing
        std::cout << a.pos_x  << std::endl; 
        std::cout << a.pos_y << std::endl;

        bodies[x] = a;
    }
}

bool collisionTest(std::vector<Body> &bodies, int duration) 
{
    bool collisionDetected = false;
    int timestepCounter = 0;
    float deltaTime = 0.01 * 24 * 60 * 60; // 1% of a day in seconds

    std::vector<std::vector<double> > accelMatrix_x(bodies.size(), std::vector<double>(bodies.size()));
    std::vector<std::vector<double> > accelMatrix_y(bodies.size(), std::vector<double>(bodies.size()));

    while (!collisionDetected && (timestepCounter < duration))
    {
        integrateStep(accelMatrix_x, accelMatrix_y, bodies, deltaTime);
        
        // Check to see if any bodies have the same position
        for (int x = 0; x < bodies.size(); x++)
        {
            for (int y = x+1; y < bodies.size(); y++) // only need to check each combination once, and don't need to check with itself
            {
// TODO check if bodies overlap using radius of each
                if((bodies[x].pos_x == bodies[y].pos_x) && (bodies[x].pos_x == bodies[y].pos_x))
                {
                    collisionDetected = true;
                    bodies[x].hasCollided = true;
                    bodies[y].hasCollided = true;
                }
            }
        }
        // Add time to the timestep counter
        timestepCounter += 1;
    }
    return collisionDetected;
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
        bodies.push_back(Body(name[i], mass[i], rad[i], pos_x[i], pos_y[i], vel_x[i], vel_y[i], false));
    }
    
    // Take in time duration from the user
    int duration;
    std::cout << "Enter the number of years you would like to test: " << std::endl;
    std::cin >> duration;

    duration = duration * 365; // change duration to days

    bool collision = collisionTest(bodies, duration);
    std::cout << collision << std::endl;

    return 0;
}