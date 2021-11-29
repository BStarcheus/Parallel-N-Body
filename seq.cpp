#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include "body.cpp"
#include <chrono>
#include "matplotlib.h"

using namespace std::chrono;

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
        Body a = bodies[x];
	        

	for (int y = x + 1; y < bodies.size(); y++) // each iter will store calculate for x,y and y,x
        {
            Body b = bodies[y];
            
            std::cout << "Body " << a.name << ": " << a.pos_x << " " << a.pos_y << " " << a.vel_x << " " << a.vel_y << std::endl;
            std::cout << "Body " << b.name << ": " << b.pos_x << " " << b.pos_y << " " << b.vel_x << " " << b.vel_y << std::endl;

            float2 f = calcForce(a, b);

            // Store the acceleration of x in [x][y]
            accelMatrix_x[x][y] = f.x / a.mass;
            accelMatrix_y[x][y] = f.y / a.mass;
            // Store the acceleration of y in [y][x]
            accelMatrix_x[y][x] = -1 * f.x / b.mass;
            accelMatrix_y[y][x] = -1 * f.y / b.mass;
            
            // Printing to make sure this is calculating force correctly.
            std::cout << accelMatrix_x[x][y] << std::endl;
            std::cout << accelMatrix_y[x][y] << std::endl;
            std::cout << accelMatrix_x[y][x] << std::endl;
            std::cout << accelMatrix_y[y][x] << std::endl;

            bodies[x] = a;
            bodies[y] = b;
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
        // std::cout << a.pos_x  << std::endl; 
        // std::cout << a.pos_y << std::endl;

        bodies[x] = a;
    }
}

int check_intersection(int x1, int y1, int x2,
           int y2, int r1, int r2)
{
    int distSq = (x1 - x2) * (x1 - x2) +
                 (y1 - y2) * (y1 - y2);
    int radSumSq = (r1 + r2) * (r1 + r2);
    if (distSq == radSumSq)
        return 1; // Circles touch each other
    else if (distSq > radSumSq)
        return -1; // Circles do not touch each other
    else
        return 0; // Circles intersect each other
}


void visualize(std::vector<Body> &bodies) {
    for (int i = 0; i < bodies.size(); i++) {
	bodies[i].plot_x.push_back(bodies[i].pos_x);
	bodies[i].plot_y.push_back(bodies[i].pos_y);
    }
}


bool collisionTest(std::vector<Body> &bodies, int duration) 
{
    bool collisionDetected = false;
    int timestepCounter = 0;
    float deltaTime = 0.01 * 24 * 60 * 60; // 1% of a day in seconds

    std::vector<std::vector<double> > accelMatrix_x(bodies.size(), std::vector<double>(bodies.size()));
    std::vector<std::vector<double> > accelMatrix_y(bodies.size(), std::vector<double>(bodies.size()));

    // Initial state viz
    visualize(bodies);

    while (!collisionDetected && (timestepCounter < duration))
    {
        integrateStep(accelMatrix_x, accelMatrix_y, bodies, deltaTime);
        // Visualize
        visualize(bodies); // iterate through positions of bodies and display them on a coordinate plane    

  
        // Check to see if any bodies have the same position
        for (int x = 0; x < bodies.size(); x++)
        {
            Body a = bodies[x];

	    // To try to minimize the number of points we have, ease with testing
	    // if (duration % 100 == 0) {
    	    //     a.plot_x.push_back(a.pos_x);
	    //     a.plot_y.push_back(a.pos_y);
	    // }

            for (int y = x + 1; y < bodies.size(); y++)
            {
                Body b = bodies[y];

		// if (duration % 100 == 0) {
		//     b.plot_x.push_back(b.pos_x);
		//     b.plot_y.push_back(b.pos_y);
		// }

                std::cout << "Body " << a.name << ": " << a.pos_x << " " << a.pos_y << " " << a.vel_x << " " << a.vel_y << std::endl;
                std::cout << "Body " << b.name << ": " << b.pos_x << " " << b.pos_y << " " << b.vel_x << " " << b.vel_y << std::endl;

                if(check_intersection(a.pos_x, a.pos_y, b.pos_x, b.pos_y, a.radius, b.radius) != -1) // When the circles are not disjoint
                {
                    collisionDetected = true;
                    a.hasCollided = true;
                    b.hasCollided = true;
                    bodies[x] = a;
                    bodies[y] = b;
                }
            }
        }
        // Add time to the timestep counter
        timestepCounter += deltaTime;
    }
    if (collisionDetected) {
        std::cout << "Collision occurred after " << timestepCounter / (60.0 * 60 * 24) << " days" << std::endl;
    }
    return collisionDetected;
}



void printBodyPositions(std::vector<Body> &bodies) {
    // for (int i = 0; i < bodies.size(); i++) {
    for (int i = 0; i < 2; i++) {
        std::cout << "bodies[" << i << "]: " << std::endl;

	// for (int j = 0; i < bodies[i].plot_x.size(); j++) {
	for (int j = 0; i < 5; j++) {
	    std::cout << "x: " << bodies[i].plot_x[j] << std::endl;
	    std::cout << "y: " << bodies[i].plot_y[j] << std::endl;
	}
    }
}


void plotBodies(std::vector<Body> &bodies) {
    // for (int i = 0; i < bodies.size(); i++) {

    //printBodyPositions(bodies);

    for (int i = 0; i < bodies.size(); i++) {
        std::cout << "bodies[i].plot_x: " << bodies[i].plot_x[1] << std::endl; 	
        matplotlibcpp::scatter(bodies[i].plot_x, bodies[i].plot_y, bodies[i].radius);
    } 

    matplotlibcpp::save("plot.pdf");
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
    std::cout << "Enter the number of years you would like to test: ";
    std::cin >> duration;

    duration = duration * 365 * 24 * 60 * 60; // change duration to seconds

    auto start = high_resolution_clock::now();
    bool collision = collisionTest(bodies, duration);
    auto stop = high_resolution_clock::now();

    auto execTime = duration_cast<microseconds>(stop - start);

    std::cout << "Time taken by function: "
         << execTime.count() << " microseconds" << std::endl;
    
    if(collision == 1) {
        std::cout << "There was a collision" << std::endl;
    } else {
        std::cout << "There was no collision" << std::endl;
    }

    std::cout << "Plotting bodies..." << std::endl;
    plotBodies(bodies);

    return 0;
}
