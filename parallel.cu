#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/generate.h>
#include <thrust/reduce.h>
#include <thrust/functional.h>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "body.cpp"
/*
CPU
    readInitStateFile
    collisionTest
    
GPU
    calcForce
    calcAccel        1 thread per cell, reduce      OR      1 thread per row 
    integrateStep    1 thread per row
    checkIntersection  1 thread per cell, access global mem to notify if there was collision

*/


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

__device__ float distance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// F = G * m1 * m2 * r / (||r|| ^3)
// Returns each force component, x and y
__device__ float2 calcForce(Body* a, Body* b) {
    float gForce = (GRAVITY * a->mass * b->mass) / (pow(distance(a->pos_x, a->pos_y, b->pos_x, b->pos_y), 3));
    float2 f = {gForce * (b->pos_x - a->pos_x), gForce * (b->pos_y - a->pos_y)};
    return f;
}

__global__ void calcAccelerations(float* accelMatrix_x, 
                                  float* accelMatrix_y, 
                                  Body* bodies,
                                  int sz) {
    int r = threadIdx.x + blockIdx.x * blockDim.x;
    int c = threadIdx.y + blockIdx.y * blockDim.y;
    while (r < sz) {
        while (c < sz) {
            int offset = c + r * blockDim.x * gridDim.x;
            
            Body* a = &bodies[r];
            Body* b = &bodies[c];
            
            float2 f = calcForce(a, b);

            // Store the acceleration of body a in [r][c]
            accelMatrix_x[offset] = f.x / a->mass;
            accelMatrix_y[offset] = f.y / a->mass;
            
            c += blockDim.y * gridDim.y;
        }
        r += blockDim.x * gridDim.x;
    }
}

__global__ void integrateStep(float* accelMatrix_x, 
                              float* accelMatrix_y, 
                              Body* bodies,
                              int sz,
                              int deltaTime) {
    int r = threadIdx.x + blockIdx.x * blockDim.x;
	while (r < sz) {
	    Body* a = &bodies[r];
	    
	    float ax, ay;
	    for (int c = 0; c < sz; c++) {
            ax += accelMatrix_x[r * sz + c];
            ay += accelMatrix_y[r * sz + c];
        }
        // Update velocity
        a->vel_x = ax * deltaTime;
        a->vel_y = ay * deltaTime;
	    
	    // Update position
        a->pos_x += a->vel_x * deltaTime;
        a->pos_y += a->vel_y * deltaTime;
	    
	    r += blockDim.x * gridDim.x;
	}
}

int check_intersection(int x1, int y1, int x2, int y2, int r1, int r2)
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

bool collisionTest(thrust::host_vector<Body> &bodies, int duration) 
{
    // Transfer to GPU
    thrust::device_vector<Body> d_bodies = bodies;
    Body* d_bodies_ptr = thrust::raw_pointer_cast(d_bodies.data());
    
    bool collisionDetected = false;
    int timestepCounter = 0;
    float deltaTime = 0.01 * 24 * 60 * 60; // 1% of a day in seconds
    // Copy to device
    /* maybe don't need
    float* d_dt;
    cudaMalloc((void**)&d_dt, sizeof(float));
    cudaMemcpy(d_dt, &deltaTime, sizeof(float), cudaMemcpyHostToDevice);
    */

    thrust::device_vector<float> accelMatrix_x(bodies.size() * bodies.size());
    thrust::device_vector<float> accelMatrix_y(bodies.size() * bodies.size());
    float* d_accel_x_ptr = thrust::raw_pointer_cast(accelMatrix_x.data());
    float* d_accel_y_ptr = thrust::raw_pointer_cast(accelMatrix_y.data());

    // Initial state viz
    // visualize(bodies);

    while (!collisionDetected && (timestepCounter < duration))
    {
        // Temp grid sizes
        calcAccelerations<<<10, 10>>>(d_accel_x_ptr, d_accel_y_ptr, d_bodies_ptr, bodies.size());
        integrateStep<<<1, 100>>>(d_accel_x_ptr, d_accel_y_ptr, d_bodies_ptr, bodies.size(), deltaTime);
        
        // Visualize
        // visualize(bodies); // iterate through positions of bodies and display them on a coordinate plane
        
        // TODO parallelize collision check
        // Check to see if any bodies have the same position
        for (int x = 0; x < bodies.size(); x++)
        {
            for (int y = x + 1; y < bodies.size(); y++)
            {
                Body a = bodies[x];
                Body b = bodies[y];
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

    thrust::host_vector<Body> h_bodies;
    // Populate body vector
    for (int i = 0; i < name.size(); i++) {
        h_bodies.push_back(Body(name[i], mass[i], rad[i], pos_x[i], pos_y[i], vel_x[i], vel_y[i], false));
    }
    
    // Take in time duration from the user
    int duration;
    std::cout << "Enter the number of years you would like to test: ";
    std::cin >> duration;

    duration = duration * 365 * 24 * 60 * 60; // change duration to seconds

    //auto start = high_resolution_clock::now();
    bool collision = collisionTest(h_bodies, duration);
    //auto stop = high_resolution_clock::now();

    //auto execTime = duration_cast<microseconds>(stop - start);

    //std::cout << "Time taken by function: " << execTime.count() << " microseconds" << std::endl;
    
    if(collision == 1) {
        std::cout << "There was a collision" << std::endl;
    } else {
        std::cout << "There was no collision" << std::endl;
    }

    return 0;
}
