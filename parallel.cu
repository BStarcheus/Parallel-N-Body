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
                       thrust::host_vector<float> &mass,
                       thrust::host_vector<float> &rad,
                       thrust::host_vector<float> &pos_x,
                       thrust::host_vector<float> &pos_y,
                       thrust::host_vector<float> &vel_x,
                       thrust::host_vector<float> &vel_y) {
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
__device__ float2 calcForce(float a_mass,
                            float a_pos_x,
                            float a_pos_y,
                            float b_mass,
                            float b_pos_x,
                            float b_pos_y) {
    float gForce = (GRAVITY * a_mass * b_mass) / (pow(distance(a_pos_x, a_pos_y, b_pos_x, b_pos_y), 3));
    float2 f = {gForce * (b_pos_x - a_pos_x), gForce * (b_pos_y - a_pos_y)};
    return f;
}

__global__ void calcAccelerations(float* accelMatrix_x, 
                                  float* accelMatrix_y, 
                                  float* mass,
                                  float* rad,
                                  float* pos_x,
                                  float* pos_y,
                                  float* vel_x,
                                  float* vel_y,
                                  int sz) {
    int r, c;
    r = threadIdx.x + blockIdx.x * blockDim.x;
    while (r < sz) {
        c = threadIdx.y + blockIdx.y * blockDim.y;
        while (c < sz) {
            int offset = c + r * blockDim.x * gridDim.x;
            
            float2 f = calcForce(mass[r], pos_x[r], pos_y[r], mass[c], pos_x[c], pos_y[c]);

            // Store the acceleration of body a in [r][c]
            accelMatrix_x[offset] = f.x / mass[r];
            accelMatrix_y[offset] = f.y / mass[r];
            
            c += blockDim.y * gridDim.y;
        }
        r += blockDim.x * gridDim.x;
    }
}

__global__ void integrateStep(float* accelMatrix_x, 
                              float* accelMatrix_y, 
                              float* mass,
                              float* rad,
                              float* pos_x,
                              float* pos_y,
                              float* vel_x,
                              float* vel_y,
                              int sz,
                              int deltaTime) {
    int r = threadIdx.x + blockIdx.x * blockDim.x;
	while (r < sz) {
	    float ax, ay;
	    for (int c = 0; c < sz; c++) {
            ax += accelMatrix_x[c + r * sz];
            ay += accelMatrix_y[c + r * sz];
        }
        // Update velocity
        vel_x[r] = ax * deltaTime;
        vel_y[r] = ay * deltaTime;
	    
	    // Update position
        pos_x[r] += vel_x[r] * deltaTime;
        pos_y[r] += vel_y[r] * deltaTime;
	    
	    r += blockDim.x * gridDim.x;
	}
}

__device__ int checkIntersection(int x1, int y1, int r1, int x2, int y2, int r2)
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

__global__ void checkCollisions(bool* hasCollided,
                                float* rad,
                                float* pos_x,
                                float* pos_y,
                                int sz) {
    int r, c, offset;
    r = threadIdx.x + blockIdx.x * blockDim.x;
    while (r < sz) {
        c = threadIdx.y + blockIdx.y * blockDim.y;
        while (c < sz) {
            if (checkIntersection(pos_x[r], pos_y[r], rad[r], pos_x[c], pos_y[c], rad[c]) != -1) {
                offset = c + r * blockDim.x * gridDim.x;
                hasCollided[offset] = true;
            }
            
            c += blockDim.y * gridDim.y;
        }
        r += blockDim.x * gridDim.x;
    }
}


bool collisionTest(std::vector<std::string> &name,
                   thrust::host_vector<float> &mass,
                   thrust::host_vector<float> &rad,
                   thrust::host_vector<float> &pos_x,
                   thrust::host_vector<float> &pos_y,
                   thrust::host_vector<float> &vel_x,
                   thrust::host_vector<float> &vel_y,
                   int duration) 
{
    // Transfer to GPU
    thrust::device_vector<float> d_mass = mass;
    thrust::device_vector<float> d_rad = rad;
    thrust::device_vector<float> d_pos_x = pos_x;
    thrust::device_vector<float> d_pos_y = pos_y;
    thrust::device_vector<float> d_vel_x = vel_x;
    thrust::device_vector<float> d_vel_y = vel_y;
    
    float* d_mass_ptr = thrust::raw_pointer_cast(d_mass.data());
    float* d_rad_ptr = thrust::raw_pointer_cast(d_rad.data());
    float* d_pos_x_ptr = thrust::raw_pointer_cast(d_pos_x.data());
    float* d_pos_y_ptr = thrust::raw_pointer_cast(d_pos_y.data());
    float* d_vel_x_ptr = thrust::raw_pointer_cast(d_vel_x.data());
    float* d_vel_y_ptr = thrust::raw_pointer_cast(d_vel_y.data());
    
    bool collisionDetected = false;
    int timestepCounter = 0;
    float deltaTime = 0.01 * 24 * 60 * 60; // 1% of a day in seconds
    // Copy to device
    /* maybe don't need
    float* d_dt;
    cudaMalloc((void**)&d_dt, sizeof(float));
    cudaMemcpy(d_dt, &deltaTime, sizeof(float), cudaMemcpyHostToDevice);
    */

    thrust::device_vector<float> accelMatrix_x(mass.size() * mass.size());
    thrust::device_vector<float> accelMatrix_y(mass.size() * mass.size());
    float* d_accel_x_ptr = thrust::raw_pointer_cast(accelMatrix_x.data());
    float* d_accel_y_ptr = thrust::raw_pointer_cast(accelMatrix_y.data());

    // Initial state viz
    // visualize(bodies);

    while (!collisionDetected && (timestepCounter < duration))
    {
        // Temp grid sizes
        calcAccelerations<<<10, 10>>>(d_accel_x_ptr, d_accel_y_ptr, d_mass_ptr, d_rad_ptr, d_pos_x_ptr, d_pos_y_ptr, d_vel_x_ptr, d_vel_y_ptr, mass.size());
        integrateStep<<<1, 100>>>(d_accel_x_ptr, d_accel_y_ptr, d_mass_ptr, d_rad_ptr, d_pos_x_ptr, d_pos_y_ptr, d_vel_x_ptr, d_vel_y_ptr, mass.size(), deltaTime);
        
        // Visualize
        // visualize(bodies); // iterate through positions of bodies and display them on a coordinate plane
        
        // Check to see if any bodies have the same position
        
        thrust::host_vector<bool> hasCollided(mass.size(), false);
        thrust::device_vector<bool> d_hasCollided = hasCollided;
        bool* d_hasCollided_ptr = thrust::raw_pointer_cast(d_hasCollided.data());
        
        checkCollisions<<<10,10>>>(d_hasCollided_ptr, d_rad_ptr, d_pos_x_ptr, d_pos_y_ptr, mass.size());
        int numColl = thrust::reduce(d_hasCollided.begin(), d_hasCollided.end(), 0);
        if (numColl > 0) {
            collisionDetected = true;
        } else {
            // Add time to the timestep counter
            timestepCounter += deltaTime;
        }
        
        // for (int x = 0; x < bodies.size(); x++)
        // {
        //     for (int y = x + 1; y < bodies.size(); y++)
        //     {
        //         Body a = bodies[x];
        //         Body b = bodies[y];
        //         std::cout << "Body " << a.name << ": " << a.pos_x << " " << a.pos_y << " " << a.vel_x << " " << a.vel_y << std::endl;
        //         std::cout << "Body " << b.name << ": " << b.pos_x << " " << b.pos_y << " " << b.vel_x << " " << b.vel_y << std::endl;

        //         if(checkIntersection(a.pos_x, a.pos_y, a.radius, b.pos_x, b.pos_y, b.radius) != -1) // When the circles are not disjoint
        //         {
        //             collisionDetected = true;
        //             a.hasCollided = true;
        //             b.hasCollided = true;
        //             bodies[x] = a;
        //             bodies[y] = b;
        //         }
        //     }
        // }
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
    thrust::host_vector<float> mass;
    thrust::host_vector<float> rad;
    thrust::host_vector<float> pos_x;
    thrust::host_vector<float> pos_y;
    thrust::host_vector<float> vel_x;
    thrust::host_vector<float> vel_y;

    // Load initial state of bodies into separate vectors for each type of data
    readInitStateFile(filename, name, mass, rad, pos_x, pos_y, vel_x, vel_y);

    // thrust::host_vector<Body> h_bodies;
    // // Populate body vector
    // for (int i = 0; i < name.size(); i++) {
    //     h_bodies.push_back(Body(name[i], mass[i], rad[i], pos_x[i], pos_y[i], vel_x[i], vel_y[i], false));
    // }
    
    // Take in time duration from the user
    int duration;
    std::cout << "Enter the number of years you would like to test: ";
    std::cin >> duration;

    duration = duration * 365 * 24 * 60 * 60; // change duration to seconds

    //auto start = high_resolution_clock::now();
    bool collision = collisionTest(name, mass, rad, pos_x, pos_y, vel_x, vel_y, duration);
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
