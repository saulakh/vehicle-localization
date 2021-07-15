## Kidnapped Vehicle
Self Driving Car Nanodegree Project

### Project Overview

The goal of this project is to implement a 2-D particle filter for vehicle localization. Given a map of the location, a noisy GPS estimate of its initial position, and noisy sensor and control data, a particle filter is used to find the vehicle's position with an accuracy of 3-10 cm.

### Project Build Instructions

This project uses the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

The main program can be built and run by doing the following from the project top directory:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

### Particle Filter

The project file is _particle_filter.cpp,_ which is located in the ```src``` folder. The main steps included the initialization of random particles based on noisy GPS data, a prediction step based on previous velocity and yaw rate data, updating particle weights based on observations and landmark data, and resampling particles based on weight.

#### Initialization Step

The vehicle's position is initialized using noisy GPS measurements. After adding additional Gaussian noise to these measurements, these values are used to generate random particles, which represent estimates for the vehicle's position.

#### Prediction Step

Given the velocity and yaw rate from each previous timestep, the vehicle's position can be predicted one timestep later. These changes in position and rotation are applied to the particles, to track where each particle would have moved and calculate the error from its observations. The prediction step is calculated from the following equations:

![image](https://user-images.githubusercontent.com/74683142/125709021-0f0a1e7d-e77b-4e32-8875-39111977795a.png) ![image](https://user-images.githubusercontent.com/74683142/125709033-a3fe5257-511a-4d72-87e8-8f5d9f008475.png)

#### Update Step

The observations from the vehicle's perspective are transformed into each particle's perspective, using these homogenous coordinate transformation equations:

![image](https://user-images.githubusercontent.com/74683142/125710475-befa5173-a020-42a2-9fbb-66c918ebc480.png) 

In this formula, xm and ym are the observations transformed to the map's coordinate system. The particle's position in the global map coordinate system is given by xp and yp, and theta is the rotation from the vehicle's frame to the particle's frame. Finally, xc and yc are the observations from the vehicle's coordinate system. Here are the simplified equations after matrix multiplication:

![image](https://user-images.githubusercontent.com/74683142/125713299-cf72ea55-8da7-4e2b-95b7-44c699ba51fc.png)

The transformed observations are then used to find the error between the observed location from the particle's perspective and the landmark's true position. If a particle's position is close to the vehicle's position, the error would be minimal. This error is used to update each particle's weight using the Multivariate Gaussian formula:

![image](https://user-images.githubusercontent.com/74683142/125712168-30ea8cd7-9146-4a57-8454-af2a32783a83.png)

In this formula, x and y are the positions of each transformed observation, and µx and µy are the positions of each associated landmark. The total weight of each particle is the product of its weight for each transformed observation.

#### Resampling Step
Using the Multivariate Gaussian formula, particles with a higher weight correspond to a higher probability. The particles are then resampled, by dropping particles with a low weight, and generating more particles with a higher weight. With each iteration, the particles begin to converge closer to the vehicle's true position, with a higher probability.

### Project Discussion

The main issue I had while working on this project was updating the particle weights correctly. For most iterations, all of the weight values were going to zero, so the particles were being treated as if they had an equal likelihood of representing the vehicle's position. Instead, the particles with a higher error should have been filtered out, while particles with a lower error should have been selected more frequently.

Initially, I was using ```particles[p].weight *= current_weight;``` to update the weights, which gave the majority of particles a zero weight.

To resolve this issue, I changed the way the code handled particle weights close to zero, specifically in lines 211-215 of particle_filter.cpp:

```
if (current_weight > 0.001) {
    particles[p].weight *= current_weight;
}
else {
    particles[p].weight *= 0.0001;
}
```
This gave the particles a better range of weight values, instead of mostly zeros. The particle filter was able to start dropping particles with consistently low weights, and generate new particles with higher weights to localize the vehicle's position more accurately.

### Project Results

After fixing the zero weights issue, the particle filter was able to track the vehicle's position within the desired accuracy. 

![image](https://user-images.githubusercontent.com/74683142/125708967-98c3cabf-a269-49a0-8ddb-87e18f4312f8.png)
