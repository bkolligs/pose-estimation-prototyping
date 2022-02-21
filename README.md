# Pose Estimation Prototyping

This repository exists to prototype the code written for the pose estimation system for the [ MoonRanger Lunar Rover ](https://labs.ri.cmu.edu/moonranger/).

## Modules
The main components tested here are as follows: 
1. [Sun Sensor](sun-sensor): Tests for verifying the math that converts `alpha` and `beta` angles to a unit vector representation of a sun ray. 
2. [EKF Simulation](ekf-simulation): Python bindings for the current implementation of the orientation EKF so that we can test, verify and visualize the results of the math. 
3. [Translation Estimation](translation-quaternions): Tests that verify we can use wheel velocities to estimate 3D position for a certain timestep.