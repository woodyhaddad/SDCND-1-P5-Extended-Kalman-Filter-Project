### Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program Final project on Udacity.com


[//]: # (Image References)

[image1]: ./images/screen-shot-2017-04-18-at-2.33.20-pm.jpg "Lidar/Radar Data Example" 

In this project I utilize a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the following:  px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]

## Overview of a Kalman Filter: Initialize, Predict, Update 

There are three main steps for programming a Kalman filter:

* Initializing Kalman filter variables
* Predicting where the object is going to be after a time step \Delta{t}?t
* Updating where the object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well the Kalman filter performs, calculate the root mean squared error (RMSE) comparing the Kalman filter results with the provided ground truth.

These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.

## Files in the Github src Folder

* `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
* `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar
* `tools.cpp` - function to calculate RMSE and the Jacobian matrix

## How the Files Relate to Each Other

* `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`
* `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called ekf_, which is an instance of a KalmanFilter class. The ekf_ will hold the matrix and vector values. The `ekf_` instance is used to call the predict and update equations.
* The `KalmanFilter` class is defined in `kalman_filter.cpp` and `kalman_filter.h`. 'kalman_filter.cpp' contains functions for the prediction and update steps.

## Explanation of the Data File
The github repo contains one data file:

`obj_pose-laser-radar-synthetic-input.txt`
Here is a screenshot of some of the data:

![alt text][image1]

The simulator will be using this data file, and feed main.cpp values from it one line at a time.

Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y). 
Groundtruth, which represents the actual path the bicycle took, is for calculating root mean squared error.

## Reading in the Data

The code in the main.cpp file will read in and parse the data. The main.cpp file creates instances of a MeasurementPackage.

The code reads in the data file line by line. The measurement data for each line gets pushed onto a `measurement_pack_list`. The ground truth [`p_x`, `p_y`, `v_x`, `v_y`] for each line in the data file gets pushed onto `ground_truth` so RMSE can be calculated later from tools.cpp.



## Simulator configuration instructions

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

# Linux Installation:
From the project repository directory run the script: install-ubuntu.sh

# Mac Installation:
From the project repository directory run the script: install-mac.sh

Some users report needing to use cmakepatch.txt which is automatically referenced and is also located in the project repository directory.

# Windows Installation
Although it is possible to install uWebSocketIO to native Windows, the process is quite involved. Instead, you can use one of several Linux-like environments on Windows to install and run the package.

# Bash on Windows
One of the newest features to Windows 10 users is an Ubuntu Bash environment that works great and is easy to setup and use. Here is a nice step by step guide for setting up the utility.

We recommend using the newest version of Ubunut Bash 16.04, which is able to run the install-ubuntu.sh script without complications. The link here can help you check which version of Ubuntu Bash you are running, and also help you upgrade if you need to.

# Docker
If you don't want to use Bash on Windows, or you don't have Windows 10, then you can use a virtual machine to run a Docker image that already contains all the project dependencies.

First install Docker Toolbox for Windows.

Next, launch the Docker Quickstart Terminal. The default Linux virtual environment should load up. You can test that Docker is setup correctly by running `docker version` and `docker ps`.

You can enter a Docker image that has all the project dependencies by running:

`docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest`

Once inside Docker you can clone over the GitHub project repositories and run the project from there.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

# Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
  

