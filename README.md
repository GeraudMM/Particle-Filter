[image1]: cover.png "cover img"

# Particle-Filter

## Introduction
In this project, we will try to obtain an accurate estimation of the location of a moving car through the use of a particulate filter.


![Example][image1] 

expliquer l'image puis la base de l'algo parler de genetic mais vite fais

## Files description

 - `Kalman.cpp` is the script file containing the Kalman class use to predict and update the state of our car.
 
 - `FusionEKF.cpp` is the script file use to initialise the variables and decide which kalman function to use in which order.
 
 - `Tools.cpp` is the script use to implement the RMSE (Root Main Square Error) of our kalman filter and also use to implement the jacobian function which is needed for the extended kalman filter update function.

## Getting Started
 
This project involves the Term 2 Simulator from Udacity which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Download the [Particle-Filter repo](https://github.com/GeraudMM/Particle-Filter).

Then install uWebSocketIO.
The Udacity repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Then you only need to open the executable file in the `Term 2 Simulator` folder and launch the third project to watch the algorithm evovle.


Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

**INPUT:** values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]

**OUTPUT:** values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions

### Other Important Dependencies

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

