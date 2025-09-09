#Extended Kalman Filter Project
#Overview

This project implements an Extended Kalman Filter (EKF) for sensor fusion to track moving objects using lidar and radar measurements. The EKF combines measurements from multiple sensors to estimate an object's position and velocity accurately, even in the presence of noisy data.

This project is part of the Udacity Self-Driving Car Engineer Nanodegree.

Features

Lidar updates using standard Kalman Filter equations

Radar updates using Extended Kalman Filter equations

RMSE calculation to evaluate estimation accuracy

CSV export for logging estimated and ground truth values

Handles multiple datasets for testing and evaluation

Project Structure
├── build/                     # CMake build directory
├── data/                      # Example sensor measurement data
├── src/                       # Source code
│   ├── main.cpp               # Main program with WebSocket connection
│   ├── FusionEKF.cpp          # Fusion EKF logic
│   ├── FusionEKF.h
│   ├── kalman_filter.cpp      # Kalman Filter implementation
│   ├── kalman_filter.h
│   ├── tools.cpp              # Helper functions (RMSE, Jacobian)
│   └── tools.h
├── CMakeLists.txt             # CMake build configuration
└── README.md                  # This file

Dependencies

C++11 or higher

Eigen
 (matrix operations)

uWebSockets
 (WebSocket communication)

JSON for Modern C++

Installation & Build

Clone the repository:

git clone <repository-url>
cd CarND-Extended-Kalman-Filter-Project


Build using CMake:

mkdir build
cd build
cmake ..
make

Running the Program
./ExtendedKF


Listens on port 4567 for sensor measurements via WebSocket.

Fuses lidar and radar measurements to estimate object states.

Computes RMSE to evaluate prediction accuracy.

Optionally exports CSV files containing estimations and ground truth.

CSV Output

The program logs the following to a CSV file:

Column	Description
est_x	Estimated x-position
est_y	Estimated y-position
est_vx	Estimated x-velocity
est_vy	Estimated y-velocity
gt_x	Ground truth x-position
gt_y	Ground truth y-position
gt_vx	Ground truth x-velocity
gt_vy	Ground truth y-velocity
rmse_x	RMSE for x-position
rmse_y	RMSE for y-position
rmse_vx	RMSE for x-velocity
rmse_vy	RMSE for y-velocity
Example Output
x_ = [5.9214, 1.418, 2.297, 0.925]
P_ =
[0.0043, 0, 0.0012, 0]
[0, 0.0043, 0, 0.0012]
[0.0012, 0, 0.0045, 0]
[0, 0.0012, 0, 0.0045]
RMSE = [0.09, 0.10, 0.45, 0.30]

Notes

Ensure Eigen is properly included (#include <Eigen/Dense>).

Pay attention to matrix dimensions to avoid NaN values or crashes.

The CSV export makes it easy to visualize and analyze estimations versus ground truth.
