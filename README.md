Extended Kalman Filter (EKF) for Sensor Fusion 🚗

This project implements a sensor fusion algorithm using an Extended Kalman Filter (EKF) to track moving objects in 2D space. It fuses measurements from radar and lidar sensors to accurately estimate both position and velocity, suitable for self-driving car applications.

Project Overview

Languages & Libraries

C++ – Core implementation

Eigen – Matrix and vector operations

uWebSockets – Real-time data exchange with simulator

JSON for Modern C++ – Parsing sensor measurements

Key Features

Sensor Fusion: Combines lidar and radar measurements for better accuracy.

EKF Updates: Radar measurements use nonlinear EKF equations; lidar uses linear Kalman Filter equations.

RMSE Evaluation: Computes Root Mean Square Error for position (x, y) and velocity (vx, vy).

CSV Export: Optionally saves estimated states and RMSE for offline analysis.

Real-Time Simulation: Integrates with a simulator or dataset for live visualization.

How It Works

Initialization

The filter initializes the state vector x_ from the first measurement (radar or lidar).

Covariance and process noise matrices are set.

Prediction

Uses a linear motion model with elapsed time dt.

Updates the process covariance matrix Q_.

Update

Lidar: Standard Kalman Filter equations (linear).

Radar: Extended Kalman Filter equations with Jacobian computation for nonlinear conversion.

RMSE Computation

Compares predicted states with ground truth.

Tracks accuracy of position and velocity estimates.

CSV Export (Optional)

Saves state estimates and RMSE values for further analysis or plotting.

Installation

Clone the repository:
