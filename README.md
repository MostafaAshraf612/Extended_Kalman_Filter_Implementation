Extended Kalman Filter (EKF) for Sensor Fusion 🚗💨

This project implements a sensor fusion algorithm using an Extended Kalman Filter (EKF) to accurately track moving objects in 2D space. It fuses measurements from radar and lidar sensors to estimate both position and velocity, providing a robust state estimation system for self-driving cars.

Project Overview

Language & Libraries:

C++ for high-performance numerical computation

Eigen for linear algebra and matrix operations

uWebSockets for real-time telemetry simulation and visualization

JSON for Modern C++ (nlohmann/json) for parsing sensor data

Core Features:

Lidar Measurements: Linear updates using standard Kalman Filter equations.

Radar Measurements: Non-linear updates using Extended Kalman Filter (EKF) equations.

RMSE Computation: Calculates Root Mean Square Error to evaluate the filter's accuracy in estimating position and velocity.

CSV Export: Optionally saves state estimates and RMSE values to CSV for offline analysis.

Real-Time Simulation: Interfaces with a simulator or dataset for live visualization of predictions versus ground truth.

Key Components:

FusionEKF – Manages sensor fusion and integrates predictions and updates for both radar and lidar measurements.

KalmanFilter – Implements the core Kalman Filter and Extended Kalman Filter equations.

Tools – Computes RMSE and Jacobian matrices; handles edge cases to prevent numerical instability.

main.cpp – Connects to a simulator, parses incoming measurements, feeds data to FusionEKF, and outputs estimates and RMSE.

How It Works

Initialization

The filter initializes the state vector x_ based on the first sensor measurement (radar or lidar).

Covariance matrices and process noise are set for prediction.

Prediction Step

Uses a linear motion model with elapsed time dt to predict the object's next state.

Updates the process covariance matrix Q_ to account for process noise.

Update Step

Lidar: Uses standard Kalman Filter update equations (linear).

Radar: Uses EKF update equations for non-linear polar-to-Cartesian conversions.

Computes the Jacobian matrix dynamically for radar measurements.

RMSE Computation

Estimates are compared to ground truth data.

Root Mean Square Error (RMSE) is calculated for position (x, y) and velocity (vx, vy) to evaluate performance.

CSV Export (Optional)

State estimates and RMSE values can be exported to a CSV file for further analysis or plotting in Python/Excel.
