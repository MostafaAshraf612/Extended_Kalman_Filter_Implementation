Extended Kalman Filter (EKF) for Sensor Fusion

This project implements an Extended Kalman Filter (EKF) for fusing radar and laser sensor measurements to estimate the position and velocity of a moving object. It is part of the Self-Driving Car Engineer Nanodegree from Udacity.

Project Overview

The goal of this project is to track an object in 2D space by combining data from multiple sensors. The EKF is used to:

Predict the state of the object (position and velocity) over time.

Update the state using noisy measurements from laser (LIDAR) and radar sensors.

Compute the Root Mean Square Error (RMSE) between estimated and ground truth positions and velocities.

Features

Sensor Fusion: Combines radar and laser data for better accuracy.

Extended Kalman Filter: Handles nonlinear radar measurements with a Jacobian matrix.

Real-time Visualization: Connects to a simulator using WebSockets for real-time updates.

RMSE Calculation: Evaluates the performance of the EKF.

CSV Export: Saves estimates and RMSE to ekf_output.csv for offline analysis or plotting.

Dependencies

C++11 or higher

Eigen
