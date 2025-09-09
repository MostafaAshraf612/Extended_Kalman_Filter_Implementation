#include "FusionEKF.h"
#include <iostream>
#include <Eigen/Dense>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // measurement covariance matrices
    R_laser_ = MatrixXd(2,2);
    R_radar_ = MatrixXd(3,3);
    H_laser_ = MatrixXd(2,4);
    Hj_ = MatrixXd(3,4);

    R_laser_ << 0.0225, 0,
                0, 0.0225;

    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // State transition and process covariance
    ekf_.F_ = MatrixXd::Identity(4,4);
    ekf_.Q_ = MatrixXd::Zero(4,4);

    // Laser measurement matrix
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        ekf_.x_ = VectorXd(4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            float rho = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            float rho_dot = measurement_pack.raw_measurements_(2);
            if (rho < 1e-6) rho = 1e-6;
            float px = rho * cos(phi);
            float py = rho * sin(phi);
            float vx = rho_dot * cos(phi);
            float vy = rho_dot * sin(phi);

            ekf_.x_ << px, py, vx, vy;

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            float px = measurement_pack.raw_measurements_(0);
            float py = measurement_pack.raw_measurements_(1);
            ekf_.x_ << px, py, 0, 0;
        }

        // Initialize covariance matrix
        ekf_.P_ = MatrixXd(4,4);
        ekf_.P_ << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1000, 0,
                   0, 0, 0, 1000;

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        cout << "EKF initialized." << endl;
        return;
    }

    // Prediction
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    float noise_ax = 9.0;
    float noise_ay = 9.0;

    // Update state transition F
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;

    // Update process covariance Q
    float dt2 = dt*dt;
    float dt3 = dt2*dt;
    float dt4 = dt3*dt;

    ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
               0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
               dt3/2*noise_ax, 0, dt2*noise_ax, 0,
               0, dt3/2*noise_ay, 0, dt2*noise_ay;

    ekf_.Predict();

    // Update
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // Print
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
