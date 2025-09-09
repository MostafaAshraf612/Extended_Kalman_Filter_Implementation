#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = z - H_ * x_;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

    x_ = x_ + K * y;
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    float rho = sqrt(px*px + py*py);
    if (rho < 1e-6) rho = 1e-6; // avoid division by zero

    VectorXd h_(3);
    h_ << rho,
          atan2(py, px),
          (px*vx + py*vy)/rho;

    Tools tools;
    MatrixXd Hj = tools.CalculateJacobian(x_);

    VectorXd y = z - h_;

    // Normalize angle to [-pi, pi]
    while (y(1) > M_PI) y(1) -= 2.*M_PI;
    while (y(1) < -M_PI) y(1) += 2.*M_PI;

    MatrixXd S = Hj * P_ * Hj.transpose() + R_;
    MatrixXd K = P_ * Hj.transpose() * S.inverse();
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

    x_ = x_ + K * y;
    P_ = (I - K * Hj) * P_;
}
