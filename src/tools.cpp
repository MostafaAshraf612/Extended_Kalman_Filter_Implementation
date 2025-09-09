#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Calculate the Root Mean Square Error (RMSE)
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    // Initialize RMSE vector
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // Check validity of inputs
    if (estimations.empty() || estimations.size() != ground_truth.size()) {
        std::cout << "Error: Estimation and ground truth vectors are not of the same size or are empty." << std::endl;
        return rmse;
    }

    // Accumulate squared residuals
    for (size_t i = 0; i < estimations.size(); i++) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // Calculate the mean
    rmse = rmse / estimations.size();

    // Calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}

/**
 * Calculate the Jacobian matrix
 */
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3, 4);
    
    // Recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px*px + py*py;

    // Check for division by zero
    if (fabs(c1) < 1e-6) {
        std::cout << "Error: Division by zero in CalculateJacobian()" << std::endl;
        return Hj.setZero();
    }

    float c2 = sqrt(c1);
    float c3 = c1 * c2;

    // Compute the Jacobian matrix
    Hj << px/c2, py/c2, 0, 0,
         -py/c1, px/c1, 0, 0,
          py*(vx*py - vy*px)/c3, px*(vy*px - vx*py)/c3, px/c2, py/c2;

    return Hj;
}
