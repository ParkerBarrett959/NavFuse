//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Unscented Kalman Filter Header                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the Unscented Kalman Filter class                                   //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "BaseFilter.hpp"

// Unscented Kalman Filter Class
class UnscentedKalmanFilter : public BaseFilter {

    // Public Class Members/Functions
    public:

        /* @filterUkfPredict
            Inputs:
                Wi: (2n+1)x1 vector of sigma point weights
                yi: nx(2n+1) matrix of sigma points passed through the full nonlinear dynamics
                Qk: nxn discrete time process noise matrix
            Outputs:
            Description:
                Function which takes in the sigma weights and points propagated through the full nonlinear
                dynamics model, along with the discrete time process noise and performs the UKF filter
                prediction.
        */
        bool filterUkfPredict(Eigen::VectorXd Wi,
                              Eigen::MatrixXd yi,
                              Eigen::MatrixXd Qk);

        /* @filterUkfUpdate
            Inputs:
                yi: nx(2n+1) dimensional matrix of sigma vectors (each column) propagated through the system dynamics
                Wi: (2n+1)x1 dimensional vector of sigma vector weights
                zi: mx(2n+1) dimensional matrix of sigma measurement vectors (each column) propagated through the nonlinear measurement model
                zk: mx1 measurement vector
                Rk: nxn measurement noise matrix 
            Outputs:
            Description:
                Function which takes in a matrix of UKF sigma measurement vectors which have been propagated 
                through the measurement model as follows: zi = h(Xi). Each column of the matrix corresponds to
                each measurement vector being propagated through the nonlinear model. The indices of each column
                corresponds to each row of the sigma weights vector, which is also inluded. The function computes
                the measurement mean, followed by measurement space covariance and cross-correlation. This is used
                to compute the Kalman gain. This is then used in the standard Kalman Filter measurement update 
                equations to compute the filter updated state and covariance.
        */
        bool filterUkfUpdate(Eigen::MatrixXd yi,
                             Eigen::VectorXd Wi,
                             Eigen::MatrixXd zi,
                             Eigen::VectorXd zk,
                             Eigen::MatrixXd Rk);

};