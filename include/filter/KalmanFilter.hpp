//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Kalman Filter Header                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for Kalman Filter class                                                 //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "BaseFilter.hpp"

// Kalman Filter Class
class KalmanFilter : public BaseFilter {

    // Public Class Members/Functions
    public:

        /* @filterPredict
            Inputs:
                Phik: nxn discrete time state transition matrix
                Qk: nxn process noise matrix
            Outputs:
            Description:
                Function which takes in the discrete time state transition and process noise
                matrices. The filter state and covariance are predicted forward in time. This
                function is valid for linear or extended Kalman Filter formulations.
        */
        bool filterPredict(Eigen::MatrixXd Phik,
                           Eigen::MatrixXd Qk);

        /* @filterUpdate
            Inputs:
                zk: mx1 measurement vector
                Hk: mxn discrete time measurement Jacobian matrix
                Rk: nxn measurement noise matrix
            Outputs:
            Description:
                Function which takes in the measurement vector and linearized measurement Jacobian
                and noise matrices and performs the Kalman Filter measurement update.
        */
        bool filterUpdate(Eigen::VectorXd zk,
                          Eigen::MatrixXd Hk,
                          Eigen::MatrixXd Rk);

};

