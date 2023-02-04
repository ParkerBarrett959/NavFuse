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

// Kalman Filter Class
class KalmanFilter {

    // Public Class Members/Functions
    public:

        /* @filterInitialize
            Inputs:
                x0: nx1 dimensional initial state vector
                P0: nxn dimensional initial covariance matrix
            Outputs:
            Description:
                Function which takes in the initial state vector and covariance matrix and sets
                the filter class members
        */
        bool filterInitialize(Eigen::VectorXd x0,
                              Eigen::MatrixXd P0);

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

        /* @getCovariance
            Inputs:
            Outputs:
                Pk: nxn dimensional covariance matrix
            Description:
                Function which returns the current filter covariance matrix.
        */
        Eigen::MatrixXd getCovariance();

        /* @getState
            Inputs:
            Outputs:
                xk: nx1 dimensional state vector
            Description:
                Function which returns the current filter state vector.
        */
        Eigen::MatrixXd getState();

    // Private Class Members/Function
    private:

        // Filter State and Covariance
        Eigen::MatrixXd filterCovariance_;      // nxn dimensional filter covariance matrix
        Eigen::VectorXd filterState_;           // nx1 dimensional filter state vector

};

