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

        /* @filterPredict
            Inputs:
                xk: nx1 dimensional state vector
                Pk: nxn dimensional covariance matrix
                Phik: nxn discrete time state transition matrix
                Qk: nxn process noise matrix
            Outputs:
                xkp1: nx1 dimensional predicted state vector
                Pkp1: nxn dimensional predicted covariance matrix
            Description:
                Function which takes in state and covariance, along with the discrete time state
                matrix and process noise matrix. The state and covariance are predicted. This
                function is valid for linear or extended Kalman Filter formulations.
        */
        bool filterPredict(Eigen::VectorXd &xk,
                           Eigen::MatrixXd &Pk,
                           Eigen::MatrixXd &Phik,
                           Eigen::MatrixXd &Qk,
                           Eigen::VectorXd &xkp1,
                           Eigen::MatrixXd &Pkp1);

        /* @filterUpdate
            Inputs:
                xk: nx1 dimensional state vector
                Pk: nxn dimensional covariance matrix
                zk: mx1 measurement vector
                H: mxn discrete time measurement Jacobian matrix
                Rk: nxn measurement noise matrix
                xkp1: nx1 dimensional measurement updated state vector
                Pkp1: nxn dimensional measurement updated covariance matrix
            Outputs:
                xkp1: nx1 dimensional measurement updated state vector
                Pkp1: nxn dimensional measurement updated covariance matrix
            Description:
                Function which takes in the predicted state and covariance, along with the 
                measurement, linearized measurement model and measurement noise matrix. It
                also takes in the Kalman gain matrix, which is used to perform a standard 
                Kalman Filter update. This function is valid for linear or extended Kalman 
                Filter formulations 
        */
        bool filterUpdate(Eigen::VectorXd &xk,
                          Eigen::MatrixXd &Pk,
                          Eigen::VectorXd &zk,
                          Eigen::MatrixXd &H,
                          Eigen::MatrixXd &Rk,
                          Eigen::VectorXd &xkp1,
                          Eigen::MatrixXd &Pkp1);

        /* @filterUkfPredict
            Inputs:
                Wi: (2n+1)x1 dimensional vector of sigma vector weights
                yi: nx(2n+1) dimensional matrix of sigma vectors (each column) propagated through the system dynamics 
            Outputs:
                xkp1: nx1 predicted weighted sample mean
                Pkp1: nxn predicted weighted sample covariance
            Description:
                Function which takes in a matrix of UKF sigma vectors which have been propagated through the 
                system dynamics as follows: yi = g(Xi). Each column of the matrix corresponds to each of the
                sigma vectors being propagated through the nonlinear dynamics. The indices of each column also
                correspond each row of the sigma weights vector, which is also inluded. The function computes 
                the weighted mean and covariance based on these inputs 
        */
        bool filterUkfPredict(Eigen::VectorXd &Wi,
                              Eigen::MatrixXd &yi,
                              Eigen::MatrixXd &Qk,
                              Eigen::VectorXd &xkp1,
                              Eigen::MatrixXd &Pkp1);

        /* @filterUkfUpdate
            Inputs:
                xk: nx1 dimensional state vector
                Pk: nxn dimensional covariance matrix
                yi: nx(2n+1) dimensional matrix of sigma vectors (each column) propagated through the system dynamics
                Wi: (2n+1)x1 dimensional vector of sigma vector weights
                zi: mx(2n+1) dimensional matrix of sigma measurement vectors (each column) propagated through the nonlinear measurement model
                zk: mx1 measurement vector
                Rk: nxn measurement noise matrix 
            Outputs:
                xkp1: nx1 dimensional measurement updated state vector
                Pkp1: nxn dimensional measurement updated covariance matrix
            Description:
                Function which takes in a matrix of UKF sigma measurement vectors which have been propagated 
                through the measurement model as follows: zi = h(Xi). Each column of the matrix corresponds to
                each measurment vector being propagated through the nonlinear model. The indices of each column also
                correspond each row of the sigma weights vector, which is also inluded. The function also takes in the
                a-priori predicted state ans covariance matrices. The function next computes the measurement mean,
                followed by measurement space covariance and cross-correlation. This is used to compute the Kalman gain.
                This is used in the standard Kalman Filter measurement update equations to compute the filter updated 
                state and covariance.
        */
        bool filterUkfUpdate(Eigen::VectorXd &xk,
                             Eigen::MatrixXd &Pk,
                             Eigen::MatrixXd &yi,
                             Eigen::VectorXd &Wi,
                             Eigen::MatrixXd &zi,
                             Eigen::VectorXd &zk,
                             Eigen::MatrixXd &Rk,
                             Eigen::VectorXd &xkp1,
                             Eigen::MatrixXd &Pkp1);

    // Private Class Members/Function
    private:


};

