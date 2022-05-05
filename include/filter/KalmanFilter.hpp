//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Kalman Filter Header                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for Kalman Filter class                                                 //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
                z: mx1 measurement vector
                H: mxn discrete time measurement Jacobian matrix
                Rk: nxn measurement noise matrix
                K: Kalman Gain matrix
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
                          Eigen::MatrixXd &K,
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
                              Eigen::VectorXd &xkp1,
                              Eigen::MatrixXd &Pkp1);

    // Private Class Members/Function
    private:


};

