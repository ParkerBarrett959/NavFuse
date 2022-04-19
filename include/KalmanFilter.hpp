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

    // Private Class Members/Function
    private:


};

