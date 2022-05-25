//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           GPS Sensor Header                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for GPS sensor class                                                    //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// IMU Sensor Class
class GpsSensor {

    // Public Class Members/Functions
    public:

        /* @buildHTightlyCoupled
            Inputs:
                xk: nx1 dimensional state vector 
                posInd: 3x1 dimensional vector of indices of position variables in the state vector 
            Outputs:
                Hk: mxn dimensional measurement Jacobian matrix
            Description:
                Function which takes in the state vector and indices corresponding to the position
                variables and computes the tightly coupled GPS model H matrix.
        */
        bool buildHTightlyCoupled(Eigen::VectorXd &xk,
                                  Eigen::VectorXd &posInd,
                                  Eigen::MatrixXd &Hk);

        /* @buildHLooselyCoupled
            Inputs:
                xk: nx1 dimensional state vector 
                posInd: 3x1 dimensional vector of indices of position variables in the state vector 
            Outputs:
                Hk: mxn dimensional measurement Jacobian matrix
            Description:
                Function which takes in the state vector and indices corresponding to the position
                variables and computes the loosely coupled GPS model H matrix.
        */
        bool buildHLooselyCoupled(Eigen::VectorXd &xk,
                                  Eigen::VectorXd &posInd,
                                  Eigen::MatrixXd &Hk);

    // Private Class Members/Function
    private:

};

