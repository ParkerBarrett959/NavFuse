//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Navigation Utility Functions                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for navigation utilities class.                                         //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "Gravity.hpp"
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Nav Utilities Class
class NavUtils {

    // Public Class Members/Functions
    public:

        /* @skewSymmetric
            Inputs:
                vec: 3x1 dimensional vector
            Outputs:
                vecX: 3x3 dimensional skew symmetric matrix
            Description:
                Function which takes in a vector and computes the skew symmetric (cross-product equivalent)
                matrix.
        */
        bool skewSymmetric(Eigen::Vector3d &vec,
                           Eigen::Matrix3d &vecX);

        /* @strapdownRk4
            Inputs:
                ykm1: 6x1 dimensional vector containing previous state (pos/vel) to be integrated
                dt: scalar double numerical integration step size
                dVN: 3x1 dimensional delta velocity measurement in the NED frame [m/s]
                gN: 3x1 dimensional gravity vector expressed in the NED frame [m/s^2] 
            Outputs:
                yk: 6x1 dimensional vector containing integrated state (pos/vel)
            Description:
                Function which performs a fourth order runge kutta numerical integration of the NED
                frame strapdown dynamics.
        */
        bool strapdownRk4(Eigen::VectorXd &ykm1,
                          double &dt,
                          Eigen::Vector3d &dVN,
                          Eigen::Vector3d &gN,
                          Eigen::VectorXd &yk);

    // Private Class Members/Function
    private:

        // Class Instantiations
        ::Gravity Gravity;

        /* @strapdownDynamics
            Inputs:
                y: 6x1 dimensional vector of states
                dVN: 3x1 dimensional delta velocity measurement in the NED frame [m/s]
                gN: 3x1 dimensional gravity vector expressed in the NED frame [m/s^2]
                dt: scalar double numerical integration step size
            Outputs:
                f: 6x1 dimensional derivative evaluated at state y
            Description:
                Function which computes the strapdown dynamics given a state, y.
        */
        bool strapdownDynamics(Eigen::VectorXd &y,
                               Eigen::VectorXd &f,
                               Eigen::Vector3d &dVN,
                               Eigen::Vector3d &gN,
                               double &dt);

};

