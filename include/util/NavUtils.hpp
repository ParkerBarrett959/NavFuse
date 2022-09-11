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

        /* @computeDcmFromQuaternion
            Inputs:
                qA2B: 4x1 dimensional quaternion relating reference frame A to frame B. The first element is the scalar element.
            Outputs:
                RA2B: 3x3 dimensional direction cosines matrix relating reference frame A to frame B
            Description:
                Function which takes in a quaternion and computes the direction cosines matrix.
        */
        bool computeDcmFromQuaternion(Eigen::VectorXd &qA2B,
                                      Eigen::MatrixXd &RA2B);

        /* @computeQuaternionFromRotationVec
            Inputs:
                phi: 3x1 dimensional rotation vector
            Outputs:
                qA2B: 4x1 dimensional quaternion relating reference frame A to frame B. The first element is the scalar element.
            Description:
                Function which takes in a rotation vector and computes the quaternion.
        */
        bool computeQuaternionFromRotationVec(Eigen::VectorXd &phi,
                                              Eigen::VectorXd &qA2B);

        /* @computeQuaternionFromRotationVec
            Inputs:
                qA2B: 4x1 dimensional quaternion relating reference frame A to frame B. The first element is the scalar element.
            Outputs:
                QA2B: 4x4 dimensional quaternion equivalent matrix relating reference frame A to frame B
            Description:
                Function which takes in a quaternion and computes the matrix equivalent.
        */
        bool buildQuaternionEquivalent(Eigen::VectorXd &qA2B,
                                       Eigen::MatrixXd &QA2B);

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

        /* @rph2Dcm
            Inputs:
                rph: 3x1 dimensional vector containing roll pitch and heading [rad]
            Outputs:
                RB2N: 3x3 dimensional DCM relating the body frame to the NED frame
            Description:
                Function which takes in roll pitch and heading of the body frame relative to the NED frame and
                computes the DCM RB2N.
        */
        bool rph2Dcm(Eigen::Vector3d &rph,
                     Eigen::Matrix3d &RB2N);

        /* @Dcm2Rph
            Inputs:
                RB2N: 3x3 dimensional DCM relating the body frame to the NED frame
            Outputs:
                rph: 3x1 dimensional vector containing roll pitch and heading [rad]
            Description:
                Function which takes in the DCM relating the body frame to the NED frame and computes the roll,
                pitch and heading.
        */
        bool dcm2Rph(Eigen::Matrix3d &RB2N,
                     Eigen::Vector3d &rph);

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

