//////////////////////////////////////////////////////////////////////////////////////////////////////
//                              Navigation Attitude Representation Functions                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for attitude representations class.                                     //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Attitude Class
class Attitude {

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

        /* @buildQuaternionEquivalent
            Inputs:
                qA2B: 4x1 dimensional quaternion relating reference frame A to frame B. The first element is the scalar element.
            Outputs:
                QA2B: 4x4 dimensional quaternion equivalent matrix relating reference frame A to frame B
            Description:
                Function which takes in a quaternion and computes the matrix equivalent.
        */
        bool buildQuaternionEquivalent(Eigen::VectorXd &qA2B,
                                       Eigen::MatrixXd &QA2B);

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

    // Private Class Members/Function
    private:

};