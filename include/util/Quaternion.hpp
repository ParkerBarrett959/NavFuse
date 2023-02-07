//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Quaternion Data Type                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the quaternion class.                                               //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// Quaternion Class
class Quaternion {

    // Public Class Members/Functions
    public:

        /* @isNormalized
            Inputs:
                q: quaternion type structure
            Outputs:
                normalized: boolean indicating whether input quaternion is normalized
            Description:
                Function which takes in a quaternion type structure and returns a boolean
                indicating whether the quaternion is normalized.
        */
        bool isNormalized(quaternion q) {
            bool normalized = false;
            if (magnitude(q) == 1.0) {
                normalized = true;
            }
            return normalized;
        }

        /* @normalize
            Inputs:
                q: quaternion type structure
            Outputs:
                qNormal: normalized quaternion type structure
            Description:
                Function which takes in a quaternion type structure and returns the normalized
                quaternion.
        */
        quaternion normalize(quaternion q) {
            double mag = magnitude(q);
            quaternion qNormal;
            qNormal.q0 = q.q0 / mag;
            qNormal.q1 = q.q1 / mag;
            qNormal.q2 = q.q2 / mag;
            qNormal.q3 = q.q3 / mag;
            return normalized;
        }  

    // Private Class Members/Function
    private:

        // Quaternion Type Definition
        struct {
            double q0 = 1;
            double q1 = 0;
            double q2 = 0;
            double q3 = 0;
        } quaternion;

        /* @magnitude
            Inputs:
                q: quaternion type structure
            Outputs:
                mag: scalar double magnitude of the quaternion
            Description:
                Function which takes in a quaternion type structure and returns the magnitude.
        */
        double magnitude(quaternion q) {
            return std::sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
        }

        /* @conjugate
            Inputs:
                q: quaternion type structure
            Outputs:
                qP: conjugate quaternion type structure of input q
            Description:
                Function which takes in a quaternion type structure and returns the conjugate.
        */
        quaternion conjugate(quaternion q) {
            quaternion qP;
            qP.q0 = q.q0; qP.q1 = -q.q1; qP.q2 = -q.q2; qP.q3 = -q.q3;
            return qP;
        }

        /* @inverse
            Inputs:
                q: quaternion type structure
            Outputs:
                qInv: inverse quaternion type structure of input q
            Description:
                Function which takes in a quaternion type structure and returns the inverse.
        */
        quaternion inverse(quaternion q) {
            quaternion qP = conjugate(q);
            double mag = magnitude(q);
            double magSq = mag*mag;
            quaternion qInv;
            qInv.q0 = qP.q0/magSq;
            qInv.q1 = qP.q1/magSq;
            qInv.q2 = qP.q2/magSq;
            qInv.q3 = qP.q3/magSq;
            return qInv;
        }

        /* @multiply
            Inputs:
                qA: quaternion type structure
                qB: quaternion type structure
            Outputs:
                qC: quaternion type structure product of qA and qB
            Description:
                Function which takes in two quaternions and computes qC = qA*qB.
        */
        quaternion multiply(quaternion qA, quaternion qB) {
            quaternion qC;
            qC.q0 = qA.q0*qB.q0 - qA.q1*qB.q1 - qA.q2*qB.q2 - qA.q3*qB.q3;
            qC.q1 = qA.q0*qB.q1 + qA.q1*qB.q0 - qA.q2*qB.q3 + qA.q3*qB.q2;
            qC.q2 = qA.q0*qB.q2 + qA.q1*qB.q3 + qA.q2*qB.q0 - qA.q3*qB.q1;
            qC.q3 = qA.q0*qB.q3 - qA.q1*qB.q2 + qA.q2*qB.q1 - qA.q3*qB.q0;
            return qC;
        }

};