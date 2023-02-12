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

// NavFuse Classes
#include "DirectionCosinesMatrix.hpp"
#include "EulerAngles.hpp"

// Quaternion Type Definition
struct quaternion_t {
    double q0 = 1;    // Scalar Quaternion Element
    double q1 = 0;    // Vector Quaternion Element 1
    double q2 = 0;    // Vector Quaternion Element 2
    double q3 = 0;    // Vector Quaternion Element 3
};

// Quaternion Class
class Quaternion {

    // Public Class Members/Functions
    public:
        
        /* @isNormalized
            Inputs:
                q: constant quaternion type structure
            Outputs:
                normalized: boolean indicating whether input quaternion is normalized
            Description:
                Function which takes in a quaternion type structure and returns a boolean
                indicating whether the quaternion is normalized.
        */
        bool isNormalized(const quaternion_t& q);

        /* @normalize
            Inputs:
                q: constant quaternion type structure
            Outputs:
                qNormal: normalized quaternion type structure
            Description:
                Function which takes in a quaternion type structure and returns the normalized
                quaternion.
        */
        quaternion_t normalize(const quaternion_t& q);

        /* @toDcm
            Inputs:
                q: constant quaternion type representing equivalent rotation
            Outputs:
                dcm: direction cosines matrix type
            Description:
                Function which takes in a quaternion type and returns the equivalent
                direction cosines matrix representation
        */
        directionCosinesMatrix_t toDcm(const quaternion_t& q);

        /* @toEuler
            Inputs:
                q: constant quaternion type representing equivalent rotation
            Outputs:
                rpy: Euler Angles type
            Description:
                Function which takes in a quaternion type and returns the equivalent
                Euler Angle representation
        */
        eulerAngles_t toEuler(const quaternion_t& q);

    // Private Class Members/Function
    private:

        /* @conjugate
            Inputs:
                q: quaternion type structure
            Outputs:
                qP: conjugate quaternion type structure of input q
            Description:
                Function which takes in a quaternion type structure and returns the conjugate.
        */
        quaternion_t conjugate(const quaternion_t& q);
        
        /* @inverse
            Inputs:
                q: constant quaternion type structure
            Outputs:
                qInv: inverse quaternion type structure of input q
            Description:
                Function which takes in a quaternion type structure and returns the inverse.
        */
        quaternion_t inverse(const quaternion_t& q);
        
        /* @magnitude
            Inputs:
                q: constant quaternion type structure
            Outputs:
                mag: scalar double magnitude of the quaternion
            Description:
                Function which takes in a quaternion type structure and returns the magnitude.
        */
        double magnitude(const quaternion_t& q) {
            return std::sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
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
        quaternion_t multiply(const quaternion_t& qA,
                              const quaternion_t& qB);

};