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
#include "RotationVector.hpp"

// Quaternion Class
class Quaternion {

    // Public Class Members/Functions
    public:

        // Quaternion Elements
        double q0_, q1_, q2_, q3_;

        /* @Quaternion
            Inputs:
                q0: Scalar quaternion element
                q1: First vector quaternion element
                q2: Second vector quaternion element
                q3: Third vector quaternion element
            Outputs:
            Description:
                Quaternion class constructor which initializes the quaternion elements.
        */
        Quaternion(double q0, double q1, double q2, double q3) : q0_(q0), q1_(q1), q2_(q2), q3_(q3) {};
        
        /* @isNormalized
            Inputs:
            Outputs:
                isNormalized: Boolean indicating whether the quaternion is normalized
            Description:
                Function which returns a boolean indicating whether the quaternion is normalized.
        */
        bool isNormalized();

        /* @normalize
            Inputs:
            Outputs:
            Description:
                Function which normalizes the quaternion.
        */
        void normalize();

        /* @toDcm
            Inputs:
            Outputs:
                dcm: Direction cosines matrix object
            Description:
                Function which converts the quaternion to the equivalent direction cosines matrix representation
        */
        DirectionCosinesMatrix toDcm();

        /* @toEuler
            Inputs:
            Outputs:
                euler: Euler Angles object
            Description:
                Function which converts the quaternion to the equivalent Euler Angle representation
        */
        EulerAngles toEuler();

        /* @toRotationVector
            Inputs:
            Outputs:
                rv: Rotation Vector object
            Description:
                Function which converts the quaternion to the equivalent rotation vector representation
        */
        RotationVector toRotationVector();

    // Private Class Members/Function
    private:

        /* @conjugate
            Inputs:
            Outputs:
                q: Quaternion object containing conjugate.
            Description:
                Function which returns a quaternion with value set to the conjugate of the
                current quaternion.
        */
        Quaternion conjugate();
        
        /* @inverse
            Inputs:
            Outputs:
                q: Quaternion object containing inverse.
            Description:
                Function which returns a quaternion with value set to the inverse of the
                current quaternion.
        */
        Quaternion inverse();
        
        /* @magnitude
            Inputs:
            Outputs:
                mag: scalar double magnitude of the quaternion
            Description:
                Function which returns the magnitude of the quaternion.
        */
        double magnitude() {
            return std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        }

        /* @multiply
            Inputs:
                qB: quaternion type structure
            Outputs:
                qC: Quaternion object containing product of two quaternions.
            Description:
                Function which takes in multiplies the current quaternion with the input argument
                qB and returns the product qC = q_*qB.
        */
        Quaternion multiply(const Quaternion& qB);

};