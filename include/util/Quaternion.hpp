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
        
        /* @* (Multiplication Overload Operator)
            Inputs:
                q2: Quaternion Type Object
            Outputs:
                q3: Quaternion Type Object
            Description:
                Function overload which returns the product of two quaternion type objects as a new quaternion 
                type object. This function allows users to easily chain togetehr multiple rotations without having
                to access specific data elements of each quaternion class.
        */
        Quaternion operator* (Quaternion q2) {
            return multiply(q2);
        };
        
        /* @getQuaternion
            Inputs:
            Outputs:
                q: Eigen::Vector4d containing the current quaternion elements - q = [q0, q1, q2, q3]
            Description:
                Function which returns the current quaternion elements as an Eigen::Vector4d.
        */
        Eigen::Vector4d getQuaternion();

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

        /* @rotateVector
            Inputs:
                vecIn: Eigen::Vector3d type input vector
            Outputs:
                vecOut: Eigen::Vector3d type output vector
            Description:
                Function which takes an Eigen::Vector3d type input vector and performs the quaternion rotation operation
                on the vector and outputs the resulting, rotated Eigen::Vector3d vector.
        */
        Eigen::Vector3d rotateVector(const Eigen::Vector3d& vecIn);

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