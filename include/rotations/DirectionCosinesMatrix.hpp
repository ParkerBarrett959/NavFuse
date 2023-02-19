//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Direction Cosines Matrix Data Type                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the direction cosines matrix class.                                 //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <array>
#include <cmath>
#include <Eigen/Dense>

// NavFuse Classes
#include "Quaternion.hpp"
#include "EulerAngles.hpp"
#include "RotationVector.hpp"

// Direction Cosines Matrix Class
class DirectionCosinesMatrix {

    // Public Class Members/Functions
    public:

        // Direction Cosines Matrix Elements
        std::array<std::array<double, 3>, 3> dcm_;

        /* @DirectionCosinesMatrix
            Inputs:
                dcm: 2D std::array containing the elements of the DCM. Elements are in row major format.
            Outputs:
            Description:
                Direction Cosines Matrix class constructor which initializes the DCM elements.
        */
        DirectionCosinesMatrix(std::array<std::array<double, 3>, 3> dcm) : dcm_(dcm) {};

        /* @* (Multiplication Overload Operator)
            Inputs:
                R2: Direction Cosines Matrix Type Object
            Outputs:
                R3: Direction Cosines Matrix Type Object
            Description:
                Function overload which returns the product of two Direction Cosines Matrix type objects as a new 
                Direction Cosines Matrix type object. This function allows users to easily chain togetehr multiple 
                rotations without having to access specific data elements of each Direction Cosines Matrix class.
        */
        DirectionCosinesMatrix operator* (DirectionCosinesMatrix R2) {
            return multiply(R2);
        };

        /* @getDirectionCosinesMatrix
            Inputs:
            Outputs:
                R: Eigen::Matrix3d containing the current DCM elements - R = [R11. R12, R13; R21, R22, R23; R31, R32, R33]
            Description:
                Function which returns the current DCMelements as an Eigen::Matrix3d.
        */
        Eigen::Matrix3d getDirectionCosinesMatrix();

        /* @passiveRotateVector
            Inputs:
                vecIn: Eigen::Vector3d type input vector
            Outputs:
                vecOut: Eigen::Vector3d type output vector
            Description:
                Function which takes an Eigen::Vector3d type input vector and performs the passive direction cosines
                matrix rotation operation on the vector and outputs the resulting, rotated Eigen::Vector3d vector. The
                passive rotation is the rotation in which the coordinate system is rotated with respect to the 
                point.
        */
        Eigen::Vector3d passiveRotateVector(const Eigen::Vector3d& vecIn);

        /* @activeRotateVector
            Inputs:
                vecIn: Eigen::Vector3d type input vector
            Outputs:
                vecOut: Eigen::Vector3d type output vector
            Description:
                Function which takes an Eigen::Vector3d type input vector and performs the active direction cosines
                matrix rotation operation on the vector and outputs the resulting, rotated Eigen::Vector3d vector. The
                active rotation is the rotation in which the point itself is rotated with respect to the coordinate
                system.
        */
        Eigen::Vector3d activeRotateVector(const Eigen::Vector3d& vecIn);

        /* @transpose
            Inputs:
                No Inputs
            Outputs:
                No Outputs
            Description:
                Function which computes the transpose of the direction cosines matrix
        */
        void transpose();

    // Private Class Members/Function
    private:

        /* @multiply
            Inputs:
                RB: Direction Cosines Matrix type structure.
            Outputs:
                RC: Direction Cosines Matrix type structure.
            Description:
                Function which takes in multiplies the current Direction Cosines Matrix with the input argument
                RB and returns the product RC = dcm_*RB.
        */
        DirectionCosinesMatrix multiply(const DirectionCosinesMatrix& qB);

};