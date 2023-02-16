//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Rotation Vector Data Type                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the Rotation Vector class.                                             //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// Rotation Vector Class
class RotationVector {

    // Public Class Members/Functions
    public:

        // Rotation Vector Elements
        std::array<double, 3> rv_;    // [rad]

        /* @RotationVector
            Inputs:
                rv: 3x1 std::array rotation vector[rad]
            Outputs:
            Description:
                Rotation Vector class constructor which initializes the Rotation Vector elements.
        */
        RotationVector(std::array<double, 3> rv) : rv_(rv) {};


    // Private Class Members/Function
    private:

};