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

    // Private Class Members/Function
    private:

};