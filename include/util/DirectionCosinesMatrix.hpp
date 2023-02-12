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
#include <cmath>
#include <Eigen/Dense>

// Direction Cosines Matrix Type Definition
struct directionCosinesMatrix_t {
    double dcm [3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
};

// Direction Cosines Matrix Class
class DirectionCosinesMatrix {

    // Public Class Members/Functions
    public:


    // Private Class Members/Function
    private:

};