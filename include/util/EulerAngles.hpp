//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Euler Angles Data Type                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the Euler Angles class.                                             //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// Euler Angles Type Definition
struct eulerAngles_t {
    double roll = 0.0;      // Roll Angle [rad]
    double pitch = 0.0;     // Pitch Angle [rad]
    double yaw = 0.0;       // Yaw Angle [rad]
};

// Euler Angles Class
class EulerAngles {

    // Public Class Members/Functions
    public:


    // Private Class Members/Function
    private:

};