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

// Euler Angles Class
class EulerAngles {

    // Public Class Members/Functions
    public:

        // Euler Angle Elements
        double roll_, pitch_, yaw_;    // [rad]

        /* @EulerAngles
            Inputs:
                roll: Roll Angle [rad]
                pitch: Pitch Angle [rad]
                yaw: Yaw Angle [rad]
            Outputs:
            Description:
                Euler Angles class constructor which initializes the Euler Angle elements.
        */
        EulerAngles(double roll, double pitch, double yaw) : roll_(roll), pitch_(pitch), yaw_(yaw) {};


    // Private Class Members/Function
    private:

};