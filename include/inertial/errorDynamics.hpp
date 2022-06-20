//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Inertial Navigation Error Dynamics                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial navigation filter error dynamics class.                    //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <Eigen/Dense>
#include "NavUtils.hpp"

// Inertial Navigation Measurement Compensator Class
class InertialErrorDynamics {

    // Public Class Members/Functions
    public:

        /* @inertialErrorDynamics
            Inputs:
                aI: 3x1 vector of sensed accelerations in the inertial frame [m/s^2]
                GI: 3x3 Gravity Gradient Matrix
                RS2I: 3x3 rotation matrix relating the sensor frame to the inertial frame
            Outputs:
                F: 9x9 inertial frame error model state transition matrix
                G: 9x9 inertial frame error model forcing matrix
            Description:
                Function which computes the inertial frame inertial navigation error dynamics.
                The error state vector, dx, is [attitude; velocity; position], and the forcing 
                function vector, du, is [angular rate; acceleration; gravity]. The general form 
                of the error dynamics are: d/dt(dx) = F * dx + G * du. The error dynamics 
                equations are defined in "Inertial Navigation Systems with Geodetic Applications", 
                5.3.1 (Jekeli, de Gruyter).
        */
        bool inertialErrorDynamics(Eigen::Vector3d &aI,
                                   Eigen::Matrix3d &GI,
                                   Eigen::Matrix3d &RS2I,
                                   Eigen::MatrixXd &F,
                                   Eigen::MatrixXd &G);

    // Private Class Members/Function
    private:

};

