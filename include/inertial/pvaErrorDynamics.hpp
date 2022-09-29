//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                Inertial Navigation PVA Error Dynamics                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial navigation filter position, velocity, attitude error       //
//              dynamics class.                                                                     //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "NavUtils.hpp"
#include "Gravity.hpp"

// Inertial Navigation PVA Error Dynamics Class
class PvaErrorDynamics {

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

        /* @ecefErrorDynamics
            Inputs:
                aE: 3x1 vector of sensed accelerations in the ECEF frame [m/s^2]
                GE: 3x3 Gravity Gradient Matrix
                RS2E: 3x3 rotation matrix relating the sensor frame to the ECEF frame
                OIE_E: 3x3 matrix of cross product equivalent of angular rate of ECEF wrt inertial expressed in ECEF 
            Outputs:
                F: 9x9 ECEF frame error model state transition matrix
                G: 9x9 ECEF frame error model forcing matrix
            Description:
                Function which computes the ECEF frame inertial navigation error dynamics.
                The error state vector, dx, is [attitude; velocity; position], and the forcing 
                function vector, du, is [angular rate; acceleration; gravity]. The general form 
                of the error dynamics are: d/dt(dx) = F * dx + G * du. The error dynamics 
                equations are defined in "Inertial Navigation Systems with Geodetic Applications", 
                5.3.2 (Jekeli, de Gruyter).
        */
        bool ecefErrorDynamics(Eigen::Vector3d &aE,
                               Eigen::Matrix3d &GE,
                               Eigen::Matrix3d &RS2E,
                               Eigen::Matrix3d &OIE_E,
                               Eigen::MatrixXd &F,
                               Eigen::MatrixXd &G);

        /* @nedErrorDynamics
            Inputs:
                aN: 3x1 vector of sensed accelerations in the NED frame [m/s^2]
                GN: 3x3 Gravity Gradient Matrix
                RS2N: 3x3 rotation matrix relating the sensor frame to the NED frame 
                lla: 3x1 vector containing latitude, longitude and altitude
                llaDot: 3x1 vector containing time derivatives of latitude, longitude and altitude
                llaDDot: 3x1 vector containing second time derivatives of latitude, longitude and altitude
            Outputs:
                F: 9x9 NED frame error model state transition matrix
                G: 9x9 NED frame error model forcing matrix
            Description:
                Function which computes the NED frame inertial navigation error dynamics.
                The error state vector, dx, is [attitude; velocity; position], and the forcing 
                function vector, du, is [angular rate; acceleration; gravity]. The general form 
                of the error dynamics are: d/dt(dx) = F * dx + G * du. All elements of the gravity
                gradient matrix, expect the [3,3] element, are assumed equal to zero. The error dynamics 
                equations are defined in "Inertial Navigation Systems with Geodetic Applications", 
                5.3.3 (Jekeli, de Gruyter).
        */
        bool nedErrorDynamics(Eigen::Vector3d &aN,
                              Eigen::Matrix3d &GN,
                              Eigen::Matrix3d &RS2N,
                              Eigen::Vector3d &lla,
                              Eigen::Vector3d &llaDot,
                              Eigen::Vector3d &llaDDot,
                              Eigen::MatrixXd &F,
                              Eigen::MatrixXd &G);

    // Private Class Members/Function
    private:

        // Object Instantiations
        ::Gravity Gravity;

};

