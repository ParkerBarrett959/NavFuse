//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Navigation Rotations Functions                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for rotations class.                                                    //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "Gravity.hpp"

// Rotations Class
class Rotations {

    // Public Class Members/Functions
    public:

        /* @computeREcef2Ned
            Inputs:
                lat: scalar geodetic latitude  [rad]
                lon: scalar geodetic longitude [rad]
            Outputs:
                RE2N: 3x3 direction cosines matrix relating ECEF to the NED frame.
            Description:
                Function which takes in latitude and longitude and computes the direction cosines matrix
                from relating the Earth Centered Earth Fixed (ECEF) reference frame to the North-East-Down
                (NED) reference frame.
        */
        bool computeREcef2Ned(double &lat,
                              double &lon,
                              Eigen::Matrix3d &RE2N);

        /* @computeRNed2Ecef
            Inputs:
                lat: scalar geodetic latitude  [rad]
                lon: scalar geodetic longitude [rad]
            Outputs:
                Rn2E: 3x3 direction cosines matrix relating NED to the ECEF frame.
            Description:
                Function which takes in latitude and longitude and computes the direction cosines matrix
                from relating the North-East-Down (NED) reference frame to the Earth Centered Earth Fixed 
                (ECEF) reference frame.
        */
        bool computeRNed2Ecef(double &lat,
                              double &lon,
                              Eigen::Matrix3d &RN2E);

        /* @ecef2Lla
            Inputs:
                rE: 3x1 dimensional ECEF position vector [m]
            Outputs:
                lat: scalar geodetic latitude  [rad]
                lon: scalar geodetic longitude [rad]
                alt: scalar altitude above earth [m]
            Description:
                Function which takes in a position vector expressed in the earth-centered earth-fixed
                reference frame and computes the latitude, longitude and altitude of the position. The
                WGS-84 earth model is assumed.
        */
        bool ecef2Lla(Eigen::Vector3d &rE,
                      double &lat,
                      double &lon,
                      double &alt);

    // Private Class Members/Function
    private:

        // Utility Class Instantiations
        Gravity Gravity_;

};