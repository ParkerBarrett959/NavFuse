//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Navigation Rotations Functions                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of helpful navigation rotation functions.  //          
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "Rotations.hpp"

// Compute Rotation from ECEF to NED
bool Rotations::computeREcef2Ned(double &lat,
                                 double &lon,
                                 Eigen::Matrix3d &RE2N) {

    // Check for Lat/Lon Out of Range
    if (std::abs(lat) > (M_PI / 2.0)) {
        std::cout << "[Rotations::computeREcef2Ned] latitude provided is outside acceptable range. " << 
                "Expected: [-pi/2, pi/2], Got: " << lat << std::endl;
        return false;
    } else if (std::abs(lon) > M_PI) {
        std::cout << "[Rotations::computeREcef2Ned] longitude provided is outside acceptable range. " << 
                "Expected: [-pi, pi], Got: " << lon << std::endl;
        return false;
    }

    // Compute Halpful Quantities
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double slon = std::sin(lon);
    double clon = std::cos(lon);

    // Compute Rotation
    RE2N << -slat*clon,   -slat*slon,   clat,
                 -slon,         clon,      0,
            -clat*clon,   -clat*slon,  -slat;
    
    // Successful Return
    return true;

}

// Compute Rotation from NED to ECEF
bool Rotations::computeRNed2Ecef(double &lat,
                                 double &lon,
                                 Eigen::Matrix3d &RN2E) {

    // Check for Lat/Lon Out of Range
    if (std::abs(lat) > (M_PI / 2.0)) {
        std::cout << "[Rotations::computeRNed2Ecef] latitude provided is outside acceptable range. " << 
                "Expected: [-pi/2, pi/2], Got: " << lat << std::endl;
        return false;
    } else if (std::abs(lon) > M_PI) {
        std::cout << "[Rotations::computeRNed2Ecef] longitude provided is outside acceptable range. " << 
                "Expected: [-pi, pi], Got: " << lon << std::endl;
        return false;
    }

    // Compute Halpful Quantities
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double slon = std::sin(lon);
    double clon = std::cos(lon);

    // Compute Rotation
    RN2E << -slat*clon,   -slon,   -clat*clon,
            -slat*slon,    clon,   -clat*slon,
                  clat,       0,        -slat;
    
    // Successful Return
    return true;

}