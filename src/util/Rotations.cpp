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

// Compute Lat/Lon/Alt from ECEF Position
bool Rotations::ecef2Lla(Eigen::Vector3d &rE,
                         double &lat,
                         double &lon,
                         double &alt) {
    
    // Unpack Inputs
    double x = rE[0];
    double y = rE[1];
    double z = rE[2];
    
    // Compute WGS-84 Model Quantities
    double a = Gravity_.gravityParams.a;
    double b = Gravity_.gravityParams.b;
    double f = Gravity_.gravityParams.f;
    double e = std::sqrt(1 - std::pow(1-f, 2));
    double ep = std::sqrt((std::pow(a, 2) - std::pow(b, 2)) / std::pow(b, 2));
    double p = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    double th = std::atan2(a * z, b * p);

    // Compute Longitude
    lon = std::atan2(y, x);

    // Compute Latitude
    double latY = z + (std::pow(ep, 2) * b * std::pow(std::sin(th), 3));
    double latX = p - (std::pow(e, 2) * a * std::pow(std::cos(th), 3));
    lat = std::atan2(latY, latX);

    // Compute Altitude
    double N = a / std::sqrt(1 - (std::pow(e, 2) * std::pow(std::sin(lat), 2)));
    alt = (p / std::cos(lat)) - N;

    // Check Longitude Range [-pi, pi]
    lon = std::fmod(lon, 2.0 * M_PI);
    if (lon > M_PI) {
        lon -= (2.0 * M_PI);
    }
    
    // Successful Return
    return true;

}

// Compute ECEF Position from Lat/Lon/Alt
bool Rotations::lla2Ecef(double &lat,
                         double &lon,
                         double &alt,
                         Eigen::Vector3d &rE) {
    
    // Check for Lat/Lon Out of Range
    if (std::abs(lat) > (M_PI / 2.0)) {
        std::cout << "[Rotations::lla2Ecef] latitude provided is outside acceptable range. " << 
                "Expected: [-pi/2, pi/2], Got: " << lat << std::endl;
        return false;
    } else if (std::abs(lon) > M_PI) {
        std::cout << "[Rotations::lla2Ecef] longitude provided is outside acceptable range. " << 
                "Expected: [-pi, pi], Got: " << lon << std::endl;
        return false;
    }
    
    // Compute WGS-84 Model Quantities
    double a = Gravity_.gravityParams.a;
    double f = Gravity_.gravityParams.f;
    double e = std::sqrt(1 - std::pow(1-f, 2));

    // Compute Prime Vertical Radius of Curvature
    double N = a / std::sqrt(1 - (std::pow(e, 2) * std::pow(std::sin(lat), 2)));

    // Compute ECEF Position
    rE[0] = (N + alt) * std::cos(lat) * std::cos(lon);
    rE[1] = (N + alt) * std::cos(lat) * std::sin(lon);
    rE[2] = (((1 - std::pow(e, 2)) * N) + alt) * std::sin(lat);
    
    // Successful Return
    return true;

}