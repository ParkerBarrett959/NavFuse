//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Gravity Functions                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of gravity models of varying fidelity.     //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "Gravity.hpp"

// Compute Simple Gravity Model
bool Gravity::simpleGravity(Eigen::Vector3d &rA,
                            Eigen::Vector3d &gA) {

    // Compute Magnitude of Position Vector
    double rSq = rA.squaredNorm();

    // Compute Gravity Vector
    gA = ((-gravityParams.G * gravityParams.mE) / rSq) * rA.normalized();

    // Return Statement for Successful Initialization
    return true;

}

// Compute Gravity in NED Frame
bool Gravity::gravityNed(double &lat,
                         double &h,
                         Eigen::Vector3d &gN) {

    // Useful Quantities
    double clat = std::cos(lat);
    double slat = std::sin(lat);
    double a = gravityParams.a;
    double b = gravityParams.b;
    double gEq = gravityParams.gEq;
    double gPole = gravityParams.gPole; 
    double f = gravityParams.f;
    double m = std::pow(gravityParams.wE, 2) * std::pow(a, 2) * b / gravityParams.kM;

    // Apply Somigliana Formula - Compute Gravity on Earth Ellipsoid
    double num = (a * gEq * std::pow(clat, 2)) + (b * gPole * std::pow(slat, 2));
    double den = std::sqrt((std::pow(a, 2) * std::pow(clat, 2)) + (std::pow(b, 2) * std::pow(slat, 2)));
    double gEarthEll = num / den;

    // Define Gravity at Altitude
    double gDown = gEarthEll * (1 - ((2/a)*(1+f+m-(2*f*std::pow(slat, 2)))*h) + (3*std::pow(h, 2)/(std::pow(a, 2))));

    // Define North Gravity Magnitude
    double gNorth = -8.08 * 1e-6 * (h / 1000) * std::sin(2 * lat);

    // Define NED Frame Gravity Vector
    gN << gNorth, 0.0, gDown;

    // Return Statement for Successful Initialization
    return true;

}