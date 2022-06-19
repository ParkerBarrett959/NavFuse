//////////////////////////////////////////////////////////////////////////////////////////////////////
//                         Inertial Navigation Initialization and Alignment                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines the inertial navigation system alignment and     //
//              initialization algorithms.                                                          //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <cmath>
#include "initialization.hpp"

// Coarse Attitude Initialization
bool Initialization::coarseAttitudeAlignment(double &lat,
                                             double &wE,
                                             double &g,
                                             Eigen::Vector3d &aB,
                                             Eigen::Vector3d &wIB_B,
                                             Eigen::Matrix3d &RB2N) {

    // Define Useful Quantities
    double slat = std::sin(lat);
    double clat = std::cos(lat);

    // Define A Matrix
    Eigen::Matrix3d A;
    A << 0.0,        0.0,         -g, 
         wE*clat,    0.0,         -wE*slat, 
         0.0,        g*wE*clat,   0.0;

    // Define B Matrix
    Eigen::Matrix3d B = Eigen::Matrix3d::Zero();
    B.row(0) = aB.transpose();
    B.row(1) = wIB_B.transpose();
    B.row(2) = (-aB.cross(wIB_B)).transpose();

    // Compute Rotation from Body to NED Navigation Frame
    RB2N = A.inverse() * B;

    // Return Statement for Successfully Building H Matrix
    return true;

}