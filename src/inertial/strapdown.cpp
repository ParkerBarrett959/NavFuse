//////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Inertial Navigation Strapdown Integration                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines the inertial navigation strapdown                //
//              integration algorithms.                                                             //   
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "strapdown.hpp"

// Initialize Strapdown Solution
bool Strapdown::initialize(Eigen::Vector3d &lla,
                           Eigen::Vector3d &vNed,
                           Eigen::Vector3d &rph,
                           int64_t &tov) {

    // Initialize Current Class Variables
    lla_ = lla;
    vNed_ = vNed;
    rph_ = rph;
    tov_ = tov;

    // Initialize Previous Class Variables
    llaPrev_ = lla;
    vNedPrev_ = vNed;
    rphPrev_ = rph;
    tovPrev_ = tov;

    // Return for Successful Initialization
    return true;

}

// Strapdown Integration
bool Strapdown::integrate(Eigen::Vector3d &dV,
                          Eigen::Vector3d &dTh,
                          int64_t &tov) {

    // Update Class Variables from Previous Step
    llaPrev_ = lla_;
    vNedPrev_ = vNed_;
    rphPrev_ = rph_;
    tovPrev_ = tov_;

    // Update Attitude

    // Rotate Specific Force to Updated Attitude Frame

    // Gravity Compensation

    // Velocity Integration

    // Position Integration

    // Return for Successful Integration
    return true;

}