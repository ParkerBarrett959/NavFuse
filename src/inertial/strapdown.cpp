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
    tov_ = tov;
    if (!NavUtil_.rph2Dcm(rph, RB2N_)) {
        std::cout << "[Strapdown::initialize] Unable to compute RB2N from rph" << std::endl;
        return false; 
    }

    // Initialize Previous Class Variables
    llaPrev_ = lla;
    vNedPrev_ = vNed;
    RB2NPrev_ = RB2N_;
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
    RB2NPrev_ = RB2N_;
    tovPrev_ = tov_;
    
    // Set Current Class Variables
    tov_ = tov;
    dt_ = (tov_ - tovPrev_) / 1e6;

    // Update Attitude
    if (!updateAttitude(dTh)) {
        std::cout << "[Strapdown::integrate] Unable to update attitude" << std::endl;
        return false;
    }

    // Rotate Specific Force to Updated Attitude Frame
    Eigen::Vector3d dVN = RB2N_ * dV;

    // Gravity Compensation
    Eigen::Vector3d gN;
    if (!Gravity_.gravityNed(lla_[0], lla_[2], gN)) {
        std::cout << "[Strapdown::integrate] Unable to compute gravity" << std::endl;
        return false;
    }

    // Velocity Integration

    // Position Integration

    // Return for Successful Integration
    return true;

}

// Attitude Update
bool Strapdown::updateAttitude(Eigen::Vector3d &dTh) {

    // Get Helpful Quantities
    double dThMag = dTh.norm();
    Eigen::Matrix3d dThX;
    if (!NavUtil_.skewSymmetric(dTh, dThX)) {
        std::cout << "[Strapdown::updateAttitude] Unable to get skew symmetric" << std::endl;
        return false;
    }

    // Compute Rotation from Previous to Current Time step
    Eigen::Matrix3d B1 = (std::sin(dThMag) / dThMag) * dThX;
    Eigen::Matrix3d B2 = ((1 - std::cos(dThMag))/(dThMag * dThMag)) * dThX * dThX;
    Eigen::Matrix3d B = Eigen::Matrix3d::Identity() + B1 + B2; 

    // Update Attitude
    RB2N_ = RB2NPrev_ * B;

    // Return for Successful Attitude Update
    return true;

}