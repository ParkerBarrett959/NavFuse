//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Inertial Measurement Unit                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a GPS sensor model.                              //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "GPS.hpp"
#include "NavUtils.hpp"

// Build Tightly Couple H Matrix
bool GpsSensor::buildHTightlyCoupled(Eigen::VectorXd &xk,
                                     Eigen::VectorXd &posInd,
                                     Eigen::MatrixXd &Hk) {

    

    // Return Statement for Successfully Building H Matrix
    return true;

}


// Build Loosely Couple H Matrix
bool GpsSensor::buildHLooselyCoupled(Eigen::VectorXd &xk,
                                     Eigen::VectorXd &posInd,
                                     Eigen::MatrixXd &Hk) {

    

    // Return Statement for Successfully Building H Matrix
    return true;

}