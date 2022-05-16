//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Inertial Measurment Unit                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines an inertial measurement unit sensor model. The   //
//              class contains functions for performing strapdown integration, measurment           //
//              compensation, and other core IMU functionalities.                                   //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "IMU.hpp"

// Strapdown Integration
bool ImuSensor::strapdownIntegrate(Eigen::VectorXd &dV,
                                   Eigen::VectorXd &dTh,
                                   double &tov) {

    // Verify Correct Measurment Dimensions
    if (dV.size() != 3) {
        // Add Logging
        return false;
    } else if (dTh.size() != 3) {
        // Add Logging
        return false;
    }

    // Verify Valid TOV
    if (tov <= tov_prev) {
        // Add Logging
        return false;
    }

    // Insert Strapdown Integration Algorithm

    // Return True for Successful Integration
    return true;

}


// IMU Measurement Compensation 
bool ImuSensor::compensateImu(Eigen::VectorXd &dV,
                              Eigen::VectorXd &dTh,
                              Eigen::VectorXd &ba,
                              Eigen::VectorXd &sfa,
                              Eigen::VectorXd &ma,
                              Eigen::VectorXd &bg,
                              Eigen::VectorXd &sfg,
                              Eigen::VectorXd &mg) {

    // Verify Correct Dimensions
    if (dV.size() != 3) {
        // Add Logging
        return false;
    } else if (dTh.size() != 3) {
        // Add Logging
        return false;
    } else if (ba.size() != 3) {
        // Add Logging
        return false; 
    } else if (sfa.size() != 3) {
        // Add Logging
        return false;
    } else if (ma.size() != 6) {
        // Add Logging
        return false; 
    } else if (bg.size() != 3) {
        // Add Logging
        return false; 
    } else if (sfg.size() != 3) {
        // Add Logging
        return false;
    } else if (mg.size() != 6) {
        // Add Logging
        return false; 
    }

    // Insert Compensation Algorithm

    // Return Statement for Successful Compensation
    return true;
    
}