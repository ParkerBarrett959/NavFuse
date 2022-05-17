//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Inertial Measurement Unit                                   //
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
    if (tov <= tov_prev_) {
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
                              double &tov,
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

    // Verify Valid TOV
    if (tov <= tov_prev_) {
        // Add Logging
        return false;
    }

    // Compensate Accelerometer Measurement
    if (!compensateMeasurement(dV, tov, ba, sfa, ma)) {
        // Add Logging
        return false;
    }

    // Compensate Gyro Measurement
    if (!compensateMeasurement(dTh, tov, bg, sfg, mg)) {
        // Add Logging
        return false;
    }

    // Return Statement for Successful Compensation
    return true;
    
}

// Compensate Measurement 
bool ImuSensor::compensateMeasurement(Eigen::VectorXd &meas,
                                      double &tov,
                                      Eigen::VectorXd &b,
                                      Eigen::VectorXd &sf,
                                      Eigen::VectorXd &mis) {
                        
    // Verify Correct Dimensions
    if (meas.size() != 3) {
        // Add Logging
        return false;
    } else if (b.size() != 3) {
        // Add Logging
        return false; 
    } else if (sf.size() != 3) {
        // Add Logging
        return false;
    } else if (mis.size() != 6) {
        // Add Logging
        return false; 
    }

    // Verify Valid TOV
    if (tov <= tov_prev_) {
        // Add Logging
        return false;
    }

    // Generate Scale Factor Matrix
    Eigen::MatrixXd sfMat = Eigen::MatrixXd::Identity(3,3);
    sfMat(0,0) += sf[0];
    sfMat(1,1) += sf[1];
    sfMat(2,2) += sf[2];

    // Generate Misalignment Matrix
    Eigen::MatrixXd misMat = Eigen::MatrixXd::Identity(3,3);
    misMat(0,1) = mis[0];
    misMat(0,2) = mis[1];
    misMat(1,0) = mis[2];
    misMat(1,2) = mis[3];
    misMat(2,0) = mis[4];
    misMat(2,1) = mis[5];

    // Generate Combined Scale Factor/Misalignment Matrix
    Eigen::MatrixXd sfMis = sfMat * misMat;

    // Generate Bias Vector
    Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(3,1);
    bias(0,0) = b[0];
    bias(1,0) = b[1];
    bias(2,0) = b[2];

    // Get Current IMU Delta Time
    double deltaT = tov - tov_prev_;

    // Compensate Measurement
    meas = sfMis.inverse() * (meas - (bias * deltaT));

    // Return Statement
    return true;

}