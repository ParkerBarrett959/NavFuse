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

// Strapdown Initialization
bool ImuSensor::strapdownInit(Eigen::VectorXd &rInit,
                              Eigen::VectorXd &vInit,
                              Eigen::VectorXd &qB2IInit,
                              double &tovInit) {

    // Verify Correct Dimensions
    if (rInit.size() != 3) {
        // Add Logging
        return false;
    } else if (vInit.size() != 3) {
        // Add Logging
        return false;
    } else if (qB2IInit.size() != 3) {
        // Add Logging
        return false;
    }

    // Verify Valid TOV
    if (tovInit <= 0.0) {
        // Add Logging
        return false;
    }

    // Set Class Variables
    rI_ = rInit;
    vI_ = vInit;
    qB2I_ = qB2IInit;
    tov_ = tovInit;
    rI_prev_ = rInit;
    vI_prev_ = vInit;
    qB2I_prev_ = qB2IInit;
    tov_prev_ = tovInit;

    // Return Statement for Successful Initialization
    return true;

}


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

    // Compute Rotation from Previous to Current Body Attitude
    if (!computeqPrev2Curr(dTh, qBprev2Bcurr_)) {
        // Add Logging
        return false;
    };

    // Update Current Attitude Estimate
    if (!updateAttitude()) {
        // Add Logging
        return false;
    }

    // Rotate Specific Force to Inertial Frame
    Eigen::MatrixXd RB2I;
    NavUtils NavUtil;
    if (!NavUtil.computeDcmFromQuaternion(qB2I_, RB2I)) {
        // Add Logging
        return false;
    }
    Eigen::VectorXd dV_I = RB2I * dV;

    // Perform Gravity Compensation
    Gravity GravityModel;
    Eigen::VectorXd gA;
    if (!GravityModel.simpleGravity(rI_prev_, gA)) {
        // Add Logging
        return false;
    }
    dV_I += gA;


    // Perform Velocity Integration

    // Perform Position Integration


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


// Compute Quaternion from Previous Body Frame to Current Body Frame
bool ImuSensor::computeqPrev2Curr(Eigen::VectorXd &dTh,
                                  Eigen::VectorXd &qBprev2Bcurr_) {

    // Check Measurement Size
    if (dTh.size() != 3) {
        // Add Logging
        return false;
    }

    // Compute Rotation Vector from Measurement
    Eigen::VectorXd phi = dTh; 

    // Compute Quaternion from Rotation Vector
    NavUtils NavUtil;
    if (!NavUtil.computeQuaternionFromRotationVec(phi, qBprev2Bcurr_)) {
        // Add Logging
        return false;
    }

    // Return Statement
    return true;

}


// Update Attitude
bool ImuSensor::updateAttitude() {

    // Save Current Attitude to Previous Attitude
    qB2I_prev_ = qB2I_;
    
    // Build 4x4 Quaternion Equivalent Matrix
    Eigen::MatrixXd qMat;
    NavUtils NavUtil;
    if (!NavUtil.buildQuaternionEquivalent(qBprev2Bcurr_, qMat)) {
        // Add Logging
        return false;
    }

    // Update Body to Inertial Quaternion
    qB2I_ = qMat.transpose() * qB2I_prev_;

    // Normalize Quaternion
    double qB2IMag = qB2I_.norm();
    qB2I_ = qB2I_ / qB2IMag;

    // Return Statement
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