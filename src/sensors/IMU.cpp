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
bool ImuSensor::strapdownInit(Eigen::Vector3d &rInit,
                              Eigen::Vector3d &vInit,
                              Eigen::VectorXd &qB2IInit,
                              int64_t &tovInit) {

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
    if (tovInit <= 0) {
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
bool ImuSensor::strapdownIntegrate(Eigen::Vector3d &dV,
                                   Eigen::Vector3d &dTh,
                                   int64_t &tov) {

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

    // Set Current tov and dt
    tov_ = tov;
    dt_ = (tov_ - tov_prev_) / 1e6;

    // Perform IMU Measurement Compensation
    if (!compensateImu(dV, dTh, ba_, sfa_, ma_, bg_, sfg_, mg_)) {
        // Add Logging
        return false;
    }

    // Compute Rotation from Previous to Current Body Attitude
    Eigen::VectorXd qBprev2Bcurr;
    if (!computeqPrev2Curr(dTh, qBprev2Bcurr)) {
        // Add Logging
        return false;
    }

    // Update Current Attitude Estimate
    if (!updateAttitude(qBprev2Bcurr)) {
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
    Eigen::Vector3d gI;
    if (!GravityModel.simpleGravity(rI_prev_, gI)) {
        // Add Logging
        return false;
    }
    double wEMag = 2.0 * M_PI / 86400.0;
    Eigen::Vector3d wE_(0, 0, wEMag);
    Eigen::VectorXd dVg_I = dt_ * (gI - (2 * wE_.cross(vI_prev_)));
    dV_I += dVg_I;

    // Perform Velocity Integration
    vI_ = vI_prev_ + dV_I;

    // Perform Position Integration
    rI_ = rI_prev_ + (dt_ * vI_prev_) + (0.5 * dt_ * dV_I);

    // Update Previous Class Variables
    tov_prev_ = tov_;
    rI_prev_ = rI_;
    vI_prev_ = vI_;
    qB2I_prev_ = qB2I_;

    // Return True for Successful Integration
    return true;

}


// IMU Measurement Compensation 
bool ImuSensor::compensateImu(Eigen::Vector3d &dV,
                              Eigen::Vector3d &dTh,
                              Eigen::Vector3d &ba,
                              Eigen::Vector3d &sfa,
                              Eigen::VectorXd &ma,
                              Eigen::Vector3d &bg,
                              Eigen::Vector3d &sfg,
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

    // Compensate Accelerometer Measurement
    if (!compensateMeasurement(dV, ba, sfa, ma)) {
        // Add Logging
        return false;
    }

    // Compensate Gyro Measurement
    if (!compensateMeasurement(dTh, bg, sfg, mg)) {
        // Add Logging
        return false;
    }

    // Return Statement for Successful Compensation
    return true;
    
}


// Compute Quaternion from Previous Body Frame to Current Body Frame
bool ImuSensor::computeqPrev2Curr(Eigen::Vector3d &dTh,
                                  Eigen::VectorXd &qBprev2Bcurr) {

    // Check Measurement Size
    if (dTh.size() != 3) {
        // Add Logging
        return false;
    }

    // Compute Rotation Vector from Measurement
    Eigen::VectorXd phi = dTh; 

    // Compute Quaternion from Rotation Vector
    NavUtils NavUtil;
    if (!NavUtil.computeQuaternionFromRotationVec(phi, qBprev2Bcurr)) {
        // Add Logging
        return false;
    }

    // Return Statement
    return true;

}


// Update Attitude
bool ImuSensor::updateAttitude(Eigen::VectorXd &qBprev2Bcurr) {
    
    // Build 4x4 Quaternion Equivalent Matrix
    Eigen::MatrixXd qMat;
    NavUtils NavUtil;
    if (!NavUtil.buildQuaternionEquivalent(qBprev2Bcurr, qMat)) {
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
bool ImuSensor::compensateMeasurement(Eigen::Vector3d &meas,
                                      Eigen::Vector3d &b,
                                      Eigen::Vector3d &sf,
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

    // Compensate Measurement
    meas = sfMis.inverse() * (meas - (bias * dt_));

    // Return Statement
    return true;

}