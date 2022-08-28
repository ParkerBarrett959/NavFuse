//////////////////////////////////////////////////////////////////////////////////////////////////////
//                             Inertial Navigation Measurement Compensation                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines the inertial navigation system measurement       //
//              compensation algorithms.                                                            //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "compensator.hpp"

// Set Accelerometer Errors
bool Compensator::setAccelerometerErrors(Eigen::Vector3d &baEst,
                                         Eigen::VectorXd &maEst,
                                         Eigen::Vector3d &sfaEst) {

    // Check for Correct Input Sizes
    if (maEst.size() != 6) {
        std::cout << "[Compensator::setAccelerometerErrors] misalignment has incorrect dimensions: Expected " << 
                "6x1, Got " << maEst.size() << "x1" << std::endl;
        return false;
    }

    // Set Class Variables
    ba_ = baEst;
    ma_ = maEst;
    sfa_ = sfaEst;

    // Return for Successful Set Accelerometer Errors
    return true;

}

// Set Gyroscope Errors
bool Compensator::setGyroscopeErrors(Eigen::Vector3d &bgEst,
                                     Eigen::VectorXd &mgEst,
                                     Eigen::Vector3d &sfgEst) {

    // Check for Correct Input Sizes
    if (mgEst.size() != 6) {
        std::cout << "[Compensator::setGyroscopeErrors] misalignment has incorrect dimensions: Expected " << 
                "6x1, Got " << mgEst.size() << "x1" << std::endl;
        return false;
    }

    // Set Class Variables
    bg_ = bgEst;
    mg_ = mgEst;
    sfg_ = sfgEst;

    // Return for Successful Set Gyroscope Errors
    return true;

}

// Get Accelerometer Errors
bool Compensator::getAccelerometerErrors(Eigen::Vector3d &baEst,
                                         Eigen::VectorXd &maEst,
                                         Eigen::Vector3d &sfaEst) {

    // Check for Correct Input Sizes
    if (maEst.size() != 6) {
        std::cout << "[Compensator::getAccelerometerErrors] misalignment has incorrect dimensions: Expected " << 
                "6x1, Got " << maEst.size() << "x1" << std::endl;
        return false;
    }

    // Get Class Variables
    baEst = ba_;
    maEst = ma_;
    sfaEst = sfa_;
    
    // Return for Successful Get Accelerometer Errors
    return true;

}

// Get Gyroscope Errors
bool Compensator::getGyroscopeErrors(Eigen::Vector3d &bgEst,
                                     Eigen::VectorXd &mgEst,
                                     Eigen::Vector3d &sfgEst) {

    // Check for Correct Input Sizes
    if (mgEst.size() != 6) {
        std::cout << "[Compensator::getGyroscopeErrors] misalignment has incorrect dimensions: Expected " << 
                "6x1, Got " << mgEst.size() << "x1" << std::endl;
        return false;
    }

    // Get Class Variables
    bgEst = bg_;
    mgEst = mg_;
    sfgEst = sfg_;
    
    // Return for Successful Get Accelerometer Errors
    return true;

}

// Accelerometer Compensation
bool Compensator::compensateAccelerometer(Eigen::Vector3d &dV) {

    // Compensate Accelerometer
    std::string measType = "accel";
    if (!compensateMeasurement(dV, measType)) {
        std::cout << "[Compensator::compensateAccelerometer] Failed to compensate accelerometer" << std::endl;
        return false;
    }

    // Return Statement for Successful Accelerometer Compensation
    return true;

}

// Gyroscope Compensation
bool Compensator::compensateGyroscope(Eigen::Vector3d &dTh) {

    // Compensate Gyroscope
    std::string measType = "gyro";
    if (!compensateMeasurement(dTh, measType)) {
        std::cout << "[Compensator::compensateGyroscope] Failed to compensate gyroscope" << std::endl;
        return false;
    }

    // Return Statement for Successful Gyroscope Compensation
    return true;

}

// Measurement Compensation
bool Compensator::compensateMeasurement(Eigen::Vector3d &meas, std::string &measType) {

    // Get Errors
    Eigen::Vector3d b;
    Eigen::VectorXd m;
    Eigen::Vector3d sf;
    if (measType == "accel") {
        b = ba_;
        m = ma_;
        sf = sfa_;
    } else if (measType == "gyro") {
        b = bg_;
        m = mg_;
        sf = sfg_;
    } else {
        std::cout << "[Compensator::compensateMeasurement] Invalid measurement type" << std::endl;
        return false;
    }

    // Define Scale Factor Error Matrix
    Eigen::Matrix3d SF;
    SF << sf[0],  0.0,    0.0,
          0.0,    sf[1],  0.0,
          0.0,    0.0,    sf[2];

    // Define Misalignment Matrix
    Eigen::Matrix3d MIS;
    MIS << 1.0,   m[0],  m[1],
           m[2],  1.0,   m[3],
           m[4],  m[5],  1.0;

    // Perform Measurment Compensation
    meas = SF * MIS * (meas - b);

    // Return Statement for Successful Measurement Compensation
    return true;

}