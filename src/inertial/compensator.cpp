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
    if (ma.size() != 6) {
        std::cout << "[Compensator::setAccelerometerErrors] misalignment has incorrect dimensions: Expected " << 
                "6x1, Got " << maEst.size() << "x1" << std::endl;
        return false;
    }

    // Set Class Variables
    ba = baEst;
    ma = maEst;
    sfa = sfaEst;

    // Return for Successful Set Accelerometer Errors
    return true;

}

// Set Gyroscope Errors
bool Compensator::setGyroscopeErrors(Eigen::Vector3d &bgEst,
                                     Eigen::VectorXd &mgEst,
                                     Eigen::Vector3d &sfgEst) {

    // Check for Correct Input Sizes
    if (mg.size() != 6) {
        std::cout << "[Compensator::setGyroscopeErrors] misalignment has incorrect dimensions: Expected " << 
                "6x1, Got " << mgEst.size() << "x1" << std::endl;
        return false;
    }

    // Set Class Variables
    bg = bgEst;
    mg = mgEst;
    sfg = sfgEst;

    // Return for Successful Set Gyroscope Errors
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
        b = ba;
        m = ma;
        sf = sfa;
    } else if (measType == "gyro") {
        b = bg;
        m = mg;
        sf = sfg;
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