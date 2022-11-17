//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Custom Navigation Data Types                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Custom navigation data type definitions.                                            //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

// Master Configuration Data
struct masterConfig_t {
    std::string configPath;             // Path to Configuration Files
    std::string imuCalibrationFile;     // Path to IMU Calibration Configuration File
};

// IMU Calibration Data
struct imuCalibrationData_t {
    Eigen::Vector3d ba;         // Accelerometer Bias [micro-g's]
    Eigen::Vector3d sfa;        // Accelerometer Scale Factor Error [ppm]
    Eigen::VectorXd ma;         // Accelerometer Misalignment Coefficients [6x1]
    Eigen::Vector3d bg;         // Gyroscope Bias [deg/hr]
    Eigen::Vector3d sfg;        // Gyroscope Scale Factor Error [ppm]
    Eigen::VectorXd mg;         // Gyroscope Misalignment Coefficients [6x1]
};
