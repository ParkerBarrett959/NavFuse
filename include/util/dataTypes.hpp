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

// IMU Sensor Simulator Data
struct imuSensorSimData_t {
    bool useImu;
    double rate;                        // Data Rate [Hz]
    double accel_bias_repeatability;    // RMS Accel Bias Repeatability [micro-g]
    double accel_bias_instability;      // RMS Accel Bias Instability [micro-g]
    double accel_sf;                    // Accel SF 1-Sigma [ppm]
    double accel_mis;                   // AMIS 1-Sigma [micro-rad]
    double accel_vrw;                   // Velocity Random Walk [m/s/sqrt(hr)]
    double gyro_bias_repeatability;     // RMS Gyro Bias Repeatability [deg/hr]
    double gyro_bias_instability;       // RMS Gyro Bias Instability [deg/hr]
    double gyro_sf;                     // Gyro SF 1-Sigma [ppm]
    double gyro_mis;                    // GMIS 1-Sigma [micro-rad]
    double gyro_arw;                    // Angle Random Walk [deg/sqrt(hr)]
};

// Loosely Coupled GPS Sensor Simulator Data
struct looselyCoupledGpsSensorSimData_t {
    bool useLooselyCoupledGps;
    double rate;                    // Data Rate [Hz]
    double level_pos_sigma;         // Level (North/East) Position Error 1-Sigma [m]
    double vertical_pos_sigma;      // Vertical (Down) Position Error 1-Sigma [m]
};

// Sensor Simulator Data
struct sensorSimData_t {
    imuSensorSimData_t imu;                                 // Sensor Simulator IMU Model
    looselyCoupledGpsSensorSimData_t looselyCoupledGps;     // Sensor Simulator Loosely Coupled GPS Model
    std::string trajectoryFile;                             // Path to Trajectory CSV File
};