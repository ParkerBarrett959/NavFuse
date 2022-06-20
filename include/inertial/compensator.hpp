//////////////////////////////////////////////////////////////////////////////////////////////////////
//                              Inertial Navigation Measurement Compensator                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial navigation measurement compensator class.                  //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <string>
#include <Eigen/Dense>

// Inertial Navigation Measurement Compensator Class
class Compensator {

    // Public Class Members/Functions
    public:

        /* @setAccelerometerErrors
            Inputs:
                baEst: 3x1 vector of accelerometer biases [m/s]
                maEst: 6x1 vector of accelerometer misalignments (top left to bottom right with ones on diagonal)
                sfaEst: 3x1 vector of accelerometer scale factor errors
            Outputs:
            Description:
                Function which takes in accelerometer errors and sets private class members.
        */
        bool setAccelerometerErrors(Eigen::Vector3d &baEst,
                                    Eigen::VectorXd &maEst,
                                    Eigen::Vector3d &sfaEst);

        /* @setGyroscopeErrors
            Inputs:
                bgEst: 3x1 vector of gyroscope biases [rad]
                mgEst: 6x1 vector of gyroscope misalignments (top left to bottom right with ones on diagonal)
                sfgEst: 3x1 vector of gyroscope scale factor errors
            Outputs:
            Description:
                Function which takes in gyroscope errors and sets private class members.
        */
        bool setGyroscopeErrors(Eigen::Vector3d &bgEst,
                                Eigen::VectorXd &mgEst,
                                Eigen::Vector3d &sfgEst);

        /* @compensateAccelerometer
            Inputs:
                ba: 3x1 vector of accelerometer biases [m/s]
                ma: 6x1 vector of accelerometer misalignments (top left to bottom right with ones on diagonal)
                sfa: 3x1 vector of accelerometer scale factor errors
                dV: 3x1 vector of raw accelerometer measurements [m/s]
            Outputs:
                dV: 3x1 vector of compensated accelerometer measurements [m/s]
            Description:
                Function which takes in a raw accelerometer delta velocity measurement along with estimates of
                the bias, scale factor and misalignments errors and performs measurement compensation.
        */
        bool compensateAccelerometer(Eigen::Vector3d &dV);

        /* @compensateGyro
            Inputs:
                bg: 3x1 vector of gyroscope biases [rad]
                mg: 6x1 vector of gyroscope misalignments (top left to bottom right with ones on diagonal)
                sfg: 3x1 vector of gyroscope scale factor errors
                dTh: 3x1 vector of raw gyroscope measurements [rad]
            Outputs:
                dTh: 3x1 vector of compensated gyroscope measurements [rad]
            Description:
                Function which takes in a raw accelerometer delta velocity measurement along with estimates of
                the bias, scale factor and misalignments errors and performs measurement compensation.
        */
        bool compensateGyroscope(Eigen::Vector3d &dTh);

    // Private Class Members/Function
    private:

        // Private Class Variables
        Eigen::Vector3d ba;
        Eigen::VectorXd ma;
        Eigen::Vector3d sfa;
        Eigen::Vector3d bg;
        Eigen::VectorXd mg;
        Eigen::Vector3d sfg;

        /* @compensateMeasurement
            Inputs:
                meas: 3x1 vector of raw measurements
                measType: string indicating whether measurement is accelerometer or gyroscope
            Outputs:
                dTh: 3x1 vector of compensated measurements
            Description:
                Function which takes in a raw measurement along with estimates of the bias, scale 
                factor and misalignments errors and performs measurement compensation.
        */
        bool compensateMeasurement(Eigen::Vector3d &meas, std::string &measType);

};

