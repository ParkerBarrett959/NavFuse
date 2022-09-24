//////////////////////////////////////////////////////////////////////////////////////////////////////
//                         Inertial Navigation Initialization and Alignment                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial navigation initialization and alignment class.             //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <cmath>
#include <Eigen/Dense>
#include "KalmanFilter.hpp"

// Inertial Navigation Initialization and Alignment Class
class Initialization {

    // Public Class Members/Functions
    public:

        /* @coarseAttitudeAlignment
            Inputs:
                lat: scalar double representing the geodetic latitude [rad]
                wE: scalar double representing Earth rotational rate magnitude [rad/s]
                g: scalar double representing gravitational acceleration magnitude [m/s^2]
                aB: 3x1 vector of accelerometer specific force outputs [m/s^2]
                wIB_B: 3x1 vector angular rate of body wrt inertial expressed in the body frame
            Outputs:
                RB2N: 3x3 dimensional direction cosines matrix from Body to NED Navigation frame
            Description:
                Function which takes in geodetic latitude, earth parameters (rotation rate and
                gravitational acceleration), body angular rate with respect to the inertial frame,
                and the accelerometer outputs in the body frame, and outputs the direction cosines
                matrix relating the body frame to the NED navigation frame. The coarse attitude 
                alignment assumes the INS is stationary on the surface of the earth and does not 
                account for accelerometer and gyroscope errors.
        */
        bool coarseAttitudeAlignment(double &lat,
                                     double &wE,
                                     double &g,
                                     Eigen::Vector3d &aB,
                                     Eigen::Vector3d &wIB_B,
                                     Eigen::Matrix3d &RB2N);

        /* @fineAlignmentPredict
            Inputs:
                lat: scalar double representing the geodetic latitude [rad]
                wE: scalar double representing Earth rotational rate magnitude [rad/s]
                g: scalar double representing gravitational acceleration magnitude [m/s^2]
                r: scalar double representing Earth geodetic radius plus the height above the ellipsoid [m]
                xk: 8x1 vector of alignment states (Attitude, gyro biases, and level velocity errors)
                Pk: 8x8 covarance matrix of alignment states
            Outputs:
                xkp1: 8x1 vector of predicted alignment states (Attitude, gyro biases, and level velocity errors)
                Pkp1: 8x8 covarance matrix of predicted alignment states
            Description:
                Function which takes in geodetic latitude, earth parameters (rotation rate and geodetic position),
                along with the fine alignment satte vector and covariance which consist of 3 attitude, 3 gyro bias 
                and 2 level velocity error states. The Kalman Filter Prediction is computed.
        */
        bool fineAlignmentPredict(double &lat,
                                  double &wE,
                                  double &g,
                                  double &r,
                                  Eigen::VectorXd &xk,
                                  Eigen::MatrixXd &Pk,
                                  Eigen::VectorXd &xkp1,
                                  Eigen::MatrixXd &Pkp1);

        /* @fineAlignmentUpdate
            Inputs:
                velValid: boolean indicating whether the velocity measurement is valid
                azValid: boolean indicating whether the azimuth measurement is valid
                velMeas: 2x1 vector of level (North/East) velocity observations
                azMeas: scalar double representing azimuth observation [rad]
                dtVel: scalar double representing the time between velocity measurements
                dtAz: scalar double representing the time between azimuth measurements
                azEst: scalar double representing the current system azimuth estimate [rad]
                xk: 8x1 vector of alignment states (Attitude, gyro biases, and level velocity errors)
                Pk: 8x8 covarance matrix of alignment states
            Outputs:
                xkp1: 8x1 vector of updated alignment states (Attitude, gyro biases, and level velocity errors)
                Pkp1: 8x8 covarance matrix of updated alignment states
            Description:
                Function which takes in geodetic latitude, earth parameters (rotation rate and geodetic position),
                along with the fine alignment satte vector and covariance which consist of 3 attitude, 3 gyro bias 
                and 2 level velocity error states. The Kalman Filter Prediction is computed.
        */
        bool fineAlignmentUpdate(bool &velValid,
                                 bool &azValid,
                                 Eigen::Vector2d velMeas,
                                 double &azMeas,
                                 double &dtVel,
                                 double &dtAz,
                                 double &azEst,
                                 double &sigVel,
                                 double &sigAz,
                                 Eigen::VectorXd &xk,
                                 Eigen::MatrixXd &Pk,
                                 Eigen::VectorXd &xkp1,
                                 Eigen::MatrixXd &Pkp1);

    // Private Class Members/Function
    private:

        // Kalman Filter Object
        ::KalmanFilter KF_;
        

};

