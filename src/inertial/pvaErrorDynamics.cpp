//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                Inertial Navigation PVA Error Dynamics                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines the inertial navigation position, velocity       //
//              attitude error dynamics.                                                            //   
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "pvaErrorDynamics.hpp"

// Inertial Frame PVA Error Dynamics
bool PvaErrorDynamics::inertialErrorDynamics(Eigen::Vector3d &aI,
                                             Eigen::Matrix3d &GI,
                                             Eigen::Matrix3d &RS2I,
                                             Eigen::MatrixXd &F,
                                             Eigen::MatrixXd &G) {

    // Check for Correct Input Sizes
    if ((F.rows() != 9) || (F.cols() != 9)) {
        std::cout << "[InertialErrorDynamics::inertialErrorDynamics] F has incorrect dimensions: Expected " << 
                "9x9, Got " << F.rows() << "x" << F.cols() << std::endl;
        return false;
    } else if ((G.rows() != 9) || (G.cols() != 9)) {
        std::cout << "[InertialErrorDynamics::inertialErrorDynamics] G has incorrect dimensions: Expected " << 
                "9x9, Got " << G.rows() << "x" << G.cols() << std::endl;
        return false;
    }

    // Set F and G to Zero Matrices
    F = Eigen::MatrixXd::Zero(9,9);
    G = Eigen::MatrixXd::Zero(9,9);
 
    // Set Jacobian of Velocity wrt Attitude
    NavUtils NavUtil;
    Eigen::Matrix3d aX;
    if (!NavUtil.skewSymmetric(aI, aX)) {
        std::cout << "[InertialErrorDynamics::inertialErrorDynamics] Unable to set skew symmetric" << std::endl;
        return false;
    }
    F.block<3,3>(3,0) = aX;

    // Set Jacobian of Position wrt Velocity
    F.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

    // Set Jacobian of Velocity wrt Position
    F.block<3,3>(3,6) = GI;

    // Set Jacobian of Attitude wrt Angular Rate Errors
    G.block<3,3>(0,0) = -RS2I;

    // Set Jacobian of Velocity wrt Acceleration Errors
    G.block<3,3>(3,3) = RS2I;

    // Set Jacobian of Velocity wrt Gravity Errors
    G.block<3,3>(3,6) = Eigen::Matrix3d::Identity();

    // Return for Successful Inertial Error Dynamics
    return true;

}

// ECEF Frame PVA Error Dynamics
bool PvaErrorDynamics::ecefErrorDynamics(Eigen::Vector3d &aE,
                                         Eigen::Matrix3d &GE,
                                         Eigen::Matrix3d &RS2E,
                                         Eigen::Matrix3d &OEI_E,
                                         Eigen::MatrixXd &F,
                                         Eigen::MatrixXd &G) {

    // Check for Correct Input Sizes
    if ((F.rows() != 9) || (F.cols() != 9)) {
        std::cout << "[InertialErrorDynamics::ecefErrorDynamics] F has incorrect dimensions: Expected " << 
                "9x9, Got " << F.rows() << "x" << F.cols() << std::endl;
        return false;
    } else if ((G.rows() != 9) || (G.cols() != 9)) {
        std::cout << "[InertialErrorDynamics::ecefErrorDynamics] G has incorrect dimensions: Expected " << 
                "9x9, Got " << G.rows() << "x" << G.cols() << std::endl;
        return false;
    }

    // Set F and G to Zero Matrices
    F = Eigen::MatrixXd::Zero(9,9);
    G = Eigen::MatrixXd::Zero(9,9);

    // Set Jacobian of Attitude wrt Attitude
    F.block<3,3>(0,0) = - OEI_E;
 
    // Set Jacobian of Velocity wrt Attitude
    NavUtils NavUtil;
    Eigen::Matrix3d aX;
    if (!NavUtil.skewSymmetric(aE, aX)) {
        std::cout << "[InertialErrorDynamics::ecefErrorDynamics] Unable to set skew symmetric" << std::endl;
        return false;
    }
    F.block<3,3>(3,0) = aX;

    // Set Jacobean of Velocity wrt Velocity
    F.block<3,3>(3,3) = -2.0 * OEI_E;

    // Set Jacobian of Position wrt Velocity
    F.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

    // Set Jacobian of Velocity wrt Position
    F.block<3,3>(3,6) = -((OEI_E * OEI_E) - GE);

    // Set Jacobian of Attitude wrt Angular Rate Errors
    G.block<3,3>(0,0) = -RS2E;

    // Set Jacobian of Velocity wrt Acceleration Errors
    G.block<3,3>(3,3) = RS2E;

    // Set Jacobian of Velocity wrt Gravity Errors
    G.block<3,3>(3,6) = Eigen::Matrix3d::Identity();

    // Return for Successful Inertial Error Dynamics
    return true;

}

// NED Frame PVA Error Dynamics
bool PvaErrorDynamics::nedErrorDynamics(Eigen::Vector3d &aN,
                                        Eigen::Matrix3d &GN,
                                        Eigen::Matrix3d &RS2N,
                                        Eigen::Vector3d &lla,
                                        Eigen::Vector3d &llaDot,
                                        Eigen::Vector3d &llaDDot,
                                        Eigen::MatrixXd &F,
                                        Eigen::MatrixXd &G) {

    // Check for Correct Input Sizes
    if ((F.rows() != 9) || (F.cols() != 9)) {
        std::cout << "[InertialErrorDynamics::nedErrorDynamics] F has incorrect dimensions: Expected " << 
                "9x9, Got " << F.rows() << "x" << F.cols() << std::endl;
        return false;
    } else if ((G.rows() != 9) || (G.cols() != 9)) {
        std::cout << "[InertialErrorDynamics::nedErrorDynamics] G has incorrect dimensions: Expected " << 
                "9x9, Got " << G.rows() << "x" << G.cols() << std::endl;
        return false;
    }

    // Unpack Position/Rates/Accelerations
    double lat = lla[0];
    double lon = lla[1];
    double h = lla[2];
    double latDot = llaDot[0];
    double lonDot = llaDot[1];
    double hDot = llaDot[2];
    double latDDot = llaDDot[0];
    double lonDDot = llaDDot[1];
    double a1 = aN[0];
    double a2 = aN[1];
    double a3 = aN[2];
    double G33 = GN.coeff(2,2);

    // Get Earth Constants and Parameters
    double wE = Gravity.gravityParams.wE;
    double fE = Gravity.gravityParams.f;
    double a = Gravity.gravityParams.a;

    // Compute Useful Quantities
    double eSq = (2.0 * fE) - std::pow(fE, 2); 
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double tlat = std::tan(lat);
    double s2lat = std::sin(2.0 * lat);
    double c2lat = std::cos(2.0 * lat);
    double N = a / (std::sqrt(1 - (eSq * std::pow(slat, 2))));
    double M = (a * (1 - eSq)) / std::pow(1 - (eSq * std::pow(slat, 2)), 1.5);
    double RPhi = std::pow(N * M, 0.5);
    double r = RPhi + h;
    double l1Dot = lonDot + wE;
    double l2Dot = lonDot + (2 * wE);  

    // Define Elements of F
    double F12 = -l1Dot*slat;
    double F13 = latDot;
    double F15 = clat;
    double F17 = -l1Dot*slat;
    double F21 = l1Dot*slat;
    double F23 = l1Dot*clat;
    double F31 = -latDot;
    double F32 = -l1Dot*clat;
    double F35 = -slat;
    double F37 = -l1Dot*clat;
    double F42 = -a3/r;
    double F43 = a2/r;
    double F44 = -2.0*hDot/r;
    double F45 = -l1Dot*s2lat;
    double F46 = -2.0*latDot/r;
    double F47 = -lonDot*l2Dot*c2lat;
    double F49 = (latDDot+(0.5*lonDot*l2Dot*s2lat))/(-r);
    double F51 = a3/(r*clat);
    double F53 = -a1/(r*clat);
    double F54 = 2*l1Dot*tlat;
    double F55 = 2*((latDot*tlat)-(hDot/r));
    double F56 = -2*l1Dot/r;
    double F57 = (2*l1Dot*(latDot+(hDot*tlat/r)))+(lonDDot*tlat);
    double F59 = ((2*latDot*tlat)-lonDDot)/r;
    double F61 = a2;
    double F62 = -a1;
    double F64 = 2*r*latDot;
    double F65 = 2*r*l1Dot*std::pow(clat, 2);
    double F67 = -r*lonDot*l2Dot*s2lat;
    double F69 = (latDot*latDot) + (lonDot*l2Dot*std::pow(clat, 2)) + G33;

    // Set Free Inertial Dynamics Matrix
    F << 0.0,  F12,  F13,  0.0,  F15,  0.0,  F17,  0.0,  0.0,
         F21,  0.0,  F23, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,
         F31,  F32,  0.0,  0.0,  F35,  0.0,  F37,  0.0,  0.0,
         0.0,  F42,  F43,  F44,  F45,  F46,  F47,  0.0,  F49,
         F51,  0.0,  F53,  F54,  F55,  F56,  F57,  0.0,  F59,
         F61,  F62,  0.0,  F64,  F65,  0.0,  F67,  0.0,  F69,
         0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,
         0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,
         0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0;

    // Define Dinv Matrix
    Eigen::Matrix3d D;
    D << M+h,   0.0,        0.0,
         0.0,  (N+h)*clat,  0.0,
         0.0,   0.0,       -1.0; 
    Eigen::Matrix3d Dinv = D.inverse();

    // Set Jacobian of Attitude wrt Angular Rate Errors
    G = Eigen::MatrixXd::Zero(9,9);
    G.block<3,3>(0,0) = -RS2N;

    // Set Jacobian of Velocity wrt Acceleration Errors
    G.block<3,3>(3,3) = Dinv * RS2N;

    // Set Jacobian of Velocity wrt Gravity Errors
    G.block<3,3>(3,6) = Dinv;

    // Return for Successful Inertial Error Dynamics
    return true;

}