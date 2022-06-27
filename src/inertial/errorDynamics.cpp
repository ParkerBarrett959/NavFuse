//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Inertial Navigation Error Dynamics                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines the inertial navigation error dynamics.          //   
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "errorDynamics.hpp"

// Inertial Frame Error Dynamics
bool InertialErrorDynamics::inertialErrorDynamics(Eigen::Vector3d &aI,
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
    F = Eigen::Matrix3d::Zero();
    G = Eigen::Matrix3d::Zero();
 
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

// ECEF Frame Error Dynamics
bool InertialErrorDynamics::ecefErrorDynamics(Eigen::Vector3d &aE,
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
    F = Eigen::Matrix3d::Zero();
    G = Eigen::Matrix3d::Zero();

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

// NED Frame Error Dynamics
bool InertialErrorDynamics::nedErrorDynamics(Eigen::Vector3d &aN,
                                             Eigen::Matrix3d &GN,
                                             Eigen::Matrix3d &RS2N,
                                             Eigen::Vector3d &lla,
                                             Eigen::Vector3d &llaDot,
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
    double a1 = aN[0];
    double a2 = aN[1];
    double a3 = aN[2];
    double G33 = GN[2][2];

    // Get Earth Constants and Parameters
    double wE = Gravity.gravityParams.wE;
    double fE = Gravity.gravityParams.f;
    double a = Gravity.gravityParams.a;

    // Compute Useful Quantities
    double eSq = (2.0 * fE) - std::pow(fE, 2); 
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double s2lat = std::sin(2.0 * lat);
    double c2lat = std::cos(2.0 * lat);
    double N = a / (std::sqrt(1 - (eSq * std::pow(slat, 2))));
    double M = (a * (1 - eSq)) / std::pow(1 - (eSq * std::pow(slat, 2)), 1.5);
    double ep = ;
    double RPhi = M * std::pow(1 + (0.007 * std::pow(clat, 2)), 0.5);
    double r = RPhi + h;
    double l1Dot = lonDot + wE;
    double l2Dot = lonDot + (2 * wE);  

    // Set Free Inertial Dynamics Matrix
    F << 0.0, -l1Dot*slat, -latDot, 0.0, clat, 0.0, -l1Dot*slat, 0.0, 0.0,
         l1Dot*slat, 0.0, l1Dot*clat, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         -latDot, -l1Dot*clat, 0.0, 0.0, -slat, 0.0, -l1Dot*clat, 0.0, 0.0,
         0.0, -a3/r, a2/r, -2.0*hDot/r, -l1Dot*s2lat, -2.0*latDot/r, -lonDot*l2Dot*c2lat, ;

    // ToDo Above - Last element of line 4 is lat double dot

    // Return for Successful Inertial Error Dynamics
    return true;

}