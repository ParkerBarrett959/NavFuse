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

// Set Accelerometer Errors
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