//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Navigation Utility Functions                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of helpful navigation utility functions.   //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "NavUtils.hpp"

// Compute DCM from Quaternion
bool NavUtils::computeDcmFromQuaternion(Eigen::VectorXd &qA2B,
                                        Eigen::MatrixXd &RA2B) {

    // Verify Correct Dimensions
    if (qA2B.size() != 4) {
        // Add Logging
        return false;
    } else if ((RA2B.rows() != 3) || (RA2B.cols() != 3)) {
        // Add Logging
        return false;
    }

    // Extract Quaternion Elements
    double q0 = qA2B[0];
    double q1 = qA2B[1];
    double q2 = qA2B[2];
    double q3 = qA2B[3];

    // Define Elements of RA2B
    double R11 = (2 * ((q0 * q0) + (q1 * q1))) - 1;
    double R12 = 2 * ((q1 * q2) - (q0 * q3));
    double R13 = 2 * ((q1 * q3) + (q0 * q2));
    double R21 = 2 * ((q1 * q2) + (q0 * q3));
    double R22 = (2 * ((q0 * q0) + (q2 * q2))) - 1;  
    double R23 = 2 * ((q2 * q3) - (q0 * q1));
    double R31 = 2 * ((q1 * q3) - (q0 * q2));
    double R32 = 2 * ((q2 * q3) + (q0 * q1));
    double R33 = (2 * ((q0 * q0) + (q3 * q3))) - 1;   

    // Set Rotation Matrix
    RA2B << R11, R12, R13,
            R21, R22, R23,
            R31, R32, R33;

    // Return Statement for Successful Initialization
    return true;

}


// Compute Quaternion from Rotation Vector
bool NavUtils::computeQuaternionFromRotationVec(Eigen::VectorXd &phi,
                                                Eigen::VectorXd &qA2B) {

    // Verify Correct Dimensions
    if (phi.size() != 3) {
        // Add Logging
        return false;
    } else if (qA2B.size() != 4) {
        // Add Logging
        return false;
    }

    // Compute Quaternion from Rotation Vector
    double phiMag = std::sqrt( (phi[0]*phi[0]) + (phi[1]*phi[1]) + (phi[2]*phi[2]) );
    double q0 = std::cos(phiMag / 2.0);
    double q1 =  (1.0 / phiMag) * std::sin(phiMag / 2.0) * phi[0];
    double q2 =  (1.0 / phiMag) * std::sin(phiMag / 2.0) * phi[1];
    double q3 =  (1.0 / phiMag) * std::sin(phiMag / 2.0) * phi[2];

    // Set Values
    qA2B[0] = q0;
    qA2B[1] = q1;
    qA2B[2] = q2;
    qA2B[3] = q3;

    // Return Statement
    return true;

}


// Build Quaternion Equivalent Matrix
bool NavUtils::buildQuaternionEquivalent(Eigen::VectorXd &qA2B,
                                         Eigen::MatrixXd &QA2B) {

    // Verify Correct Dimensions
    if (qA2B.size() != 4) {
        // Add Logging
        return false;
    }

    // Get Quaternion Elements
    double q0 = qA2B[0];
    double q1 = qA2B[1];
    double q2 = qA2B[2];
    double q3 = qA2B[3];

    // Build 4x4 Quaternion Equivalent Matrix
    QA2B << q0,  -q1,  -q2,  -q3,
            q1,   q0,  -q3,   q2,
            q2,   q3,   q0,  -q1,
            q3,  -q2,   q1,   q0;

    // Return Statement
    return true;

}

// Skew Symmetric Matrix Function
bool NavUtils::skewSymmetric(Eigen::Vector3d &vec,
                             Eigen::Matrix3d &vecX) {

    // Get Elements of Vector
    double v1 = vec[0];
    double v2 = vec[1];
    double v3 = vec[2];

    // Set Elements of Skew Symmetric
    vecX << 0.0, -v3, v2,
            v3,  0.0, -v1,
            -v2, v1,  0.0;

    // Return Statement
    return true;

}