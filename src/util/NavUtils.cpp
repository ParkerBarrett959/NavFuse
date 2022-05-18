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

// Strapdown Initialization
bool NavUtils::computeRotationFromQuaternion(Eigen::VectorXd &qA2B,
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