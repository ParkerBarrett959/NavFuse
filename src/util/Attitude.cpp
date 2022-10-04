//////////////////////////////////////////////////////////////////////////////////////////////////////
//                              Navigation Attitude Representation Functions                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of helpful navigation attitude             //
//              representation functions.                                                           //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "Attitude.hpp"

// Compute DCM from Quaternion
bool Attitude::computeDcmFromQuaternion(Eigen::VectorXd &qA2B,
                                        Eigen::MatrixXd &RA2B) {

    // Verify Correct Dimensions
    if (qA2B.size() != 4) {
        std::cout << "[Attitude::computeDcmFromQuaternion] qA2B has incorrect dimensions: Expected " << 
                "4x1, Got " << qA2B.size() << "x1" << std::endl;
        return false;
    } else if ((RA2B.rows() != 3) || (RA2B.cols() != 3)) {
        std::cout << "[Attitude::computeDcmFromQuaternion] RA2B has incorrect dimensions: Expected " << 
                "3x3, Got " << RA2B.rows() << "x" << RA2B.cols() << std::endl;
        return false;
    }

    // Extract Quaternion Elements
    double q0 = qA2B[0];
    double q1 = qA2B[1];
    double q2 = qA2B[2];
    double q3 = qA2B[3];

    // Compute Denominator
    double den = (q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3); 

    // Define Elements of RA2B
    double R11 = ((q0 * q0) + (q1 * q1) - (q2 * q2) - (q3 * q3)) / den;
    double R12 = (2 * ((q1 * q2) + (q0 * q3))) / den;
    double R13 = (2 * ((q1 * q3) - (q0 * q2))) / den;
    double R21 = (2 * ((q1 * q2) - (q0 * q3))) / den;
    double R22 = ((q0 * q0) - (q1 * q1) + (q2 * q2) - (q3 * q3)) / den;  
    double R23 = (2 * ((q2 * q3) + (q0 * q1))) / den;
    double R31 = (2 * ((q1 * q3) + (q0 * q2))) / den;
    double R32 = (2 * ((q2 * q3) - (q0 * q1))) / den;
    double R33 = ((q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3)) / den;   

    // Set Rotation Matrix
    RA2B << R11, R12, R13,
            R21, R22, R23,
            R31, R32, R33;
    
    // Return Statement for Successful Initialization
    return true;

}


// Compute Quaternion from Rotation Vector
bool Attitude::computeQuaternionFromRotationVec(Eigen::VectorXd &phi,
                                                Eigen::VectorXd &qA2B) {

    // Verify Correct Dimensions
    if (phi.size() != 3) {
        std::cout << "[Attitude::computeQuaternionFromRotationVec] phi has incorrect dimensions: Expected " << 
                "3x1, Got " << phi.size() << "x1" << std::endl;
        return false;
    } else if (qA2B.size() != 4) {
        std::cout << "[Attitude::computeQuaternionFromRotationVec] qA2B has incorrect dimensions: Expected " << 
                "4x1, Got " << qA2B.size() << "x1" << std::endl;
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
bool Attitude::buildQuaternionEquivalent(Eigen::VectorXd &qA2B,
                                         Eigen::MatrixXd &QA2B) {

    // Verify Correct Dimensions
    if (qA2B.size() != 4) {
        std::cout << "[Attitude::buildQuaternionEquivalent] qA2B has incorrect dimensions: Expected " << 
                "4x1, Got " << qA2B.size() << "x1" << std::endl;
        return false;
    } else if ((QA2B.rows() != 4) || (QA2B.cols() != 4)) {
        std::cout << "[Attitude::buildQuaternionEquivalent] QA2B has incorrect dimensions: Expected " << 
                "4x4, Got " << QA2B.rows() << "x" << QA2B.cols() << std::endl;
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

// Compute DCM from RPH
bool Attitude::rph2Dcm(Eigen::Vector3d &rph,
                       Eigen::Matrix3d &RB2N) {

    // Extract Roll, Pitch and Heading
    double r = rph[0];
    double p = rph[1];
    double h = rph[2];
    
    // Define R3 (About Heading)
    Eigen::Matrix3d R3;
    R3 <<  std::cos(h),  std::sin(h),  0.0,
          -std::sin(h),  std::cos(h),  0.0,
           0.0,          0.0,          1.0;

    // Define R2 (About Pitch)
    Eigen::Matrix3d R2;
    R2 <<  std::cos(p),  0.0,  -std::sin(p),
           0.0,          1.0,   0.0,
           std::sin(p),  0.0,   std::cos(p);

    // Define R1 (About Roll)
    Eigen::Matrix3d R1;
    R1 <<  1.0,   0.0,            0.0,
           0.0,   std::cos(r),   std::sin(r),
           0.0,  -std::sin(r),   std::cos(r);

    // Compute Final Rotation
    RB2N = R1 * R2 * R3;
    
    // Return Statement
    return true;

}

// Compute RPH from DCM
bool Attitude::dcm2Rph(Eigen::Matrix3d &RB2N,
                       Eigen::Vector3d &rph) {
    
    // Extract Elements of DCM
    double R11 = RB2N(0, 0);
    double R12 = RB2N(0, 1);
    double R13 = RB2N(0, 2);
    double R23 = RB2N(1, 2);
    double R33 = RB2N(2, 2);
    
    // Check for Singularities
    if ((std::abs(R11) < 1e-9) || (std::abs(R33) < 1e-9)) {
        std::cout << "[Attitude::dcm2Rph] Singularity in Direction Cosines Matrix" << std::endl;
        return false;
    }
    
    // Check for Outside Range of Arcsin
    if ((R13 > 1) || (R13 < -1)) {
        std::cout << "[Attitude::dcm2Rph] Element R13 falls outside domain of Arcsin [-1, 1]" << std::endl;
        return false;
    }
    
    // Compute Roll
    rph(0) = std::atan(R23 / R33);

    // Compute Pitch
    rph(1) = -std::asin(R13);

    // Compute Heading
    rph(2)= std::atan(R12 / R11);
    if (rph(2) < 0) {
        rph(2) += M_PI;
    } 
    
    // Return Statement
    return true;

}