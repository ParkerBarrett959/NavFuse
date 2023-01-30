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

// Compute Quaternion from DCM
bool Attitude::computeQuaternionFromDcm(Eigen::MatrixXd &RA2B,
                                        Eigen::VectorXd &qA2B) {

    // Verify Correct Dimensions
    if (qA2B.size() != 4) {
        std::cout << "[Attitude::computeQuaternionFromDcm] qA2B has incorrect dimensions: Expected " << 
                "4x1, Got " << qA2B.size() << "x1" << std::endl;
        return false;
    } else if ((RA2B.rows() != 3) || (RA2B.cols() != 3)) {
        std::cout << "[Attitude::computeQuaternionFromDcm] RA2B has incorrect dimensions: Expected " << 
                "3x3, Got " << RA2B.rows() << "x" << RA2B.cols() << std::endl;
        return false;
    }

    // Extract Elements
    double R11 = RA2B(0, 0);
    double R12 = RA2B(0, 1);
    double R13 = RA2B(0, 2);
    double R21 = RA2B(1, 0);
    double R22 = RA2B(1, 1);
    double R23 = RA2B(1, 2);
    double R31 = RA2B(2, 0);
    double R32 = RA2B(2, 1);
    double R33 = RA2B(2, 2);

    // Compute Squares of Quaternion Elements
    double qSq0 = 0.25 * (1 + R11 + R22 + R33);
    double qSq1 = 0.25 * (1 + R11 - R22 - R33);
    double qSq2 = 0.25 * (1 - R11 + R22 - R33);
    double qSq3 = 0.25 * (1 - R11 - R22 + R33);

    // Compute Quaternion
    if ((qSq0 > qSq1) && (qSq0 > qSq2) && (qSq0 > qSq3)) {
        double q0 = std::sqrt(qSq0);
        qA2B << 4.0 * std::pow(q0, 2), (R23 - R32), (R31 - R13), (R12 - R21); 
        qA2B *= (1.0 / (4.0 * q0));
    } else if ((qSq1 > qSq0) && (qSq1 > qSq2) && (qSq1 > qSq3)) {
        double q1 = std::sqrt(qSq1);
        qA2B << (R23 - R32), 4.0 * std::pow(q1, 2), (R12 + R21), (R31 + R13);
        qA2B *= (1.0 / (4.0 * q1));
    } else if ((qSq2 > qSq0) && (qSq2 > qSq1) && (qSq2 > qSq3)) {
        double q2 = std::sqrt(qSq2);
        qA2B << (R31 - R13), (R12 + R21), 4.0 * std::pow(q2, 2), (R23 + R32);
        qA2B *= (1.0 / (4.0 * q2));
    } else {
        double q3 = std::sqrt(qSq3);
        qA2B << (R12 - R21), (R31 + R13), (R23 + R32), 4.0 * std::pow(q3, 2);
        qA2B *= (1.0 / (4.0 * q3));
    }
    
    // Return Statement for Successful Computation
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

// Compute Rotation Vector from Quaternion
bool Attitude::computeRotationVecFromQuaternion(Eigen::VectorXd &qA2B,
                                                Eigen::VectorXd &phi) {

    // Verify Correct Dimensions
    if (phi.size() != 3) {
        std::cout << "[Attitude::computeRotationVecFromQuaternion] phi has incorrect dimensions: Expected " <<
                "3x1, Got " << phi.size() << "x1" << std::endl;
        return false;
    } else if (qA2B.size() != 4) {
        std::cout << "[Attitude::computeRotationVecFromQuaternion] qA2B has incorrect dimensions: Expected " <<
                "4x1, Got " << qA2B.size() << "x1" << std::endl;
        return false;
    }

    // Get Vector Part of Quaternion and Enforce Sign Convention
    Eigen::Vector3d quatVec(3);
    quatVec << qA2B[1], qA2B[2], qA2B[3];
    if (qA2B[0] < 0) {
        quatVec *= -1.0;
    }

    // Get Norm of Vector Part of Quaternion
    double length = quatVec.norm();

    // Compute Angle-Axis
    double angle = 0.0;
    Eigen::Vector3d axis(3);
    axis << 1.0, 0.0, 0.0;
    if (length > 1.0e-6) {
        axis = quatVec / length;
        angle = -2.0 * std::asin(length);
    }

    // Compute Rotation Vector
    phi = angle * axis;
	
    // Successful Return
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

// Compute DCM from Euler Angles
bool Attitude::euler2Dcm(Eigen::Vector3d &euler,
                         Eigen::Matrix3d &RB2N) {

    // Extract Euler Angles
    double e1 = euler[0];
    double e2 = euler[1];
    double e3 = euler[2];
    
    // Define R3
    Eigen::Matrix3d R3;
    R3 <<  std::cos(e3),  std::sin(e3),  0.0,
          -std::sin(e3),  std::cos(e3),  0.0,
           0.0,          0.0,          1.0;

    // Define R2
    Eigen::Matrix3d R2;
    R2 <<  std::cos(e2),  0.0,  -std::sin(e2),
           0.0,          1.0,   0.0,
           std::sin(e2),  0.0,   std::cos(e2);

    // Define R1
    Eigen::Matrix3d R1;
    R1 <<  1.0,   0.0,            0.0,
           0.0,   std::cos(e1),   std::sin(e1),
           0.0,  -std::sin(e1),   std::cos(e1);

    // Compute Final Rotation
    RB2N = R1 * R2 * R3;
    
    // Return Statement
    return true;

}

// Compute Euler Angles from DCM
bool Attitude::dcm2Euler(Eigen::Matrix3d &RB2N,
                         Eigen::Vector3d &euler) {
    
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
    
    // Compute Euler Angle 1
    euler(0) = std::atan(R23 / R33);

    // Compute Euler Angle 2
    euler(1) = -std::asin(R13);

    // Compute Euler Angle 3
    euler(2)= std::atan(R12 / R11);
    if (euler(2) < 0) {
        euler(2) += M_PI;
    } 
    
    // Return Statement
    return true;

}
