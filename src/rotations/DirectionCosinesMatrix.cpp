//////////////////////////////////////////////////////////////////////////////////////////////////////
//                             Direction Cosines Matrix Class Implementation                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of direction cosines matrix operations.    //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "DirectionCosinesMatrix.hpp"

// Get Direction Cosines Matrix
Eigen::Matrix3d DirectionCosinesMatrix::getDirectionCosinesMatrix() {

    // Initialize Matrix and Set Values
    Eigen::Matrix3d R;
    R << dcm_[0][0], dcm_[0][1], dcm_[0][2],
         dcm_[1][0], dcm_[1][1], dcm_[1][2],
         dcm_[2][0], dcm_[2][1], dcm_[2][2];

    // Return Statement
    return R;

}

// Rotate Vector
Eigen::Vector3d DirectionCosinesMatrix::passiveRotateVector(const Eigen::Vector3d& vecIn) {

    // Get Matrix
    Eigen::Matrix3d R = getDirectionCosinesMatrix();

    // Perform Passive Rotation
    Eigen::Vector3d vecOut = R * vecIn;

    // Return Statement
    return vecOut;

}

// Rotate Vector
Eigen::Vector3d DirectionCosinesMatrix::activeRotateVector(const Eigen::Vector3d& vecIn) {

    // Get Matrix
    Eigen::Matrix3d R = getDirectionCosinesMatrix();

    // Perform Passive Rotation
    Eigen::Vector3d vecOut = R.transpose() * vecIn;

    // Return Statement
    return vecOut;

}

// Transpose Matrix
void DirectionCosinesMatrix::transpose() {

    // Get Matrix as Eigen::Matrix3d
    Eigen::Matrix3d R = getDirectionCosinesMatrix();

    // Tranpose Result
    Eigen::Matrix3d Rp = R.transpose();

    // Set Class Elements
    std::array<std::array<double, 3>, 3> ROut = {{{Rp(0,0), Rp(0,1), Rp(0,2)},{Rp(1,0), Rp(1,1), Rp(1,2)},{Rp(2,0), Rp(2,1), Rp(2,2)}}};
    dcm_ = ROut;

}

// Compute Quaternion from DCM
Quaternion DirectionCosinesMatrix::toQuaternion() {
    
    // Compute Squared Magnitude of each Element
    double q0Sq = 0.25 * (1 + dcm_[0][0] + dcm_[1][1] + dcm_[2][2]);
    double q1Sq = 0.25 * (1 + dcm_[0][0] - dcm_[1][1] - dcm_[2][2]);
    double q2Sq = 0.25 * (1 - dcm_[0][0] + dcm_[1][1] - dcm_[2][2]);
    double q3Sq = 0.25 * (1 + dcm_[0][0] - dcm_[1][1] + dcm_[2][2]);

    // Initialize Quaternion Elements
    double q0, q1, q2, q3;

    // Check for Maximum Squared Value and Set Quaternion Values
    if ((q0Sq >= q1Sq) && (q0Sq >= q2Sq) && (q0Sq >= q3Sq)) {
        q0 = std::sqrt(q0Sq);
        q1 = (1.0/(4.0*q0)) * (dcm_[1][2] - dcm_[2][1]);
        q2 = (1.0/(4.0*q0)) * (dcm_[2][0] - dcm_[0][2]);
        q3 = (1.0/(4.0*q0)) * (dcm_[0][1] - dcm_[1][0]);
    } else if ((q1Sq >= q0Sq) && (q1Sq >= q2Sq) && (q1Sq >= q3Sq)) {
        q1 = std::sqrt(q1Sq);
        q0 = (1.0/(4.0*q1)) * (dcm_[1][2] - dcm_[2][1]);
        q2 = (1.0/(4.0*q1)) * (dcm_[0][1] + dcm_[1][0]);
        q3 = (1.0/(4.0*q1)) * (dcm_[2][0] + dcm_[0][2]);
    } else if ((q2Sq >= q0Sq) && (q2Sq >= q1Sq) && (q2Sq >= q3Sq)) {
        q2 = std::sqrt(q2Sq);
        q0 = (1.0/(4.0*q2)) * (dcm_[2][0] - dcm_[0][2]);
        q1 = (1.0/(4.0*q2)) * (dcm_[0][1] + dcm_[1][0]);
        q3 = (1.0/(4.0*q2)) * (dcm_[1][2] + dcm_[2][1]);
    } else {
        q3 = std::sqrt(q3Sq);
        q0 = (1.0/(4.0*q3)) * (dcm_[0][1] - dcm_[1][0]);
        q1 = (1.0/(4.0*q3)) * (dcm_[2][0] + dcm_[0][2]);
        q2 = (1.0/(4.0*q3)) * (dcm_[1][2] + dcm_[2][1]);
    }
    
    // Set Quaternion
    Quaternion q(q0, q1, q2, q3);

    // Return Result
    return q;

}

// Compute Euler Angles from DCM
EulerAngles DirectionCosinesMatrix::toEuler() {
    
    // Convert Direction Cosines Matrix to Quaternion
    Quaternion q = toQuaternion();

    // Normalize Quaternion
    q.normalize();

    // Convert Quaternion to Euler Angles
    EulerAngles euler = q.toEuler();

    // Return Result
    return euler;

}

// Compute Rotation Vector from Direction Cosines Matrix
RotationVector DirectionCosinesMatrix::toRotationVector() {

    // Convert Direction Cosines Matrix to Quaternion
    Quaternion q = toQuaternion();
    std::cout << std::endl;
    
    // Normalize Quaternion
    q.normalize();
    
    // Convert Quaternion to Rotation Vector
    RotationVector rv = q.toRotationVector();

    // Return Result
    return rv;

}

// Direction Cosines Matrix Multiplication
DirectionCosinesMatrix DirectionCosinesMatrix::multiply(const DirectionCosinesMatrix& RB) {

    // Compute Direction Cosines Matrix Multiplication
    double R11 = dcm_[0][0]*RB.dcm_[0][0] + dcm_[0][1]*RB.dcm_[1][0] + dcm_[0][2]*RB.dcm_[2][0]; 
    double R12 = dcm_[0][0]*RB.dcm_[0][1] + dcm_[0][1]*RB.dcm_[1][1] + dcm_[0][2]*RB.dcm_[2][1];
    double R13 = dcm_[0][0]*RB.dcm_[0][2] + dcm_[0][1]*RB.dcm_[1][2] + dcm_[0][2]*RB.dcm_[2][2];
    double R21 = dcm_[1][0]*RB.dcm_[0][0] + dcm_[1][1]*RB.dcm_[1][0] + dcm_[1][2]*RB.dcm_[2][0]; 
    double R22 = dcm_[1][0]*RB.dcm_[0][1] + dcm_[1][1]*RB.dcm_[1][1] + dcm_[1][2]*RB.dcm_[2][1];
    double R23 = dcm_[1][0]*RB.dcm_[0][2] + dcm_[1][1]*RB.dcm_[1][2] + dcm_[1][2]*RB.dcm_[2][2];
    double R31 = dcm_[2][0]*RB.dcm_[0][0] + dcm_[2][1]*RB.dcm_[1][0] + dcm_[2][2]*RB.dcm_[2][0]; 
    double R32 = dcm_[2][0]*RB.dcm_[0][1] + dcm_[2][1]*RB.dcm_[1][1] + dcm_[2][2]*RB.dcm_[2][1];
    double R33 = dcm_[2][0]*RB.dcm_[0][2] + dcm_[2][1]*RB.dcm_[1][2] + dcm_[2][2]*RB.dcm_[2][2];

    // Assemble Direction Cosines Matrix
    std::array<std::array<double, 3>, 3> R = {{{R11, R12, R13},{R21, R22, R23},{R31, R32, R33}}};

    // Create Direction Cosines Matrix Output
    DirectionCosinesMatrix RC(R);
    
    //Return Statement
    return RC;

}