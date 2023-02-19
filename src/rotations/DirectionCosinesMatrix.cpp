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
/*
// Compute DCM from Quaternion
DirectionCosinesMatrix Quaternion::toDcm() {
    
    // Define DCM Elements
    double R11 = q0_*q0_ + q1_*q1_ - q2_*q2_ - q3_*q3_;
    double R12 = 2*q1_*q2_ + 2*q0_*q3_;
    double R13 = 2*q1_*q3_ - 2*q0_*q2_;
    double R21 = 2*q1_*q2_ - 2*q0_*q3_;
    double R22 = q0_*q0_ - q1_*q1_ + q2_*q2_ - q3_*q3_;
    double R23 = 2*q2_*q3_ + 2*q0_*q1_;
    double R31 = 2*q1_*q3_ + 2*q0_*q2_;
    double R32 = 2*q2_*q3_ - 2*q0_*q1_;
    double R33 = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;

    // Define DCM
    std::array<std::array<double, 3>, 3> R = {{{R11, R12, R13},{R21, R22, R23},{R31, R32, R33}}};

    // Set DCM Output
    DirectionCosinesMatrix dcm(R);

    // Return Result
    return dcm;

}

// Compute Euler Angles from Quaternion
EulerAngles Quaternion::toEuler() {
    
    // Compute Pitch Argument
    double pitchArg = 2*(q0_*q2_ - q1_*q3_);

    // Check for Gimbal Lock Case - Within 1 deg
    double roll, pitch, yaw;
    if (pitchArg >= sin(89.0 * (M_PI / 180.0))) {
        
        // Set Euler Angles
        pitch = M_PI / 2.0;
        roll = 0.0;
        yaw = -2.0 * std::atan2(q1_, q0_);

    } else if (pitchArg <= sin(-89.0 * (M_PI / 180.0))) {

        // Set Euler Angles
        pitch = -M_PI / 2.0;
        roll = 0.0;
        yaw = 2.0 * std::atan2(q1_, q0_);

    } else {
        
        // Compute Roll and Yaw - Standard Case
        pitch = std::asin(pitchArg);
        roll = std::atan2(2*(q0_*q1_ + q2_*q3_), q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_);
        yaw = std::atan2(2*(q0_*q3_ + q1_*q2_), q0_*q0_ + q1_*q1_ - q2_*q2_ - q3_*q3_);

    }

    // Set Euler Angles Output
    EulerAngles euler(roll, pitch, yaw);

    // Return Result
    return euler;

}

// Compute Rotation Vector from Quaternion
RotationVector Quaternion::toRotationVector() {

    // Compute Rotation Angle
    double theta = 2.0 * std::acos(q0_);

    // Compute Rotation Vector
    double x, y, z = 0.0;
    if (theta != 0.0) {
        x = (theta * q1_) / std::sin(theta / 2.0);
        y = (theta * q2_) / std::sin(theta / 2.0);
        z = (theta * q3_) / std::sin(theta / 2.0);
    }

    // Set Rotation Vector Output
    std::array<double, 3> vec = {x, y, z};
    RotationVector rv(vec);

    // Return Result
    return rv;

}


// Compute Quaternion Inverse
Quaternion Quaternion::inverse() {

    // Compute Conjugate
    Quaternion qConj = conjugate();

    // Compute Squared Magnitude
    double mag = magnitude();
    double magSq = mag*mag;

    // Compute Quaternion Inverse
    double q0 = qConj.q0_/magSq;
    double q1 = qConj.q1_/magSq;
    double q2 = qConj.q2_/magSq;
    double q3 = qConj.q3_/magSq;

    // Create Quaternion Output
    Quaternion q(q0, q1, q2, q3);

    // Return Statement
    return q;

}*/

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