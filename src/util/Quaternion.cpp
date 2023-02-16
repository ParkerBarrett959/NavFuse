//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Quaternion Class Implementation                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of quaternion operations.                  //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "Quaternion.hpp"

// Get Quaternion
Eigen::Vector4d Quaternion::getQuaternion() {

    // Initialize Vector and Set Values
    Eigen::Vector4d q(4);
    q << q0_, q1_, q2_, q3_;

    // Return Statement
    return q;

}

// Verify if Quaternion is Normalized
bool Quaternion::isNormalized() {

    // Initialize Normalized Flag
    bool normalized = false;

    // Verify if Quaternion is Normalized
    if (magnitude() == 1.0) {
        normalized = true;
    }
    
    // Return Statement
    return normalized;
}

// Normalize Quaternion
void Quaternion::normalize() {

    // Compute Magnitude
    double mag = magnitude();

    // Normalize Quaternion
    q0_ /= mag;
    q1_ /= mag;
    q2_ /= mag;
    q3_ /= mag;

    // Return Statement
    return;

} 

// Rotate Vector
Eigen::Vector3d Quaternion::rotateVector(const Eigen::Vector3d& vecIn) {

    // Convert 3D Vector to Temporary Quaternion
    Quaternion p(0.0, vecIn(0), vecIn(1), vecIn(2));

    // Get Quaternion Inverse
    Quaternion qP = inverse();

    // Perform Multiplication
    Quaternion pP = multiply(p.multiply(qP));

    // Set Output Vector
    Eigen::Vector3d vecOut(3);
    vecOut << pP.q1_, pP.q2_, pP.q3_; 

    // Return Statement
    return vecOut;

}

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
    
    // Compute Pitch
    double pitch = std::asin(2*(q0_*q2_ - q1_*q3_));

    // Check for Gimbal Lock Case - Within 1 deg
    double roll, yaw;
    if (std::abs(pitch - M_PI / 2.0) < (M_PI / 180.0)) {
        
        // Set Euler Angles
        pitch = M_PI / 2.0;
        roll = 0.0;
        yaw = -2.0 * std::atan2(q1_, q0_);

    } else if (std::abs(pitch + M_PI / 2.0) < (M_PI / 180.0)) {

        // Set Euler Angles
        pitch = -M_PI / 2.0;
        roll = 0.0;
        yaw = 2.0 * std::atan2(q1_, q0_);

    } else {

        // Compute Roll and Yaw - Standard Case
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

// Compute Quaternion Conjugate
Quaternion Quaternion::conjugate() {

    // Compute Conjugate
    double q0 = q0_;
    double q1 = -q1_; 
    double q2 = -q2_; 
    double q3 = -q3_;

    // Create Quaternion Output
    Quaternion q(q0, q1, q2, q3);

    // Return Statement
    return q;

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

}

// Quaternion Multiplication
Quaternion Quaternion::multiply(const Quaternion& qB) {

    // Compute Quaternion Multiplication
    double q0 = q0_*qB.q0_ - q1_*qB.q1_ - q2_*qB.q2_ - q3_*qB.q3_;
    double q1 = q0_*qB.q1_ + q1_*qB.q0_ - q2_*qB.q3_ + q3_*qB.q2_;
    double q2 = q0_*qB.q2_ + q1_*qB.q3_ + q2_*qB.q0_ - q3_*qB.q1_;
    double q3 = q0_*qB.q3_ - q1_*qB.q2_ + q2_*qB.q1_ - q3_*qB.q0_;

    // Create Quaternion Output
    Quaternion qC(q0, q1, q2, q3);
    
    //Return Statement
    return qC;

}