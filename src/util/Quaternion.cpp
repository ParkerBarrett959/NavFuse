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

// Verify if Quaternion is Normalized
bool Quaternion::isNormalized(const quaternion_t& q) {

    // Initialize Normalized Flag
    bool normalized = false;

    // Verify if Quaternion is Normalized
    if (magnitude(q) == 1.0) {
        normalized = true;
    }
    
    // Return Statement
    return normalized;
}

// Normalize Quaternion
quaternion_t Quaternion::normalize(const quaternion_t& q) {

    // Compute Magnitude
    double mag = magnitude(q);

    // Normalize Quaternion
    quaternion_t qNormalized;
    qNormalized.q0 = q.q0 / mag;
    qNormalized.q1 = q.q1 / mag;
    qNormalized.q2 = q.q2 / mag;
    qNormalized.q3 = q.q3 / mag;

    // Return Statement
    return qNormalized;

} 

// Compute DCM from Quaternion
directionCosinesMatrix_t Quaternion::toDcm(const quaternion_t& q) {

    // Extract Elements of Quaternion
    double q0 = q.q0;
    double q1 = q.q1;
    double q2 = q.q2;
    double q3 = q.q3;
    
    // Define DCM Elements
    directionCosinesMatrix_t dcm;
    dcm.dcm[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    dcm.dcm[0][1] = 2*q1*q2 + 2*q0*q3;
    dcm.dcm[0][2] = 2*q1*q3 - 2*q0*q2;
    dcm.dcm[1][0] = 2*q1*q2 - 2*q0*q3;
    dcm.dcm[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    dcm.dcm[1][2] = 2*q2*q3 + 2*q0*q1;
    dcm.dcm[2][0] = 2*q1*q3 + 2*q0*q2;
    dcm.dcm[2][1] = 2*q2*q3 - 2*q0*q1;
    dcm.dcm[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // Return Statement
    return dcm;

}

// Compute Euler Angles from Quaternion
eulerAngles_t Quaternion::toEuler(const quaternion_t& q) {

    // Extract Elements of Quaternion
    double q0 = q.q0;
    double q1 = q.q1;
    double q2 = q.q2;
    double q3 = q.q3;
    
    // Compute Pitch
    eulerAngles_t rpy;
    rpy.pitch = std::asin(2*(q0*q2 - q1*q3));

    // Check for Gimbal Lock Case - Within 1 deg
    if (std::abs(rpy.pitch - M_PI / 2.0) < (M_PI / 180.0)) {
        
        // Set Euler Angles
        rpy.pitch = M_PI / 2.0;
        rpy.roll = 0.0;
        rpy.yaw = -2.0 * std::atan2(q1, q0);

        // Return Result
        return rpy;

    } else if (std::abs(rpy.pitch + M_PI / 2.0) < (M_PI / 180.0)) {

        // Set Euler Angles
        rpy.pitch = -M_PI / 2.0;
        rpy.roll = 0.0;
        rpy.yaw = 2.0 * std::atan2(q1, q0);

        // Return Result
        return rpy;

    }

    // Compute Roll and Yaw 
    rpy.roll = std::atan2(2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);
    rpy.yaw = std::atan2(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3);

    // Return Result
    return rpy;

}

// Compute Quaternion Conjugate
quaternion_t Quaternion::conjugate(const quaternion_t& q) {

    // Compute Conjugate
    quaternion_t qP;
    qP.q0 = q.q0; 
    qP.q1 = -q.q1; 
    qP.q2 = -q.q2; 
    qP.q3 = -q.q3;

    // Return Statement
    return qP;

}

// Compute Quaternion Inverse
quaternion_t Quaternion::inverse(const quaternion_t& q) {

    // Cpmpute Conjugate
    quaternion_t qP = conjugate(q);

    // Compute Squared Magnitude
    double mag = magnitude(q);
    double magSq = mag*mag;

    // Compute Quaternion Inverse
    quaternion_t qInv;
    qInv.q0 = qP.q0/magSq;
    qInv.q1 = qP.q1/magSq;
    qInv.q2 = qP.q2/magSq;
    qInv.q3 = qP.q3/magSq;

    // Return Statement
    return qInv;

}

// Quaternion Multiplication
quaternion_t Quaternion::multiply(const quaternion_t& qA,
                                  const quaternion_t& qB) {

    // Compute Quaternion Multiplication
    quaternion_t qC;
    qC.q0 = qA.q0*qB.q0 - qA.q1*qB.q1 - qA.q2*qB.q2 - qA.q3*qB.q3;
    qC.q1 = qA.q0*qB.q1 + qA.q1*qB.q0 - qA.q2*qB.q3 + qA.q3*qB.q2;
    qC.q2 = qA.q0*qB.q2 + qA.q1*qB.q3 + qA.q2*qB.q0 - qA.q3*qB.q1;
    qC.q3 = qA.q0*qB.q3 - qA.q1*qB.q2 + qA.q2*qB.q1 - qA.q3*qB.q0;
    
    //Return Statement
    return qC;

}