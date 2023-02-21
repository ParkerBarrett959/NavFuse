//////////////////////////////////////////////////////////////////////////////////////////////////////
//                             Direction Cosines Matrix Class Unit Testing                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Direction Cosines Matrix Class Unit Tests.                                          //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include <Eigen/Dense>

// NavFuse Includes
#include "Quaternion.hpp"
#include "DirectionCosinesMatrix.hpp"
#include "EulerAngles.hpp"
#include "RotationVector.hpp"

// Test Constructor
TEST(Constructor, SetValuesDcm)
{

    // Create Direction Cosines Matrix Object
    std::array<std::array<double, 3>, 3> R = {{{1.0, 2.0, 3.0},{4.0, 5.0, 6.0},{7.0, 8.0, 9.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Check Values
    EXPECT_EQ(Dcm.dcm_[0][0], 1.0);
    EXPECT_EQ(Dcm.dcm_[0][1], 2.0);
    EXPECT_EQ(Dcm.dcm_[0][2], 3.0);
    EXPECT_EQ(Dcm.dcm_[1][0], 4.0);
    EXPECT_EQ(Dcm.dcm_[1][1], 5.0);
    EXPECT_EQ(Dcm.dcm_[1][2], 6.0);
    EXPECT_EQ(Dcm.dcm_[2][0], 7.0);
    EXPECT_EQ(Dcm.dcm_[2][1], 8.0);
    EXPECT_EQ(Dcm.dcm_[2][2], 9.0);

}

// Multiplication Overload Operator
TEST(OverloadOperator, MultiplyDcm)
{

    // Create DCM Objects
    std::array<std::array<double, 3>, 3> R = {{{1.0, 2.0, 3.0},{4.0, 5.0, 6.0},{7.0, 8.0, 9.0}}};
    DirectionCosinesMatrix Dcm1(R);
    DirectionCosinesMatrix Dcm2(R);

    // Perform Multiplication
    DirectionCosinesMatrix Dcm3 = Dcm1 * Dcm2;

    // Check Values
    EXPECT_EQ(Dcm3.dcm_[0][0], 30.0);
    EXPECT_EQ(Dcm3.dcm_[0][1], 36.0);
    EXPECT_EQ(Dcm3.dcm_[0][2], 42.0);
    EXPECT_EQ(Dcm3.dcm_[1][0], 66.0);
    EXPECT_EQ(Dcm3.dcm_[1][1], 81.0);
    EXPECT_EQ(Dcm3.dcm_[1][2], 96.0);
    EXPECT_EQ(Dcm3.dcm_[2][0], 102.0);
    EXPECT_EQ(Dcm3.dcm_[2][1], 126.0);
    EXPECT_EQ(Dcm3.dcm_[2][2], 150.0);

}

// Get Direction Cosines Matrix
TEST(GetDirectionCosinesMatrix, GetDirectionCosinesMatrix)
{

    // Create DCM Object
    std::array<std::array<double, 3>, 3> R = {{{1.0, 2.0, 3.0},{4.0, 5.0, 6.0},{7.0, 8.0, 9.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Get Quaternion as Eigen::Vector4d
    Eigen::Matrix3d ROut = Dcm.getDirectionCosinesMatrix();

    // Check Values
    EXPECT_EQ(ROut(0,0), 1.0);
    EXPECT_EQ(ROut(0,1), 2.0);
    EXPECT_EQ(ROut(0,2), 3.0);
    EXPECT_EQ(ROut(1,0), 4.0);
    EXPECT_EQ(ROut(1,1), 5.0);
    EXPECT_EQ(ROut(1,2), 6.0);
    EXPECT_EQ(ROut(2,0), 7.0);
    EXPECT_EQ(ROut(2,1), 8.0);
    EXPECT_EQ(ROut(2,2), 9.0);

}

// Rotate Vector by Quaternion: Passive
TEST(RotateVectorDcm, PassiveRotation)
{

    // Create DCM Objects
    std::array<std::array<double, 3>, 3> R = {{{1.0, 2.0, 3.0},{4.0, 5.0, 6.0},{7.0, 8.0, 9.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Create Eigen::Vector3d Object
    Eigen::Vector3d vecIn(1.0, 1.0, 1.0);

    // Rotate Vector
    Eigen::Vector3d vecOut = Dcm.passiveRotateVector(vecIn);

    // Check Values
    EXPECT_EQ(vecOut(0), 6.0);
    EXPECT_EQ(vecOut(1), 15.0);
    EXPECT_EQ(vecOut(2), 24.0);

}

// Rotate Vector by Quaternion: Active
TEST(RotateVectorDcm, ActiveRotation)
{

    // Create DCM Objects
    std::array<std::array<double, 3>, 3> R = {{{1.0, 2.0, 3.0},{4.0, 5.0, 6.0},{7.0, 8.0, 9.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Create Eigen::Vector3d Object
    Eigen::Vector3d vecIn(1.0, 1.0, 1.0);

    // Rotate Vector
    Eigen::Vector3d vecOut = Dcm.activeRotateVector(vecIn);

    // Check Values
    EXPECT_EQ(vecOut(0), 12.0);
    EXPECT_EQ(vecOut(1), 15.0);
    EXPECT_EQ(vecOut(2), 18.0);

}

// Test Transpose
TEST(TransposeMatrix, TransposeMatrix)
{

    // Create Direction Cosines Matrix Object
    std::array<std::array<double, 3>, 3> R = {{{1.0, 2.0, 3.0},{4.0, 5.0, 6.0},{7.0, 8.0, 9.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Transpose Matrix
    Dcm.transpose();

    // Check Values
    EXPECT_EQ(Dcm.dcm_[0][0], 1.0);
    EXPECT_EQ(Dcm.dcm_[0][1], 4.0);
    EXPECT_EQ(Dcm.dcm_[0][2], 7.0);
    EXPECT_EQ(Dcm.dcm_[1][0], 2.0);
    EXPECT_EQ(Dcm.dcm_[1][1], 5.0);
    EXPECT_EQ(Dcm.dcm_[1][2], 8.0);
    EXPECT_EQ(Dcm.dcm_[2][0], 3.0);
    EXPECT_EQ(Dcm.dcm_[2][1], 6.0);
    EXPECT_EQ(Dcm.dcm_[2][2], 9.0);

}

// Convert DCM to Quaternion: Case 1
TEST(DcmToQuaternion, Case1)
{

    // Create Direction Cosines Matrix Object
    std::array<std::array<double, 3>, 3> R = {{{1.0, 0.703, 0.245},{-0.028, 1.0, 0.934},{0.744, 0.616, 1.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Convert to Quaternion
    Quaternion q = Dcm.toQuaternion();

    // Check Values
    EXPECT_NEAR(q.q0_, 1.0, 1.0e-3);
    EXPECT_NEAR(q.q1_, 0.0795, 1.0e-3);
    EXPECT_NEAR(q.q2_, 0.1248, 1.0e-3);
    EXPECT_NEAR(q.q3_, 0.1828, 1.0e-3);

}

// Convert DCM to Quaternion: Case 2
TEST(DcmToQuaternion, Case2)
{

    // Create Direction Cosines Matrix Object
    std::array<std::array<double, 3>, 3> R = {{{0.0, 1.0, 0.0},{1.0, 0.0, 0.0},{0.0, 0.0, -1.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Convert to Quaternion
    Quaternion q = Dcm.toQuaternion();

    // Check Values
    EXPECT_NEAR(q.q0_, 0.0, 1.0e-3);
    EXPECT_NEAR(q.q1_, 0.7071, 1.0e-3);
    EXPECT_NEAR(q.q2_, 0.7071, 1.0e-3);
    EXPECT_NEAR(q.q3_, 0.0, 1.0e-3);

}

// Convert DCM to Quaternion: Case 3
TEST(DcmToQuaternion, Case3)
{

    // Create Direction Cosines Matrix Object
    std::array<std::array<double, 3>, 3> R = {{{-0.667, 0.703, 0.245},{-0.028, -0.354, 0.934},{0.744, 0.616, 0.256}}};
    DirectionCosinesMatrix Dcm(R);

    // Convert to Quaternion
    Quaternion q = Dcm.toQuaternion();

    // Check Values
    EXPECT_NEAR(q.q0_, 0.2422, 1.0e-3);
    EXPECT_NEAR(q.q1_, 0.3277, 1.0e-3);
    EXPECT_NEAR(q.q2_, 0.5136, 1.0e-3);
    EXPECT_NEAR(q.q3_, 0.7545, 1.0e-3);

}

// Convert DCM to Quaternion: Case 4
TEST(DcmToQuaternion, Case4)
{

    // Create Direction Cosines Matrix Object
    std::array<std::array<double, 3>, 3> R = {{{1.0, 0.703, 0.245},{-0.028, -1.0, 0.934},{0.744, 0.616, 1.0}}};
    DirectionCosinesMatrix Dcm(R);

    // Convert to Quaternion
    Quaternion q = Dcm.toQuaternion();

    // Check Values
    EXPECT_NEAR(q.q0_, 0.1828, 1.0e-3);
    EXPECT_NEAR(q.q1_, 0.2473, 1.0e-3);
    EXPECT_NEAR(q.q2_, 0.3875, 1.0e-3);
    EXPECT_NEAR(q.q3_, 1.0, 1.0e-3);

}

// Convert DCM to Euler Angles: Nominal Case
TEST(ConvertDcm, ToEuler_Nominal)
{

    // Create Quaternion Object
    Quaternion q(0.7071, 0.7071, 0.0, 0.0);

    // Normalize Quaternion
    q.normalize();

    // Convert Quaternion to DCM
    DirectionCosinesMatrix Dcm = q.toDcm();

    // Convert to Euler Angles
    EulerAngles eul = Dcm.toEuler();

    // Check Values
    EXPECT_NEAR(eul.pitch_, 0, 1.0e-3);
    EXPECT_NEAR(eul.roll_, 1.5708, 1.0e-3);
    EXPECT_NEAR(eul.yaw_, 0, 1.0e-3);

}

// Convert DCM to Euler Angles: +90 deg Gimbal Lock Case
TEST(ConvertDcm, ToEuler_Plus90Gimbal)
{

    // Create Quaternion Object
    Quaternion q(0.7071, 0.0, 0.7071, 0.0);

    // Normalize Quaternion
    q.normalize();

    // Convert Quaternion to DCM
    DirectionCosinesMatrix Dcm = q.toDcm();

    // Convert to Euler Angles
    EulerAngles eul = Dcm.toEuler();

    // Check Values
    EXPECT_NEAR(eul.pitch_, 90*(M_PI / 180.0), 1.0e-3);
    EXPECT_EQ(eul.roll_, 0);
    EXPECT_NEAR(eul.yaw_, 0.0, 1.0e-3);

}

// Convert DCM to Euler Angles: -90 deg Gimbal Lock Case
TEST(ConvertDcm, ToEuler_Minus90Gimbal)
{

    // Create Quaternion Object
    Quaternion q(0.0, 0.7071, 0.0, 0.7071);

    // Normalize Quaternion
    q.normalize();

    // Convert Quaternion to DCM
    DirectionCosinesMatrix Dcm = q.toDcm();

    // Convert to Euler Angles
    EulerAngles eul = Dcm.toEuler();

    // Check Values
    EXPECT_NEAR(eul.pitch_, -90*(M_PI / 180.0), 1.0e-3);
    EXPECT_EQ(eul.roll_, 0);
    EXPECT_NEAR(eul.yaw_, 2.0*1.5708, 1.0e-3);

}

// Convert DCM to Rotation Vector: 0 Magnitude Case
TEST(ConvertDcm, ToRotVec_ZeroMag)
{

    // Create Quaternion Object
    Quaternion q(1.0, 0.0, 0.0, 0.0);

    // Normalize Quaternion
    q.normalize();

    // Convert Quaternion to DCM
    DirectionCosinesMatrix Dcm = q.toDcm();
    
    // Convert to Rotation Vector
    RotationVector rotVec = Dcm.toRotationVector();

    // Check Values
    EXPECT_NEAR(rotVec.rv_[0], 0.0, 1.0e-12);
    EXPECT_NEAR(rotVec.rv_[1], 0.0, 1.0e-12);
    EXPECT_NEAR(rotVec.rv_[2], 0.0, 1.0e-12);

}

// Convert DCM to Rotation Vector: General Case
TEST(ConvertDcm, ToRotVec)
{

    // Create Quaternion Object
    Quaternion q(1.0, 2.0, 3.0, 4.0);

    // Normalize Quaternion
    q.normalize();

    // Convert Quaternion to DCM
    DirectionCosinesMatrix Dcm = q.toDcm();

    // Convert to Rotation Vector
    RotationVector rotVec = Dcm.toRotationVector();

    // Check Values
    EXPECT_NEAR(rotVec.rv_[0], 1.0303806, 1.0e-6);
    EXPECT_NEAR(rotVec.rv_[1], 1.5455709, 1.0e-6);
    EXPECT_NEAR(rotVec.rv_[2], 2.0607612, 1.0e-6);

}