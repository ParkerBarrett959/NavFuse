//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Quaternion Class Unit Testing                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Quaternion Class Unit Tests.                                                        //
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
TEST(Constructor, SetValues)
{

    // Create Quaternion Object
    Quaternion q(1.0, 2.0, 3.0, 4.0);

    // Check Values
    EXPECT_EQ(q.q0_, 1.0);
    EXPECT_EQ(q.q1_, 2.0);
    EXPECT_EQ(q.q2_, 3.0);
    EXPECT_EQ(q.q3_, 4.0);

}

// Multiplication Overload Operator
TEST(OverloadOperator, MultiplyQuaternions)
{

    // Create Quaternion Objects
    Quaternion q1(1.0, 0.5, 0.5, 0.75);
    Quaternion q2(1.0, 0.0, 1.0, 0.0);

    // Perform Multiplication
    Quaternion q3 = q2 * q1;

    // Check Values
    EXPECT_EQ(q3.q0_, 0.50);
    EXPECT_EQ(q3.q1_, 1.25);
    EXPECT_EQ(q3.q2_, 1.50);
    EXPECT_EQ(q3.q3_, 0.25);

}

// Get Quaternion
TEST(GetQuaternion, GetQuaternion)
{

    // Create Quaternion Object
    Quaternion q(1.0, 2.0, 3.0, 4.0);

    // Get Quaternion as Eigen::Vector4d
    Eigen::Vector4d qOut = q.getQuaternion();

    // Check Values
    EXPECT_EQ(qOut(0), 1.0);
    EXPECT_EQ(qOut(1), 2.0);
    EXPECT_EQ(qOut(2), 3.0);
    EXPECT_EQ(qOut(3), 4.0);

}

// Check if Quaternion is Normalized: False Case
TEST(IsNormalized, FalseCase)
{

    // Create Quaternion Object
    Quaternion q(1.0, 2.0, 3.0, 4.0);

    // Check Values
    EXPECT_FALSE(q.isNormalized());

}

// Normalize Quaternion
TEST(IsNormalized, TrueCase)
{

    // Create Quaternion Object
    Quaternion q(1.0, 0.0, 0.0, 0.0);

    // Check Values
    EXPECT_TRUE(q.isNormalized());

}

// Check if Quaternion is Normalized: True Case
TEST(Normalize, NormalizeQuaternion)
{

    // Create Quaternion Object
    Quaternion q(1.0, 2.0, 3.0, 4.0);

    // Noramlize Quaternion
    q.normalize();

    // Check Values
    EXPECT_EQ(q.q0_, 1.0 / std::sqrt(30.0));
    EXPECT_EQ(q.q1_, 2.0 / std::sqrt(30.0));
    EXPECT_EQ(q.q2_, 3.0 / std::sqrt(30.0));
    EXPECT_EQ(q.q3_, 4.0 / std::sqrt(30.0));

}

// Rotate Vector by Quaternion: Passive
TEST(RotateVector, PassiveRotation)
{

    // Create Quaternion Object
    Quaternion q(1.0, 0.0, 1.0, 0.0);

    // Create Eigen::Vector3d Object
    Eigen::Vector3d vecIn(1.0, 1.0, 1.0);

    // Noramlize Quaternion
    q.normalize();

    // Rotate Vector
    Eigen::Vector3d vecOut = q.passiveRotateVector(vecIn);

    // Check Values
    EXPECT_EQ(vecOut(0), -1.0);
    EXPECT_EQ(vecOut(1), 1.0);
    EXPECT_EQ(vecOut(2), 1.0);

}

// Rotate Vector by Quaternion: Active
TEST(RotateVector, ActiveRotation)
{

    // Create Quaternion Object
    Quaternion q(1.0, 0.0, 1.0, 0.0);

    // Create Eigen::Vector3d Object
    Eigen::Vector3d vecIn(1.0, 1.0, 1.0);

    // Noramlize Quaternion
    q.normalize();

    // Rotate Vector
    Eigen::Vector3d vecOut = q.activeRotateVector(vecIn);

    // Check Values
    EXPECT_EQ(vecOut(0), 1.0);
    EXPECT_EQ(vecOut(1), 1.0);
    EXPECT_EQ(vecOut(2), -1.0);

}

// Convert Quaternion to DCM
TEST(ConvertQuaternion, ToDcm)
{

    // Create Quaternion Object
    Quaternion q(1.0, 0.5, 0.3, 0.1);

    // Noramlize Quaternion
    q.normalize();

    // Convert to DCM
    DirectionCosinesMatrix R = q.toDcm();

    // Check Values
    EXPECT_NEAR(R.dcm_[0][0], 0.8519, 1.0e-3);
    EXPECT_NEAR(R.dcm_[0][1], 0.3704, 1.0e-3);
    EXPECT_NEAR(R.dcm_[0][2], -0.3704, 1.0e-3);
    EXPECT_NEAR(R.dcm_[1][0], 0.0741, 1.0e-3);
    EXPECT_NEAR(R.dcm_[1][1], 0.6148, 1.0e-3);
    EXPECT_NEAR(R.dcm_[1][2], 0.7852, 1.0e-3);
    EXPECT_NEAR(R.dcm_[2][0], 0.5185, 1.0e-3);
    EXPECT_NEAR(R.dcm_[2][1], -0.6963, 1.0e-3);
    EXPECT_NEAR(R.dcm_[2][2], 0.4963, 1.0e-3);

}

// Convert Quaternion to Euler Angles: Nominal Case
TEST(ConvertQuaternion, ToEuler_Nominal)
{

    // Create Quaternion Object
    Quaternion q(0.7071, 0.7071, 0.0, 0.0);

    // Noramlize Quaternion
    q.normalize();

    // Convert to Euler Angles
    EulerAngles eul = q.toEuler();

    // Check Values
    EXPECT_EQ(eul.pitch_, 0);
    EXPECT_NEAR(eul.roll_, 1.5708, 1.0e-3);
    EXPECT_EQ(eul.yaw_, 0);

}

// Convert Quaternion to Euler Angles: +90 deg Gimbal Lock Case
TEST(ConvertQuaternion, ToEuler_Plus90Gimbal)
{

    // Create Quaternion Object
    Quaternion q(0.7071, 0.0, 0.7071, 0.0);

    // Noramlize Quaternion
    q.normalize();

    // Convert to Euler Angles
    EulerAngles eul = q.toEuler();

    // Check Values
    EXPECT_NEAR(eul.pitch_, 90*(M_PI / 180.0), 1.0e-3);
    EXPECT_EQ(eul.roll_, 0);
    EXPECT_NEAR(eul.yaw_, 0.0, 1.0e-3);

}

// Convert Quaternion to Euler Angles: -90 deg Gimbal Lock Case
TEST(ConvertQuaternion, ToEuler_Minus90Gimbal)
{

    // Create Quaternion Object
    Quaternion q(0.0, 0.7071, 0.0, 0.7071);

    // Noramlize Quaternion
    q.normalize();

    // Convert to Euler Angles
    EulerAngles eul = q.toEuler();

    // Check Values
    EXPECT_NEAR(eul.pitch_, -90*(M_PI / 180.0), 1.0e-3);
    EXPECT_EQ(eul.roll_, 0);
    EXPECT_NEAR(eul.yaw_, 2.0*1.5708, 1.0e-3);

}

// Convert Quaternion to Rotation Vector: 0 Magnitude Case
TEST(ConvertQuaternion, ToRotVec_ZeroMag)
{

    // Create Quaternion Object
    Quaternion q(1.0, 0.0, 0.0, 0.0);

    // Convert to Rotation Vector
    RotationVector rotVec = q.toRotationVector();

    // Check Values
    EXPECT_NEAR(rotVec.rv_[0], 0.0, 1.0e-12);
    EXPECT_NEAR(rotVec.rv_[1], 0.0, 1.0e-12);
    EXPECT_NEAR(rotVec.rv_[2], 0.0, 1.0e-12);

}

// Convert Quaternion to Rotation Vector: General Case
TEST(ConvertQuaternion, ToRotVec)
{

    // Create Quaternion Object
    Quaternion q(1.0, 2.0, 3.0, 4.0);

    // Normalize Quaternion
    q.normalize();

    // Convert to Rotation Vector
    RotationVector rotVec = q.toRotationVector();

    // Check Values
    EXPECT_NEAR(rotVec.rv_[0], 1.0303806, 1.0e-6);
    EXPECT_NEAR(rotVec.rv_[1], 1.5455709, 1.0e-6);
    EXPECT_NEAR(rotVec.rv_[2], 2.0607612, 1.0e-6);

}