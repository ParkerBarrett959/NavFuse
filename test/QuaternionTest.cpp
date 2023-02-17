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
#include "Quaternion.hpp"
#include <Eigen/Dense>

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

/*
// Compute DCM from Quaternion: Incorrect Quaternion Size
TEST(ComputeDcmFromQuat, incorrectQuaternionSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd RA2B = Eigen::MatrixXd::Zero(4, 4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(att.computeDcmFromQuaternion(qA2B, RA2B));

}

// Compute DCM from Quaternion
TEST(ComputeDcmFromQuat, ComputeResult)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B(4);
    qA2B << 0.275, 0.372, 0.583, 0.856;
    Eigen::MatrixXd RA2B(3, 3);

    // Successfully Compute DCM
    EXPECT_TRUE(att.computeDcmFromQuaternion(qA2B, RA2B));

    // Define Expected Solutions
    Eigen::MatrixXd RA2BSol(3, 3);
    RA2BSol << -0.667335077419064,      0.703037538258743,       0.245768415882061,
               -0.028794513435833,     -0.354106917740399,       0.934761556122409,
                0.744200759501148,      0.616722393470093,       0.256551591206202;

    // Check Results
    EXPECT_TRUE(RA2B.isApprox(RA2BSol, 1e-6));

}*/