//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Navigation Utilities Unit Testing                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Navigation Utility Class Unit Tests.                                                //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "NavUtils.hpp"
#include <Eigen/Dense>

// Compute DCM from Quaternion: Incorrect Quaternion Size
TEST(ComputeDcmFromQuat, incorrectQuaternionSize)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd RA2B = Eigen::MatrixXd::Zero(3, 4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(util.computeDcmFromQuaternion(qA2B, RA2B));

}

// Compute DCM from Quaternion: Incorrect DCM Size
TEST(ComputeDcmFromQuat, incorrectDcmSize)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd RA2B = Eigen::MatrixXd::Zero(3, 4);

    // DCM has Incorrect Number of Columns
    ASSERT_FALSE(util.computeDcmFromQuaternion(qA2B, RA2B));

    // Redefine RA2B
    RA2B = Eigen::MatrixXd::Zero(4, 3);

    // DCM has Incorrect Number of Rows
    ASSERT_FALSE(util.computeDcmFromQuaternion(qA2B, RA2B));

}

// Compute DCM from Quaternion
TEST(ComputeDcmFromQuat, ComputeResult)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B(4);
    qA2B << 0.275, 0.372, 0.583, 0.856;
    Eigen::MatrixXd RA2B(3, 3);

    // Successfully Compute DCM
    EXPECT_TRUE(util.computeDcmFromQuaternion(qA2B, RA2B));

    // Define Expected Solutions
    Eigen::MatrixXd RA2BSol(3, 3);
    RA2BSol << -0.667335077419064,      0.703037538258743,       0.245768415882061,
               -0.028794513435833,     -0.354106917740399,       0.934761556122409,
                0.744200759501148,      0.616722393470093,       0.256551591206202;
;

    // Check Results
    EXPECT_TRUE(RA2B.isApprox(RA2BSol, 1e-6));

}

// Compute Quaternion from Rotation Vector: Incorrect Quaternion Size
TEST(ComputeQuatFromRotVec, incorrectQuaternionSize)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd phi = Eigen::VectorXd::Zero(3);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(util.computeQuaternionFromRotationVec(phi, qA2B));

}

// Compute Quaternion from Rotation Vector: Incorrect Rotation Vector Size
TEST(ComputeQuatFromRotVec, incorrectRotationVectorSize)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd phi = Eigen::VectorXd::Zero(4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(util.computeQuaternionFromRotationVec(phi, qA2B));

}

// Compute Quaternion from Rotation Vector
TEST(ComputeQuatFromRotVec, ComputeResult)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B(4);
    Eigen::VectorXd phi(3);
    phi << 0.384, 1.384, 4.291;

    // Successfully Compute Quaternion
    EXPECT_TRUE(util.computeQuaternionFromRotationVec(phi, qA2B));

    // Define Expected Solutions
    Eigen::VectorXd qA2BSol(4);
    qA2BSol << -0.637848847370359, 0.0653574088345108, 0.235558994341049, 0.730335003408557;

    // Check Results
    EXPECT_TRUE(qA2B.isApprox(qA2BSol, 1e-6));

}