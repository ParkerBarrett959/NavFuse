//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Attitude Class Unit Testing                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Attitude Class Unit Tests.                                                          //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "Attitude.hpp"
#include <Eigen/Dense>

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

// Compute DCM from Quaternion: Incorrect DCM Size
TEST(ComputeDcmFromQuat, incorrectDcmSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd RA2B = Eigen::MatrixXd::Zero(3, 4);

    // DCM has Incorrect Number of Columns
    ASSERT_FALSE(att.computeDcmFromQuaternion(qA2B, RA2B));

    // Redefine RA2B
    RA2B = Eigen::MatrixXd::Zero(4, 3);

    // DCM has Incorrect Number of Rows
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

}

// Compute Quaternion from DCM: Incorrect Quaternion Size
TEST(ComputeQuatFromDcm, incorrectQuaternionSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd RA2B = Eigen::MatrixXd::Zero(4, 4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(att.computeQuaternionFromDcm(RA2B, qA2B));

}

// Compute Quaternion from DCM: Incorrect DCM Size
TEST(ComputeQuatFromDcm, incorrectDcmSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd RA2B = Eigen::MatrixXd::Zero(3, 4);

    // DCM has Incorrect Number of Columns
    ASSERT_FALSE(att.computeQuaternionFromDcm(RA2B, qA2B));

    // Redefine RA2B
    RA2B = Eigen::MatrixXd::Zero(4, 3);

    // DCM has Incorrect Number of Rows
    ASSERT_FALSE(att.computeQuaternionFromDcm(RA2B, qA2B));

}

// Compute Quaternion from DCM
TEST(ComputeQuatFromDcm, ComputeResult)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B(4);
    Eigen::MatrixXd RA2B(3, 3);
    RA2B << -0.667,  0.703,  0.245,
            -0.028, -0.354,  0.934,
             0.744,  0.616,  0.256;

    // Successfully Compute DCM
    EXPECT_TRUE(att.computeQuaternionFromDcm(RA2B, qA2B));

    // Define Expected Solutions
    Eigen::VectorXd qA2BSol(4);
    qA2BSol << 0.2422177, 0.3277063, 0.5135942, 0.7544866;
    
    // Check Results
    EXPECT_TRUE(qA2B.isApprox(qA2BSol, 1e-6));

}

// Compute Quaternion from Rotation Vector: Incorrect Quaternion Size
TEST(ComputeQuatFromRotVec, incorrectQuaternionSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd phi = Eigen::VectorXd::Zero(3);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(att.computeQuaternionFromRotationVec(phi, qA2B));

}

// Compute Quaternion from Rotation Vector: Incorrect Rotation Vector Size
TEST(ComputeQuatFromRotVec, incorrectRotationVectorSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd phi = Eigen::VectorXd::Zero(4);

    // Rotation Vector has Incorrect Number of Rows
    ASSERT_FALSE(att.computeQuaternionFromRotationVec(phi, qA2B));

}

// Compute Quaternion from Rotation Vector
TEST(ComputeQuatFromRotVec, ComputeResult)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B(4);
    Eigen::VectorXd phi(3);
    phi << 0.384, 1.384, 4.291;

    // Successfully Compute Quaternion
    EXPECT_TRUE(att.computeQuaternionFromRotationVec(phi, qA2B));

    // Define Expected Solutions
    Eigen::VectorXd qA2BSol(4);
    qA2BSol << -0.637848847370359, 0.0653574088345108, 0.235558994341049, 0.730335003408557;

    // Check Results
    EXPECT_TRUE(qA2B.isApprox(qA2BSol, 1e-6));

}

// Compute Quaternion Equivalent Matrix: Incorrect Quaternion Size
TEST(ComputeQuatEquivalent, incorrectQuaternionSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd QA2B = Eigen::MatrixXd::Zero(4, 4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(att.buildQuaternionEquivalent(qA2B, QA2B));

}

// Compute Quaternion Equivalent Matrix: Incorrect Quaternion Equivalent Matrix Size
TEST(ComputeQuatEquivalent, incorrectQuaternionMatrixSize)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd QA2B = Eigen::MatrixXd::Zero(4, 3);

    // Quaternion Matrix has Incorrect Number of Columns
    ASSERT_FALSE(att.buildQuaternionEquivalent(qA2B, QA2B));

    // Reset Quaternion Equivalent Matrix
    QA2B = Eigen::MatrixXd::Zero(3, 4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(att.buildQuaternionEquivalent(qA2B, QA2B));

}

// Compute Quaternion Equivalent Matrix
TEST(ComputeQuatEquivalent, ComputeResult)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::VectorXd qA2B(4);
    qA2B << 0.275, 0.372, 0.583, 0.856;
    Eigen::MatrixXd QA2B(4, 4);

    // Successfully Compute Quaternion Equivalent
    EXPECT_TRUE(att.buildQuaternionEquivalent(qA2B, QA2B));

    // Define Expected Solutions
    Eigen::MatrixXd QA2BSol(4, 4);
    QA2BSol << 0.275,  -0.372,  -0.583,  -0.856,
               0.372,   0.275,  -0.856,   0.583,
               0.583,   0.856,   0.275,  -0.372,
               0.856,  -0.583,   0.372,   0.275;

    // Check Results
    EXPECT_TRUE(QA2B.isApprox(QA2BSol, 1e-6));

}

// Compute DCM from Euler
TEST(ComputeEuler2Dcm, ComputeResult)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::Vector3d euler(3);
    euler << 0.349, 0.125, 2.094;
    Eigen::Matrix3d RB2N(3, 3);

    // Successfully Compute DCM
    EXPECT_TRUE(att.euler2Dcm(euler, RB2N));

    // Define Expected Solutions
    Eigen::MatrixXd RB2NSol(3, 3);
    RB2NSol <<  -0.49575929590605,  0.85946432815914, -0.124674733385228,
                -0.83530495080520, -0.43260583128062,  0.339290191285157,
                 0.23767279962808,  0.27234768837635,  0.932383170672339;

    // Check Results
    EXPECT_TRUE(RB2N.isApprox(RB2NSol, 1e-6));

}

// Compute Euler2DCM: Singularity
TEST(ComputeDcm2Euler, Singularity)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::Matrix3d RB2N(3, 3);
    RB2N <<   0.00000000000000,  0.85946432815914, -0.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;
    Eigen::Vector3d euler;

    // Singularity in R11
    EXPECT_FALSE(att.dcm2Euler(RB2N, euler));
    
    // Redefine Matrix
    RB2N <<  -0.49575929590605,  0.85946432815914, -0.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.000000000000000;

    // Singularity in R33
    EXPECT_FALSE(att.dcm2Euler(RB2N, euler));

}

// Compute Euler2DCM: Arcsin Limits
TEST(ComputeDcm2Euler, ArcsinDomain)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::Matrix3d RB2N(3, 3);
    RB2N <<  -0.49575929590605,  0.85946432815914,  1.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;
    Eigen::Vector3d euler;

    // R13 > 1
    EXPECT_FALSE(att.dcm2Euler(RB2N, euler));
    
    // Redefine Matrix
    RB2N <<  -0.49575929590605,  0.85946432815914, -1.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;

    // R13 < -1
    EXPECT_FALSE(att.dcm2Euler(RB2N, euler));

}

// Compute Euler2DCM
TEST(ComputeDcm2Euler, ComputeResult)
{

    // Create Attitude Object
    Attitude att;

    // Initialize Variables
    Eigen::Matrix3d RB2N(3, 3);
    RB2N <<  -0.49575929590605,  0.85946432815914, -0.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;
    Eigen::Vector3d euler;

    // Successfully Compute Euler Angles
    EXPECT_TRUE(att.dcm2Euler(RB2N, euler));
    
    // Define Expected Solutions
    Eigen::Vector3d eulerSol(3);
    eulerSol << 0.349, 0.125, 2.094;

    // Check Results
    EXPECT_TRUE(euler.isApprox(eulerSol, 1e-3));

}