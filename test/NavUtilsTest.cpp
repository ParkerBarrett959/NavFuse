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

    // Rotation Vector has Incorrect Number of Rows
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

// Compute Quaternion Equivalent Matrix: Incorrect Quaternion Size
TEST(ComputeQuatEquivalent, incorrectQuaternionSize)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd QA2B = Eigen::MatrixXd::Zero(4, 4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(util.buildQuaternionEquivalent(qA2B, QA2B));

}

// Compute Quaternion Equivalent Matrix: Incorrect Quaternion Equivalent Matrix Size
TEST(ComputeQuatEquivalent, incorrectQuaternionMatrixSize)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd QA2B = Eigen::MatrixXd::Zero(4, 3);

    // Quaternion Matrix has Incorrect Number of Columns
    ASSERT_FALSE(util.buildQuaternionEquivalent(qA2B, QA2B));

    // Reset Quaternion Equivalent Matrix
    QA2B = Eigen::MatrixXd::Zero(3, 4);

    // Quaternion has Incorrect Number of Rows
    ASSERT_FALSE(util.buildQuaternionEquivalent(qA2B, QA2B));

}

// Compute Quaternion Equivalent Matrix
TEST(ComputeQuatEquivalent, ComputeResult)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::VectorXd qA2B(4);
    qA2B << 0.275, 0.372, 0.583, 0.856;
    Eigen::MatrixXd QA2B(4, 4);

    // Successfully Compute Quaternion Equivalent
    EXPECT_TRUE(util.buildQuaternionEquivalent(qA2B, QA2B));

    // Define Expected Solutions
    Eigen::MatrixXd QA2BSol(4, 4);
    QA2BSol << 0.275,  -0.372,  -0.583,  -0.856,
               0.372,   0.275,  -0.856,   0.583,
               0.583,   0.856,   0.275,  -0.372,
               0.856,  -0.583,   0.372,   0.275;

    // Check Results
    EXPECT_TRUE(QA2B.isApprox(QA2BSol, 1e-6));

}

// Compute Skew Matrix
TEST(ComputeSkewSymmetric, ComputeResult)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Vector3d vec(3);
    vec << 1.0, 2.0, 3.0;
    Eigen::Matrix3d vecX(3, 3);

    // Successfully Compute Skew Symmetric
    EXPECT_TRUE(util.skewSymmetric(vec, vecX));

    // Define Expected Solutions
    Eigen::MatrixXd vecXSol(3, 3);
    vecXSol <<  0.0, -3.0,  2.0,
                3.0,  0.0, -1.0,
               -2.0,  1.0,  0.0;

    // Check Results
    EXPECT_TRUE(vecX.isApprox(vecXSol, 1e-6));

}

// Compute DCM from RPH
TEST(ComputeRph2Dcm, ComputeResult)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Vector3d rph(3);
    rph << 0.349, 0.125, 2.094;
    Eigen::Matrix3d RB2N(3, 3);

    // Successfully Compute DCM
    EXPECT_TRUE(util.rph2Dcm(rph, RB2N));

    // Define Expected Solutions
    Eigen::MatrixXd RB2NSol(3, 3);
    RB2NSol <<  -0.49575929590605,  0.85946432815914, -0.124674733385228,
                -0.83530495080520, -0.43260583128062,  0.339290191285157,
                 0.23767279962808,  0.27234768837635,  0.932383170672339;

    // Check Results
    EXPECT_TRUE(RB2N.isApprox(RB2NSol, 1e-6));

}

// Compute RPH2DCM: Singularity
TEST(ComputeDcm2Rph, Singularity)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Matrix3d RB2N(3, 3);
    RB2N <<   0.00000000000000,  0.85946432815914, -0.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;
    Eigen::Vector3d rph;

    // Singularity in R11
    EXPECT_FALSE(util.dcm2Rph(RB2N, rph));
    
    // Redefine Matrix
    RB2N <<  -0.49575929590605,  0.85946432815914, -0.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.000000000000000;

    // Singularity in R33
    EXPECT_FALSE(util.dcm2Rph(RB2N, rph));

}

// Compute RPH2DCM: Arcsin Limits
TEST(ComputeDcm2Rph, ArcsinDomain)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Matrix3d RB2N(3, 3);
    RB2N <<  -0.49575929590605,  0.85946432815914,  1.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;
    Eigen::Vector3d rph;

    // R13 > 1
    EXPECT_FALSE(util.dcm2Rph(RB2N, rph));
    
    // Redefine Matrix
    RB2N <<  -0.49575929590605,  0.85946432815914, -1.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;

    // R13 < -1
    EXPECT_FALSE(util.dcm2Rph(RB2N, rph));

}

// Compute RPH2DCM
TEST(ComputeDcm2Rph, ComputeResult)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Matrix3d RB2N(3, 3);
    RB2N <<  -0.49575929590605,  0.85946432815914, -0.124674733385228,
             -0.83530495080520, -0.43260583128062,  0.339290191285157,
              0.23767279962808,  0.27234768837635,  0.932383170672339;
    Eigen::Vector3d rph;

    // Successfully Compute RPH
    EXPECT_TRUE(util.dcm2Rph(RB2N, rph));
    
    // Define Expected Solutions
    Eigen::Vector3d rphSol(3);
    rphSol << 0.349, 0.125, 2.094;

    // Check Results
    EXPECT_TRUE(rph.isApprox(rphSol, 1e-3));

}

// Compute RK4 Strapdown Dynamics: Incorrect ykm1 Dimensions
TEST(ComputeStrapdownRk4, IncorrectDimensionsYkm1)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Vector3d gN(3);
    Eigen::Vector3d dVN(3);
    double dt = 10.000;
    Eigen::VectorXd ykm1(5);
    Eigen::VectorXd yk(6);

    // Incorrect Dimensions
    EXPECT_FALSE(util.strapdownRk4(ykm1, dt, dVN, gN, yk));

}

// Compute RK4 Strapdown Dynamics: Incorrect yk Dimensions
TEST(ComputeStrapdownRk4, IncorrectDimensionsYk)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Vector3d gN(3);
    Eigen::Vector3d dVN(3);
    double dt = 10.000;
    Eigen::VectorXd ykm1(6);
    Eigen::VectorXd yk(5);

    // Incorrect Dimensions
    EXPECT_FALSE(util.strapdownRk4(ykm1, dt, dVN, gN, yk));

}

// Compute RK4 Strapdown Dynamics
TEST(ComputeStrapdownRk4, ComputeResult)
{

    // Create Nav Util Object
    NavUtils util;

    // Initialize Variables
    Eigen::Vector3d gN(3);
    gN << 0.072, 0.013, 9.798;
    Eigen::Vector3d dVN(3);
    dVN << 0.002, 0.274, 0.162;
    double dt = 10.000;
    Eigen::VectorXd ykm1(6);
    ykm1 << 0.739, -1.240, 12.019, 2.928, 1.472, 0.005;
    Eigen::VectorXd yk(6);
    
    // Successfully Compute Dynamic Integration
    EXPECT_TRUE(util.strapdownRk4(ykm1, dt, dVN, gN, yk));
    
    // Define Expected Solutions
    Eigen::VectorXd ykSol(6);
    ykSol << 0.739005, -1.239996, -478.732187, 3.648595, 1.932334, 98.145153;

    // Check Results
    EXPECT_TRUE(yk.isApprox(ykSol, 1e-6));

}