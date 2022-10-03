//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                   PVA Error Dynamics Unit Testing                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    PVA Error Dynamics Class Unit Tests.                                                //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "pvaErrorDynamics.hpp"
#include <Eigen/Dense>

// Inertial Error Dynamics: Incorrect Input Dimensions F
TEST(InertialErrorDynamics, IncorrectFDimensions)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aI = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d GI = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d RS2I = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(8, 9);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(9, 9);

    // Incorrect Number of Rows F
    EXPECT_FALSE(pva.inertialErrorDynamics(aI, GI, RS2I, F, G));

    // Redefine F
    F = Eigen::MatrixXd::Zero(9, 8);

    // Incorrect Number of Columns F
    EXPECT_FALSE(pva.inertialErrorDynamics(aI, GI, RS2I, F, G));

}

// Inertial Error Dynamics: Incorrect Input Dimensions G
TEST(InertialErrorDynamics, IncorrectGDimensions)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aI = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d GI = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d RS2I = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(9, 9);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(8, 9);

    // Incorrect Number of Rows G
    EXPECT_FALSE(pva.inertialErrorDynamics(aI, GI, RS2I, F, G));

    // Redefine F
    G = Eigen::MatrixXd::Zero(9, 8);

    // Incorrect Number of Columns G
    EXPECT_FALSE(pva.inertialErrorDynamics(aI, GI, RS2I, F, G));

}

// Inertial Error Dynamics: Compute Matrices
TEST(InertialErrorDynamics, ComputeMatrices)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aI(3);
    aI << 0.362, 0.474, 0.367;
    Eigen::Matrix3d GI(3, 3);
    GI << 0.284, 0.001, 0.012,
          0.028, 0.274, 0.008,
          0.001, 0.003, 0.736;
    Eigen::Matrix3d RS2I = Eigen::Matrix3d::Identity(3, 3);
    Eigen::MatrixXd F(9, 9);
    Eigen::MatrixXd G(9, 9);

    // Compute Dynamics
    EXPECT_TRUE(pva.inertialErrorDynamics(aI, GI, RS2I, F, G));

    // Define Expected Solutions
    Eigen::MatrixXd FSol(9, 9);
    FSol <<    0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0, -0.367,  0.474,     0,      0,      0,  0.284,  0.001,  0.012,
           0.367,      0, -0.362,     0,      0,      0,  0.028,  0.274,  0.008,
          -0.474,  0.362,      0,     0,      0,      0,  0.001,  0.003,  0.736,
               0,      0,      0,     1,      0,      0,      0,      0,      0,
               0,      0,      0,     0,      1,      0,      0,      0,      0,
               0,      0,      0,     0,      0,      1,      0,      0,      0;
    Eigen::MatrixXd GSol(9, 9);
    GSol <<   -1,      0,      0,     0,      0,      0,      0,      0,     0,
               0,     -1,      0,     0,      0,      0,      0,      0,     0,
               0,      0,     -1,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     1,      0,      0,      1,      0,     0,
               0,      0,      0,     0,      1,      0,      0,      1,     0,
               0,      0,      0,     0,      0,      1,      0,      0,     1,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     1;

    // Correct F Matrix
    EXPECT_TRUE(F.isApprox(FSol, 1e-6));

    // Correct G Matrix
    EXPECT_TRUE(G.isApprox(GSol, 1e-6));

}

// ECEF Error Dynamics: Incorrect Input Dimensions F
TEST(EcefErrorDynamics, IncorrectFDimensions)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aE = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d GE = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d RS2E = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d OEI_I = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(8, 9);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(9, 9);

    // Incorrect Number of Rows F
    EXPECT_FALSE(pva.ecefErrorDynamics(aE, GE, RS2E, OEI_I, F, G));

    // Redefine F
    F = Eigen::MatrixXd::Zero(9, 8);

    // Incorrect Number of Columns F
    EXPECT_FALSE(pva.ecefErrorDynamics(aE, GE, RS2E, OEI_I, F, G));

}

// ECEF Error Dynamics: Incorrect Input Dimensions G
TEST(EcefErrorDynamics, IncorrectGDimensions)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aE = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d GE = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d RS2E = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d OEI_I = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(9, 9);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(8, 9);

    // Incorrect Number of Rows G
    EXPECT_FALSE(pva.ecefErrorDynamics(aE, GE, RS2E, OEI_I, F, G));

    // Redefine F
    G = Eigen::MatrixXd::Zero(9, 8);

    // Incorrect Number of Columns G
    EXPECT_FALSE(pva.ecefErrorDynamics(aE, GE, RS2E, OEI_I, F, G));

}

// ECEF Error Dynamics: Compute Matrices

// NED Error Dynamics: Incorrect Input Dimensions F
TEST(NedErrorDynamics, IncorrectFDimensions)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aN = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d GN = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d RS2N = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Vector3d lla = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d llaDot = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d llaDDot = Eigen::Vector3d::Zero(3);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(8, 9);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(9, 9);

    // Incorrect Number of Rows F
    EXPECT_FALSE(pva.nedErrorDynamics(aN, GN, RS2N, lla, llaDot, llaDDot, F, G));

    // Redefine F
    F = Eigen::MatrixXd::Zero(9, 8);

    // Incorrect Number of Columns F
    EXPECT_FALSE(pva.nedErrorDynamics(aN, GN, RS2N, lla, llaDot, llaDDot, F, G));

}

// NED Error Dynamics: Incorrect Input Dimensions G
TEST(NedErrorDynamics, IncorrectGDimensions)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aN = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d GN = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d RS2N = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Vector3d lla = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d llaDot = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d llaDDot = Eigen::Vector3d::Zero(3);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(9, 9);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(8, 9);

    // Incorrect Number of Rows G
    EXPECT_FALSE(pva.nedErrorDynamics(aN, GN, RS2N, lla, llaDot, llaDDot, F, G));

    // Redefine F
    G = Eigen::MatrixXd::Zero(9, 8);

    // Incorrect Number of Columns G
    EXPECT_FALSE(pva.nedErrorDynamics(aN, GN, RS2N, lla, llaDot, llaDDot, F, G));

}

// NED Error Dynamics: Compute Matrices