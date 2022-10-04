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
               0,      0,      0,     0,      0,      0,      0,      0,     0;

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
TEST(EcefErrorDynamics, ComputeMatrices)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aE(3);
    aE << 0.362, 0.474, 0.367;
    Eigen::Matrix3d GE(3, 3);
    GE << 0.284, 0.001, 0.012,
          0.028, 0.274, 0.008,
          0.001, 0.003, 0.736;
    Eigen::Matrix3d RS2E = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Matrix3d OEI_I(3, 3);
    OEI_I <<      0, -0.273,  0.163,
              0.273,      0, -0.573,
             -0.163,  0.573,      0;
    Eigen::MatrixXd F(9, 9);
    Eigen::MatrixXd G(9, 9);

    // Compute Dynamics
    EXPECT_TRUE(pva.ecefErrorDynamics(aE, GE, RS2E, OEI_I, F, G));

    // Define Expected Solutions
    Eigen::MatrixXd FSol(9, 9);
    FSol <<    0,  0.273, -0.163,     0,      0,      0,         0,         0,         0,
          -0.273,      0,  0.573,     0,      0,      0,         0,         0,         0,
           0.163, -0.573,      0,     0,      0,      0,         0,         0,         0,
               0, -0.367,  0.474,     0,  0.546, -0.326,  0.385098, -0.092399, -0.144429,
           0.367,      0, -0.362,-0.546,      0,  1.146, -0.065399,  0.676858, -0.036499,
          -0.474,  0.362,      0, 0.326, -1.146,      0, -0.155429, -0.041499,  1.090898,
               0,      0,      0,     1,      0,      0,         0,         0,         0,
               0,      0,      0,     0,      1,      0,         0,         0,         0,
               0,      0,      0,     0,      0,      1,         0,         0,         0;
    Eigen::MatrixXd GSol(9, 9);
    GSol <<   -1,      0,      0,     0,      0,      0,      0,      0,     0,
               0,     -1,      0,     0,      0,      0,      0,      0,     0,
               0,      0,     -1,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     1,      0,      0,      1,      0,     0,
               0,      0,      0,     0,      1,      0,      0,      1,     0,
               0,      0,      0,     0,      0,      1,      0,      0,     1,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     0;

    // Correct F Matrix
    EXPECT_TRUE(F.isApprox(FSol, 1e-6));

    // Correct G Matrix
    EXPECT_TRUE(G.isApprox(GSol, 1e-6));

}

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
TEST(NedErrorDynamics, ComputeMatrices)
{

    // Create PVA Error Dynamics Object
    PvaErrorDynamics pva;

    // Initialize Variables
    Eigen::Vector3d aN(3);
    aN << 0.362, 0.474, 0.367;
    Eigen::Matrix3d GN(3, 3);
    GN << 0.284, 0.001, 0.012,
          0.028, 0.274, 0.008,
          0.001, 0.003, 0.736;
    Eigen::Matrix3d RS2N = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Vector3d lla(3);
    lla << 0.731, -1.240, 4.364;
    Eigen::Vector3d llaDot(3);
    llaDot << 0.002, 0.017, 0.001;
    Eigen::Vector3d llaDDot(3);
    llaDDot << 0.012, 0.004, 0.005;
    Eigen::MatrixXd F(9, 9);
    Eigen::MatrixXd G(9, 9);

    // Compute Dynamics
    EXPECT_TRUE(pva.nedErrorDynamics(aN, GN, RS2N, lla, llaDot, llaDDot, F, G));

    // Define Expected Solutions
    Eigen::MatrixXd FSol(9, 9);
    FSol <<       0, -0.011398,     0.002,             0,      0.744507,         0,       -0.011398,             0,         0,
           0.011398,         0,  0.012711,            -1,             0,         0,               0,             0,         0,
             -0.002, -0.012711,         0,             0,     -0.667614,         0,       -0.012711,             0,         0,
                  0, -5.77e-08,  7.43e-08,     -1.37e-06,     -0.016972, -6.27e-10,    -3.16494e-05,             0, -1.89e-09,
           7.75e-08,         0, -7.63e-08,      0.030619,      0.003587, -5.36e-09,        0.003655,             0, -6.18e-10,
              0.474,    -0.362,         0,  25503.121498, 120672.674050,         0,    -1847.420312,             0,  0.736166,
                  0,         0,         0,             1,             0,         0,               0,             0,         0,
                  0,         0,         0,             0,             1,         0,               0,             0,         0,
                  0,         0,         0,             0,             0,         1,               0,             0,         0;
    Eigen::MatrixXd GSol(9, 9);
    GSol <<   -1,      0,      0,     0,      0,      0,      0,      0,     0,
               0,     -1,      0,     0,      0,      0,      0,      0,     0,
               0,      0,     -1,     0,      0,      0,      0,      0,     0,
               0,      0,      0,  1.57e-07,  0,      0,   1.57e-07,  0,     0,
               0,      0,      0,     0,   2.10e-07,  0,      0,   2.10e-07, 0,
               0,      0,      0,     0,      0,     -1,      0,      0,    -1,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     0,
               0,      0,      0,     0,      0,      0,      0,      0,     0;
    
    // Correct F Matrix
    EXPECT_TRUE(F.isApprox(FSol, 1e-6));

    // Correct G Matrix
    EXPECT_TRUE(G.isApprox(GSol, 1e-6));

}