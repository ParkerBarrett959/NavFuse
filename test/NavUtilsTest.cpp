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