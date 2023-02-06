//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Base Kalman Filter Unit Testing                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Base Filter Class Unit Tests.                                                       //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "BaseFilter.hpp"
#include <Eigen/Dense>

// Filter Initialize: Incorrect P0 Dimensions
TEST(BaseFilterInitialize, IncorrectP0Dimensions)
{

    // Create Base Filter Object
    BaseFilter bf;

    // Set State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,3);

    // P0 Matrix has Incorrect Number of Columns
    ASSERT_FALSE(bf.filterInitialize(x0, P0));

    // Redefine P0
    P0 = Eigen::MatrixXd::Zero(3, 4);

    // Pk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(bf.filterInitialize(x0, P0));

}

// Filter Initialize: Set Values
TEST(BaseFilterInitialize, SetValues)
{

    // Create Base Filter Object
    BaseFilter bf;

    // Set State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Values
    bf.filterInitialize(x0, P0);

    // Get Output Values
    Eigen::VectorXd xOut = bf.getState();
    Eigen::MatrixXd POut = bf.getCovariance();

    // Define Expected Solutions
    Eigen::VectorXd xOutSol(4);
    xOutSol << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd POutSol(4, 4);
    POutSol << 4.675, 0.000, 0.000, 0.000,
               0.000, 6.295, 0.000, 0.000,
               0.000, 0.000, 8.294, 0.000,
               0.000, 0.000, 0.000, 4.385;

    // Correct State
    EXPECT_TRUE(xOut.isApprox(xOutSol, 1e-6));

    // Correct Covariance
    EXPECT_TRUE(POut.isApprox(POutSol, 1e-6));

}

// Get Filter Covariance
TEST(BaseFilterGetters, GetCovariance)
{

    // Create Base Filter Object
    BaseFilter bf;

    // Set State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Values
    bf.filterInitialize(x0, P0);

    // Get Output Values
    Eigen::MatrixXd POut = bf.getCovariance();

    // Correct Covariance
    EXPECT_TRUE(POut.isApprox(P0, 1e-6));

}

// Get Filter State
TEST(BaseFilterGetters, GetState)
{

    // Create Base Filter Object
    BaseFilter bf;

    // Set State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Values
    bf.filterInitialize(x0, P0);

    // Get Output Values
    Eigen::VectorXd xOut = bf.getState();

    // Correct State
    EXPECT_TRUE(xOut.isApprox(x0, 1e-6));

}