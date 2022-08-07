//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Batch Least Squares Unit Testing                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Batch Least Squares Class Unit Tests.                                               //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "BatchLeastSquares.hpp"
#include <Eigen/Dense>

// Unweighted Linear Least Squares: Incorrect H Dimensions
TEST(UnweightedLinearLeastSquares, IncorrectHDimensions)
{

    // Create Least Squares Object
    BatchLeastSquares bls;

    // Initialize Variables
    Eigen::MatrixXd Hk = Eigen::MatrixXd::Zero(2, 5);
    Eigen::VectorXd yk = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    double J;

    // H Matrix has Incorrect Number of Columns
    ASSERT_FALSE(bls.UnweightedLinearLeastSquares(yk, Hk, xk, J));

    // Redefine H
    Hk = Eigen::MatrixXd::Zero(3, 4);

    // H Matrix has Incorrect Number of Rows
    ASSERT_FALSE(bls.UnweightedLinearLeastSquares(yk, Hk, xk, J));

    // Redefine H
    Hk = Eigen::MatrixXd::Zero(2, 4);

    // H Matrix has More Columns than Rows
    ASSERT_FALSE(bls.UnweightedLinearLeastSquares(yk, Hk, xk, J));

}

// Unweighted Linear Least Squares: Compute Least Squares Estimate
TEST(UnweightedLinearLeastSquares, ComputeLeastSquaresEstimate)
{

    // Create Least Squares Object
    BatchLeastSquares bls;

    // Initialize Variables
    Eigen::MatrixXd Hk(9, 3);
    Hk << 1.07, 4.56, 2.21,
          0.74, 1.45, 3.46,
          2.12, 6.65, 9.45,
	  1.07, 4.56, 2.21,
          0.74, 1.45, 3.46,
          2.12, 6.65, 9.45,
	  1.07, 4.56, 2.21,
          0.74, 1.45, 3.46,
          2.12, 6.65, 9.45;
    Eigen::VectorXd yk(9);
    yk << 3.5624, 7.3716, 2.2897,
          3.2618, 7.1234, 2.0987,
          3.7215, 7.3546, 2.6815;
    Eigen::VectorXd xk(3); 
    double J;

    // Successfully Solved Unweighted Linear Batch Least Squares
    EXPECT_TRUE(bls.UnweightedLinearLeastSquares(yk, Hk, xk, J));

    // Define Expected Solutions
    Eigen::VectorXd xSol(3);
    xSol << 48.635615, -8.307677, -4.815325;
    double JSol = 0.161998;

    // Correct Least Squares Solution
    EXPECT_TRUE(xk.isApprox(xSol, 1e-6));

    // Correct Cost Value
    EXPECT_NEAR(J, JSol, 1e-6);

}

