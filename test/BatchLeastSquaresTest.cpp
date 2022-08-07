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
    Eigen::MatrixXd Hk = Eigen::MatrixXd::Zero(2,5);
    Eigen::VectorXd yk = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    double J = 0.0;

    // H Matrix has Incorrect Number of Columns
    EXPECT_FALSE(bls.UnweightedLinearLeastSquares(yk, Hk, xk, J));

    // H Matrix has Incorrect Number of Rows

    // H Matrix has More Columns than Rows

}
