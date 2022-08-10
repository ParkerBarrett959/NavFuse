//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Kalman Filter Unit Testing                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Kalman Filter Class Unit Tests.                                                     //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "KalmanFilter.hpp"
#include <Eigen/Dense>

// Filter Predict: Incorrect Pk Dimensions
TEST(FilterPredict, IncorrectPkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 3);
    Eigen::MatrixXd Phik = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // Pk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

    // Redefine Pk
    Pk = Eigen::MatrixXd::Zero(3, 4);

    // Pk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

}