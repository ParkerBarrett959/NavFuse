//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Unscented Kalman Filter Unit Testing                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Unscented Kalman Filter Class Unit Tests.                                           //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "UnscentedKalmanFilter.hpp"
#include <Eigen/Dense>

// Filter Initialize: Incorrect P0 Dimensions
TEST(FilterUkfInitialize, IncorrectP0Dimensions)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Set State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,3);

    // P0 Matrix has Incorrect Number of Columns
    ASSERT_FALSE(ukf.filterInitialize(x0, P0));

    // Redefine P0
    P0 = Eigen::MatrixXd::Zero(3, 4);

    // Pk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterInitialize(x0, P0));

}

// Filter Initialize: Set Values
TEST(FilterUkfInitialize, SetValues)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Set State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Values
    ukf.filterInitialize(x0, P0);

    // Get Output Values
    Eigen::VectorXd xOut = ukf.getState();
    Eigen::MatrixXd POut = ukf.getCovariance();

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

// Filter UKF Predict: Incorrect Wi Dimensions
TEST(FilterUkfPredict, IncorrectWiDimensions)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(3, 7);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(3, 3);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(3,3);
    ukf.filterInitialize(x0, P0);

    // Wi Vector has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterUkfPredict(Wi, yi, Qk));

}

// Filter UKF Predict: Incorrect yi Dimensions
TEST(FilterUkfPredict, IncorrectyiDimensions)
{

    // Create Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(3, 3);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(3,3);
    ukf.filterInitialize(x0, P0);

    // yi Matrix has Incorrect Number of Columns
    ASSERT_FALSE(ukf.filterUkfPredict(Wi, yi, Qk));

    // Redefine yi
    yi = Eigen::MatrixXd::Zero(4, 7);

    // yi Matrix has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterUkfPredict(Wi, yi, Qk));

}

// Filter UKF Predict: Incorrect Qk Dimensions
TEST(FilterUkfPredict, IncorrectQkDimensions)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(3, 7);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(3, 4);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(3,3);
    ukf.filterInitialize(x0, P0);

    // Qk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(ukf.filterUkfPredict(Wi, yi, Qk));

    // Redefine Qk
    Qk = Eigen::MatrixXd::Zero(4, 3);

    // Qk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterUkfPredict(Wi, yi, Qk));

}

// Kalman Filter UKF Predict: Compute Kalman Filter UKF Prediction
TEST(FilterUkfPredict, ComputeFilterUkfPrediction)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::VectorXd Wi(9);
    Wi << 0.05, 0.10, 0.10, 0.20, 0.15, 0.20, 0.10, 0.10, 0.00;
    Eigen::MatrixXd yi(4, 9);
    yi << 0.21, 0.84, 0.19, 0.20, 0.92, 0.82, 0.02, 0.12, 0.36,
          0.67, 0.15, 0.13, 0.13, 0.11, 0.92, 0.28, 0.10, 0.23,
          0.25, 0.28, 0.93, 0.84, 0.15, 0.12, 0.15, 0.82, 0.73,
          0.31, 0.49, 0.19, 0.47, 0.63, 0.46, 0.89, 0.91, 0.16;
    Eigen::MatrixXd Qk(4, 4);
    Qk << 3.284, 0.000, 0.000, 0.000,
          0.000, 8.284, 0.000, 0.000,
          0.000, 0.000, 8.394, 0.000,
          0.000, 0.000, 0.000, 3.219;

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    ukf.filterInitialize(x0, P0);

    // Successfully Performed Filter Prediction 
    EXPECT_TRUE(ukf.filterUkfPredict(Wi, yi, Qk));
    
    // Get Output Values
    Eigen::VectorXd xkp1 = ukf.getState();
    Eigen::MatrixXd Pkp1 = ukf.getCovariance();
    
    // Define Expected Solutions
    Eigen::VectorXd xkSol(4);
    xkSol << 0.469500, 0.326000, 0.445000, 0.544000;
    Eigen::MatrixXd PkSol(4, 4);
    PkSol << 3.4109,    0.0421,   -0.0810,   -0.0135,
             0.0421,    8.3874,   -0.0616,   -0.0159,
            -0.0810,   -0.0616,    8.5103,   -0.0147,
            -0.0135,   -0.0159,   -0.0147,    3.2635;

    // Correct State
    EXPECT_TRUE(xkp1.isApprox(xkSol, 1e-6));

    // Correct Covariance
    EXPECT_TRUE(Pkp1.isApprox(PkSol, 1e-5));
 
}

// Filter UKF Update: Incorrect yi Dimensions
TEST(FilterUkfUpdate, IncorrectyiDimensions)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(4, 8);
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(9);
    Eigen::MatrixXd zi = Eigen::MatrixXd::Zero(2, 9);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    ukf.filterInitialize(x0, P0);

    // yi Matrix has Incorrect Number of Columns
    ASSERT_FALSE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

    // Redefine yi
    yi = Eigen::MatrixXd::Zero(3, 9);

    // yi Matrix has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

}

// Filter UKF Update: Incorrect Wi Dimensions
TEST(FilterUkfUpdate, IncorrectWiDimensions)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(4, 9);
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd zi = Eigen::MatrixXd::Zero(2, 9);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    ukf.filterInitialize(x0, P0);

    // Wi Vector has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

}

// Filter UKF Update: Incorrect zi Dimensions
TEST(FilterUkfUpdate, IncorrectziDimensions)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(4, 9);
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(9);
    Eigen::MatrixXd zi = Eigen::MatrixXd::Zero(2, 8);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    ukf.filterInitialize(x0, P0);

    // zi Matrix has Incorrect Number of Columns
    ASSERT_FALSE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

    // Redefine zi
    zi = Eigen::MatrixXd::Zero(3, 9);

    // zi Matrix has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

}

// Filter UKF Update: Incorrect Rk Dimensions
TEST(FilterUkfUpdate, IncorrectRkDimensions)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Initialize Variables
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(4, 9);
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(9);
    Eigen::MatrixXd zi = Eigen::MatrixXd::Zero(2, 9);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 3);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    ukf.filterInitialize(x0, P0);

    // Rk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

    // Redefine Rk
    Rk = Eigen::MatrixXd::Zero(3, 2);

    // Rk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

}

// Kalman Filter UKF Update: Compute Filter UKF Update
TEST(FilterUkfUpdate, ComputeFilterUkfUpdate)
{

    // Create Unscented Kalman Filter Object
    UnscentedKalmanFilter ukf;

    // Set Filter State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;
    ukf.filterInitialize(x0, P0);

    // Set Filter Update Inputs
    Eigen::VectorXd Wi(9);
    Wi << 0.05, 0.10, 0.10, 0.20, 0.15, 0.20, 0.10, 0.10, 0.00;
    Eigen::MatrixXd yi(4, 9);
    yi << 0.21, 0.84, 0.19, 0.20, 0.92, 0.82, 0.02, 0.12, 0.36,
          0.67, 0.15, 0.13, 0.13, 0.11, 0.92, 0.28, 0.10, 0.23,
          0.25, 0.28, 0.93, 0.84, 0.15, 0.12, 0.15, 0.82, 0.73,
          0.31, 0.49, 0.19, 0.47, 0.63, 0.46, 0.89, 0.91, 0.16;
    Eigen::MatrixXd zi(2, 9);
    zi << 101.738, 101.848, 101.482, 101.942, 101.039, 101.323, 101.583, 101.473, 101.484,
          104.738, 104.848, 104.482, 104.942, 104.039, 104.323, 104.583, 104.473, 104.484;
    Eigen::VectorXd zk(2);
    zk << 101.744, 104.384;
    Eigen::MatrixXd Rk(2, 2);
    Rk << 4.029, 0.000,
          0.000, 6.294;

    // Successfully Performed Filter Update 
    EXPECT_TRUE(ukf.filterUkfUpdate(yi, Wi, zi, zk, Rk));

    // Get Output Values
    Eigen::VectorXd xkp1 = ukf.getState();
    Eigen::MatrixXd Pkp1 = ukf.getCovariance();
    
    // Define Expected Solutions
    Eigen::VectorXd xkSol(4);
    xkSol << 3.450376, 8.374325, 5.286477, 9.452684;
    Eigen::MatrixXd PkSol(4, 4);
    PkSol << 4.6736,   -0.0006,    0.0013,   -0.0003,
            -0.0006,    6.2948,    0.0005,   -0.0001,
             0.0013,    0.0005,    8.2928,    0.0002,
            -0.0003,   -0.0001,    0.0002,    4.3849;

    // Correct State
    EXPECT_TRUE(xkp1.isApprox(xkSol, 1e-6));

    // Correct Covariance
    EXPECT_TRUE(Pkp1.isApprox(PkSol, 1e-4));
 
}