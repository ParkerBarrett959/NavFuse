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

// Filter Initialize: Incorrect P0 Dimensions
TEST(FilterInitialize, IncorrectP0Dimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Set State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,3);

    // P0 Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterInitialize(x0, P0));

    // Redefine P0
    P0 = Eigen::MatrixXd::Zero(3, 4);

    // Pk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterInitialize(x0, P0));

}

// Filter Initialize: Set Values
TEST(FilterInitialize, SetValues)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Set State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Values
    kf.filterInitialize(x0, P0);

    // Get Output Values
    Eigen::VectorXd xOut = kf.getState();
    Eigen::MatrixXd POut = kf.getCovariance();

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

// Filter Predict: Incorrect Phik Dimensions
TEST(FilterPredict, IncorrectPhikDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::MatrixXd Phik = Eigen::MatrixXd::Zero(4, 3);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(4, 4);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    kf.filterInitialize(x0, P0);

    // Phik Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterPredict(Phik, Qk));

    // Redefine Phik
    Phik = Eigen::MatrixXd::Zero(3, 4);

    // Pk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterPredict(Phik, Qk));

}

// Filter Predict: Incorrect Qk Dimensions
TEST(FilterPredict, IncorrectQkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::MatrixXd Phik = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(4, 3);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    kf.filterInitialize(x0, P0);

    // Qk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterPredict(Phik, Qk));

    // Redefine Qk
    Qk = Eigen::MatrixXd::Zero(3, 4);

    // Qk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterPredict(Phik, Qk));

}

// Kalman Filter Predict: Compute Kalman Filter Prediction
TEST(FilterPredict, ComputeFilterPrediction)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Filter Values
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Filter State and Covariance
    kf.filterInitialize(x0, P0);

    // Set State Transition Matrix
    Eigen::MatrixXd Phik(4, 4);
    Phik << 3.674, 0.375, 0.000, 0.000,
            0.000, 4.284, 0.000, 8.294,
            1.329, 4.295, 6.295, 0.000,
            0.000, 0.000, 3.294, 7.184;

    // Set Process Noise Matrix
    Eigen::MatrixXd Qk(4, 4);
    Qk << 3.284, 0.000, 0.000, 0.000,
          0.000, 8.284, 0.000, 0.000,
          0.000, 0.000, 8.394, 0.000,
          0.000, 0.000, 0.000, 3.219;

    // Successfully Performed Filter Prediction 
    EXPECT_TRUE(kf.filterPredict(Phik, Qk));

    // Get Output Values
    Eigen::VectorXd xkp1 = kf.getState();
    Eigen::MatrixXd Pkp1 = kf.getCovariance();

    // Define Expected Solutions
    Eigen::VectorXd xkSol(4);
    xkSol << 15.823273, 114.281682, 73.827408, 85.319142;
    Eigen::MatrixXd PkSol(4, 4);
    PkSol << 67.2737,   10.1129,   32.9657,    0.0000,
             10.1129,  425.4600,  115.8266,  261.2763,
             32.9657,  115.8266,  461.4417,  171.9821,
              0.0000,  261.2763,  171.9821,  319.5217;

    // Correct State
    EXPECT_TRUE(xkp1.isApprox(xkSol, 1e-6));

    // Correct Covariance
    EXPECT_TRUE(Pkp1.isApprox(PkSol, 1e-6));

}

// Filter Update: Incorrect Hk Dimensions
TEST(FilterUpdate, IncorrectHkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd Hk = Eigen::MatrixXd::Zero(2, 5);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    kf.filterInitialize(x0, P0);

    // H Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUpdate(zk, Hk, Rk));

    // Redefine H
    Hk = Eigen::MatrixXd::Zero(3, 4);

    // H Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUpdate(zk, Hk, Rk));

}

// Filter Update: Incorrect Rk Dimensions
TEST(FilterUpdate, IncorrectRkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd Hk = Eigen::MatrixXd::Zero(2, 4);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 3);

    // Set Filter State and Covariance
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(4,4);
    kf.filterInitialize(x0, P0);

    // Rk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUpdate(zk, Hk, Rk));

    // Redefine Rk
    Rk = Eigen::MatrixXd::Zero(3, 2);

    // Rk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUpdate(zk, Hk, Rk));

}

// Kalman Filter Update: Compute Kalman Filter Update
TEST(FilterUpdate, ComputeFilterUpdate)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Filter Values
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Filter Values
    kf.filterInitialize(x0, P0);

    // Set Measurement Values/Matrices
    Eigen::VectorXd zk(2);
    zk << 101.744, 104.384;
    Eigen::MatrixXd Hk(2, 4);
    Hk << 3.573, 8.278, 2.485, 0.583,
          1.573, 3.584, 7.284, 3.294;
    Eigen::MatrixXd Rk(2, 2);
    Rk << 4.029, 0.000,
          0.000, 6.294;

    // Successfully Performed Filter Update
    EXPECT_TRUE(kf.filterUpdate(zk, Hk, Rk));

    // Get Output Values
    Eigen::VectorXd xkp1 = kf.getState();
    Eigen::MatrixXd Pkp1 = kf.getCovariance();

    // Define Expected Solutions
    Eigen::VectorXd xkSol(4);
    xkSol << 3.515714, 8.575666, 5.106958, 9.396168;
    Eigen::MatrixXd PkSol(4, 4);
    PkSol <<  4.1183,  -1.7411,  -0.0761,   0.0733,
             -1.7411,   0.8487,  -0.1845,   0.2430,
             -0.0761,  -0.1845,   1.0691,  -1.8616,
              0.0733,   0.2430,  -1.8616,   3.8898;

    // Correct State
    EXPECT_TRUE(xkp1.isApprox(xkSol, 1e-6));

    // Correct Covariance
    EXPECT_TRUE(Pkp1.isApprox(PkSol, 1e-3));

}

// Get Filter Covariance
TEST(FilterGetters, GetCovariance)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Set State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Values
    kf.filterInitialize(x0, P0);

    // Get Output Values
    Eigen::MatrixXd POut = kf.getCovariance();

    // Correct Covariance
    EXPECT_TRUE(POut.isApprox(P0, 1e-6));

}

// Get Filter State
TEST(FilterGetters, GetState)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Set State and Covariance
    Eigen::VectorXd x0(4);
    x0 << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd P0(4, 4);
    P0 << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;

    // Set Values
    kf.filterInitialize(x0, P0);

    // Get Output Values
    Eigen::VectorXd xOut = kf.getState();

    // Correct State
    EXPECT_TRUE(xOut.isApprox(x0, 1e-6));

}