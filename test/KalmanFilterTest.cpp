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

// Filter Predict: Incorrect Phik Dimensions
TEST(FilterPredict, IncorrectPhikDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Phik = Eigen::MatrixXd::Zero(4, 3);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // Phik Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

    // Redefine Phik
    Phik = Eigen::MatrixXd::Zero(3, 4);

    // Pk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

}

// Filter Predict: Incorrect Qk Dimensions
TEST(FilterPredict, IncorrectQkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Phik = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(4, 3);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // Qk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

    // Redefine Qk
    Qk = Eigen::MatrixXd::Zero(3, 4);

    // Qk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

}

// Filter Predict: Incorrect xkp1 Dimensions
TEST(FilterPredict, Incorrectxkp1Dimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Phik = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(5);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // xkp1 Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

}

// Filter Predict: Incorrect Pkp1 Dimensions
TEST(FilterPredict, IncorrectPkp1Dimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Phik = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 3);

    // Pkp1 Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

    // Redefine Qk
    Pkp1 = Eigen::MatrixXd::Zero(3, 4);

    // Pkp1 Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

}

// Kalman Filter Predict: Compute Kalman Filter Prediction
TEST(FilterPredict, ComputeFilterPrediction)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk(4);
    xk << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd Pk(4, 4);
    Pk << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;
    Eigen::MatrixXd Phik(4, 4);
    Phik << 3.674, 0.375, 0.000, 0.000,
            0.000, 4.284, 0.000, 8.294,
            1.329, 4.295, 6.295, 0.000,
            0.000, 0.000, 3.294, 7.184;
    Eigen::MatrixXd Qk(4, 4);
    Qk << 3.284, 0.000, 0.000, 0.000,
          0.000, 8.284, 0.000, 0.000,
          0.000, 0.000, 8.394, 0.000,
          0.000, 0.000, 0.000, 3.219;
    Eigen::VectorXd xkp1(4);
    Eigen::MatrixXd Pkp1(4, 4);

    // Successfully Performed Filter Prediction 
    EXPECT_TRUE(kf.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1));

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

// Filter Update: Incorrect Pk Dimensions
TEST(FilterUpdate, IncorrectPkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 3);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // Pk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

    // Redefine Pk
    Pk = Eigen::MatrixXd::Zero(3, 4);

    // Pk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

}

// Filter Update: Incorrect H Dimensions
TEST(FilterUpdate, IncorrectHDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // H Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

    // Redefine H
    H = Eigen::MatrixXd::Zero(3, 4);

    // H Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

}

// Filter Update: Incorrect Rk Dimensions
TEST(FilterUpdate, IncorrectRkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 3);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // Rk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

    // Redefine Rk
    Rk = Eigen::MatrixXd::Zero(3, 2);

    // Rk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

}

// Filter Update: Incorrect xkp1 Dimensions
TEST(FilterUpdate, Incorrectxkp1Dimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(5);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 4);

    // xkp1 Vector has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

}

// Filter Update: Incorrect Pkp1 Dimensions
TEST(FilterUpdate, IncorrectPkp1Dimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd zk = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(2, 2);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(4, 3);

    // Pkp1 Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

    // Redefine Pkp1
    Pkp1 = Eigen::MatrixXd::Zero(3, 4);

    // Pkp1 Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

}

// Kalman Filter Update: Compute Kalman Filter Update
TEST(FilterUpdate, ComputeFilterUpdate)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd xk(4);
    xk << 3.452, 8.375, 5.285, 9.453;
    Eigen::MatrixXd Pk(4, 4);
    Pk << 4.675, 0.000, 0.000, 0.000,
          0.000, 6.295, 0.000, 0.000,
          0.000, 0.000, 8.294, 0.000,
          0.000, 0.000, 0.000, 4.385;
    Eigen::VectorXd zk(2);
    zk << 101.744, 104.384;
    Eigen::MatrixXd H(2, 4);
    H << 3.573, 8.278, 2.485, 0.583,
         1.573, 3.584, 7.284, 3.294;
    Eigen::MatrixXd Rk(2, 2);
    Rk << 4.029, 0.000,
          0.000, 6.294;
    Eigen::VectorXd xkp1(4);
    Eigen::MatrixXd Pkp1(4, 4);

    // Successfully Performed Filter Update
    EXPECT_TRUE(kf.filterUpdate(xk, Pk, zk, H, Rk, xkp1, Pkp1));

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

// Filter UKF Predict: Incorrect Pkp1 Dimensions
TEST(filterUkfPredict, IncorrectPkp1Dimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(3, 7);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(3, 4);

    // Pkp1 Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUkfPredict(Wi, yi, Qk, xkp1, Pkp1));

    // Redefine Pkp1
    Pkp1 = Eigen::MatrixXd::Zero(4, 3);

    // Pkp1 Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUkfPredict(Wi, yi, Qk, xkp1, Pkp1));

}

// Filter UKF Predict: Incorrect Wi Dimensions
TEST(filterUkfPredict, IncorrectWiDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(3, 7);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(3, 3);

    // Wi Vector has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUkfPredict(Wi, yi, Qk, xkp1, Pkp1));

}

// Filter UKF Predict: Incorrect yi Dimensions
TEST(filterUkfPredict, IncorrectyiDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(3, 3);

    // yi Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUkfPredict(Wi, yi, Qk, xkp1, Pkp1));

    // Redefine yi
    yi = Eigen::MatrixXd::Zero(4, 7);

    // yi Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUkfPredict(Wi, yi, Qk, xkp1, Pkp1));

}

// Filter UKF Predict: Incorrect Qk Dimensions
TEST(filterUkfPredict, IncorrectQkDimensions)
{

    // Create Kalman Filter Object
    KalmanFilter kf;

    // Initialize Variables
    Eigen::VectorXd Wi = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd yi = Eigen::MatrixXd::Zero(3, 7);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(3, 4);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(3, 3);

    // Qk Matrix has Incorrect Number of Columns
    ASSERT_FALSE(kf.filterUkfPredict(Wi, yi, Qk, xkp1, Pkp1));

    // Redefine Qk
    Qk = Eigen::MatrixXd::Zero(4, 3);

    // Qk Matrix has Incorrect Number of Rows
    ASSERT_FALSE(kf.filterUkfPredict(Wi, yi, Qk, xkp1, Pkp1));

}