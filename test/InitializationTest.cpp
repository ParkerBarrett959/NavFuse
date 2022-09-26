//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                Strapdown Initialization Unit Testing                             //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Strapdown initialization (alignment and calibration) unit testing.                  //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "initialization.hpp"
#include <Eigen/Dense>

// Compute Coarse Alignment
TEST(CoarseAttitudeAlignment, ComputeResult)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double wE = 7.292115 * 1e-5;
    double lat = 0.7361;
    double g = -9.798;
    Eigen::Vector3d aB(3);
    aB << 0.00001, 0.00002, 0.00003;
    Eigen::Vector3d wIB_B(3);
    wIB_B << 0.0002, 0.0092, 0.0034;
    Eigen::Matrix3d RB2N(3, 3);

    // Successfully Compute Coarse Initialization
    EXPECT_TRUE(init.coarseAttitudeAlignment(lat, wE, g, aB, wIB_B, RB2N));

    // Define Expected Solutions
    Eigen::Matrix3d RB2NSol(3, 3);
    RB2NSol <<     3.70087216369588,         170.240078845804,         62.914813837855,
              -0.000392825687754338,   -0.0000528803810438532,    0.000166195483280682,
               0.000001020616452337,    0.0000020412329046744,      0.0000030618493570;

    // Check Results
    EXPECT_TRUE(RB2N.isApprox(RB2NSol, 1e-6));

}

// Fine Alignment Predict: Incorrect Pk Dimensions 
TEST(FineAlignmentPredict, IncorrectPkDimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double wE = 7.292115 * 1e-5;
    double lat = 0.7361;
    double g = -9.798;
    double r = 6378000.0;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 7);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Failed Fine Alignment Prediction: Incorrect Pk columns
    EXPECT_FALSE(init.fineAlignmentPredict(lat, wE, g, r, xk, Pk, xkp1, Pkp1));

    // Redefine Pk
    Pk = Eigen::MatrixXd::Zero(7, 8);

    // Failed Fine Alignment Prediction: Incorrect Pk rows
    EXPECT_FALSE(init.fineAlignmentPredict(lat, wE, g, r, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Predict: Incorrect xkp1 Dimensions 
TEST(FineAlignmentPredict, Incorrectxkp1Dimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double wE = 7.292115 * 1e-5;
    double lat = 0.7361;
    double g = -9.798;
    double r = 6378000.0;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 8);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Failed Fine Alignment Prediction: Incorrect xkp1 columns
    EXPECT_FALSE(init.fineAlignmentPredict(lat, wE, g, r, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Predict: Incorrect Pkp1 Dimensions 
TEST(FineAlignmentPredict, IncorrectPkp1Dimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double wE = 7.292115 * 1e-5;
    double lat = 0.7361;
    double g = -9.798;
    double r = 6378000.0;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 8);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 7);

    // Failed Fine Alignment Prediction: Incorrect Pkp1 columns
    EXPECT_FALSE(init.fineAlignmentPredict(lat, wE, g, r, xk, Pk, xkp1, Pkp1));

    // Redefine Pkp1
    Pkp1 = Eigen::MatrixXd::Zero(7, 8);

    // Failed Fine Alignment Prediction: Incorrect Pkp1 rows
    EXPECT_FALSE(init.fineAlignmentPredict(lat, wE, g, r, xk, Pk, xkp1, Pkp1));

}

// Compute Fine Alignment Prediction
TEST(FineAlignmentPredict, ComputeResult)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double wE = 7.292115 * 1e-5;
    double lat = 0.7361;
    double g = -9.798;
    double r = 6378256;
    Eigen::VectorXd xk(8);
    xk << 3.452, 8.375, 5.285, 9.453, 3.582, 0.374, 6.283, 0.378;
    Eigen::MatrixXd Pk(8, 8);
    Pk << 4.675,     0,     0,     0,     0,     0,     0,     0,
              0, 6.295,     0,     0,     0,     0,     0,     0,
              0,     0, 8.294,     0,     0,     0,     0,     0,
              0,     0,     0, 4.385,     0,     0,     0,     0,
              0,     0,     0,     0, 7.284,     0,     0,     0,
              0,     0,     0,     0,     0, 9.733,     0,     0,
              0,     0,     0,     0,     0,     0, 0.372,     0,
              0,     0,     0,     0,     0,     0,     0, 4.283;
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Successfully Compute Coarse Initialization
    EXPECT_TRUE(init.fineAlignmentPredict(lat, wE, g, r, xk, Pk, xkp1, Pkp1));
   
    // Define Expected Solutions
    Eigen::VectorXd xkp1Sol(8);
    xkp1Sol << -9.4534099, -3.5815463, -0.3744525, 0, 0, 0, -82.0582870, 33.8233112;
    Eigen::MatrixXd Pkp1Sol(8, 8);
    Pkp1Sol << 4.385000,         0,1.6656e-08,     0,     0,     0,  0.003019,         0,
                      0,  7.284000,         0,     0,     0,     0,         0,  0.002242,
             1.6656e-08,         0,  9.733000,     0,     0,     0,  0.003333,         0,
                      0,         0,         0,     0,     0,     0,         0,         0,
                      0,         0,         0,     0,     0,     0,         0,         0,
                      0,         0,         0,     0,     0,     0,         0,         0,
               0.003019,         0,  0.003333,     0,     0,     0,604.325061,         0,
                      0,  0.002242,         0,     0,     0,     0,         0,448.803758;
    
    // Check Results
    EXPECT_TRUE(xkp1.isApprox(xkp1Sol, 1e-6));
    EXPECT_TRUE(Pkp1.isApprox(Pkp1Sol, 1e-6));

}

// Fine Alignment Azimuth Update: Incorrect Pk Dimensions 
TEST(FineAlignmentAzimuthUpdate, IncorrectPkDimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double azMeas = 0.368;
    double dtAz = 0.1;
    double azEst = 0.365;
    double sigAz = 0.001;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 7);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Failed Fine Alignment Azimuth Update: Incorrect Pk columns
    EXPECT_FALSE(init.fineAlignmentAzimuthUpdate(azMeas, dtAz, azEst, sigAz, xk, Pk, xkp1, Pkp1));

    // Redefine Pk
    Pk = Eigen::MatrixXd::Zero(7, 8);

    // Failed Fine Alignment Azimuth Update: Incorrect Pk rows
    EXPECT_FALSE(init.fineAlignmentAzimuthUpdate(azMeas, dtAz, azEst, sigAz, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Azimuth Update: Incorrect xkp1 Dimensions 
TEST(FineAlignmentAzimuthUpdate, Incorrectxkp1Dimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double azMeas = 0.368;
    double dtAz = 0.1;
    double azEst = 0.365;
    double sigAz = 0.001;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 8);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Failed Fine Alignment Azimuth Update: Incorrect xkp1 rows
    EXPECT_FALSE(init.fineAlignmentAzimuthUpdate(azMeas, dtAz, azEst, sigAz, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Azimuth Update: Incorrect Pkp1 Dimensions 
TEST(FineAlignmentAzimuthUpdate, IncorrectPkp1Dimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double azMeas = 0.368;
    double dtAz = 0.1;
    double azEst = 0.365;
    double sigAz = 0.001;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 8);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 7);

    // Failed Fine Alignment Azimuth Update: Incorrect Pkp1 columns
    EXPECT_FALSE(init.fineAlignmentAzimuthUpdate(azMeas, dtAz, azEst, sigAz, xk, Pk, xkp1, Pkp1));

    // Redefine Pkp1
    Pkp1 = Eigen::MatrixXd::Zero(7, 8);

    // Failed Fine Alignment Azimuth Update: Incorrect Pkp1 rows
    EXPECT_FALSE(init.fineAlignmentAzimuthUpdate(azMeas, dtAz, azEst, sigAz, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Azimuth Update: Compute Result 
TEST(FineAlignmentAzimuthUpdate, ComputeResult)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    double azMeas = 0.368;
    double dtAz = 0.1;
    double azEst = 0.365;
    double sigAz = 0.001;
    Eigen::VectorXd xk(8);
    xk << 3.452, 8.375, 5.285, 9.453, 3.582, 0.374, 6.283, 0.378;
    Eigen::MatrixXd Pk(8, 8);
    Pk << 4.675,     0,     0,     0,     0,     0,     0,     0,
              0, 6.295,     0,     0,     0,     0,     0,     0,
              0,     0, 8.294,     0,     0,     0,     0,     0,
              0,     0,     0, 4.385,     0,     0,     0,     0,
              0,     0,     0,     0, 7.284,     0,     0,     0,
              0,     0,     0,     0,     0, 9.733,     0,     0,
              0,     0,     0,     0,     0,     0, 0.372,     0,
              0,     0,     0,     0,     0,     0,     0, 4.283;
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Successful Fine Alignment Azimuth Update
    EXPECT_TRUE(init.fineAlignmentAzimuthUpdate(azMeas, dtAz, azEst, sigAz, xk, Pk, xkp1, Pkp1));
    
    // Define Expected Solutions
    Eigen::VectorXd xkp1Sol(8);
    xkp1Sol << 3.452, 8.375, 0.003, 9.453, 3.582, 0.374, 6.283, 0.378;
    Eigen::MatrixXd Pkp1Sol(8, 8);
    Pkp1Sol << 4.675,     0,     0,     0,     0,     0,     0,     0,
                   0, 6.295,     0,     0,     0,     0,     0,     0,
                   0,     0, 0.000,     0,     0,     0,     0,     0,
                   0,     0,     0, 4.385,     0,     0,     0,     0,
                   0,     0,     0,     0, 7.284,     0,     0,     0,
                   0,     0,     0,     0,     0, 9.733,     0,     0,
                   0,     0,     0,     0,     0,     0, 0.372,     0,
                   0,     0,     0,     0,     0,     0,     0, 4.283;
    
    // Check Results
    EXPECT_TRUE(xkp1.isApprox(xkp1Sol, 1e-6));
    EXPECT_TRUE(Pkp1.isApprox(Pkp1Sol, 1e-6));

}

// Fine Alignment Velocity Update: Incorrect Pk Dimensions 
TEST(FineAlignmentVelocityUpdate, IncorrectPkDimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    Eigen::Vector2d velMeas(2);
    velMeas << 0.0, 0.0;
    double dtVel = 0.1;
    double sigVel = 0.01;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 7);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Failed Fine Alignment Velocity Update: Incorrect Pk columns
    EXPECT_FALSE(init.fineAlignmentVelocityUpdate(velMeas, dtVel, sigVel, xk, Pk, xkp1, Pkp1));

    // Redefine Pk
    Pk = Eigen::MatrixXd::Zero(7, 8);

    // Failed Fine Alignment Velocity Update: Incorrect Pk rows
    EXPECT_FALSE(init.fineAlignmentVelocityUpdate(velMeas, dtVel, sigVel, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Velocity Update: Incorrect xkp1 Dimensions 
TEST(FineAlignmentVelocityUpdate, Incorrectxkp1Dimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    Eigen::Vector2d velMeas(2);
    velMeas << 0.0, 0.0;
    double dtVel = 0.1;
    double sigVel = 0.01;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 8);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Failed Fine Alignment Velocity Update: Incorrect xkp1 rows
    EXPECT_FALSE(init.fineAlignmentVelocityUpdate(velMeas, dtVel, sigVel, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Velocity Update: Incorrect Pkp1 Dimensions 
TEST(FineAlignmentVelocityUpdate, IncorrectPkp1Dimensions)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    Eigen::Vector2d velMeas(2);
    velMeas << 0.0, 0.0;
    double dtVel = 0.1;
    double sigVel = 0.01;
    Eigen::VectorXd xk = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(8, 8);
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 7);

    // Failed Fine Alignment Velocity Update: Incorrect Pkp1 columns
    EXPECT_FALSE(init.fineAlignmentVelocityUpdate(velMeas, dtVel, sigVel, xk, Pk, xkp1, Pkp1));

    // Redefine Pkp1
    Pkp1 = Eigen::MatrixXd::Zero(7, 8);

    // Failed Fine Alignment Velocity Update: Incorrect Pkp1 rows
    EXPECT_FALSE(init.fineAlignmentVelocityUpdate(velMeas, dtVel, sigVel, xk, Pk, xkp1, Pkp1));

}

// Fine Alignment Velocity Update: Compute Result 
TEST(FineAlignmentVelocityUpdate, ComputeResult)
{

    // Create Initialization Object
    Initialization init;

    // Initialize Variables
    Eigen::Vector2d velMeas(2);
    velMeas << 0.0, 0.0;
    double dtVel = 0.1;
    double sigVel = 0.01;
    Eigen::VectorXd xk(8);
    xk << 3.452, 8.375, 5.285, 9.453, 3.582, 0.374, 6.283, 0.378;
    Eigen::MatrixXd Pk(8, 8);
    Pk << 4.675,     0,     0,     0,     0,     0,     0,     0,
              0, 6.295,     0,     0,     0,     0,     0,     0,
              0,     0, 8.294,     0,     0,     0,     0,     0,
              0,     0,     0, 4.385,     0,     0,     0,     0,
              0,     0,     0,     0, 7.284,     0,     0,     0,
              0,     0,     0,     0,     0, 9.733,     0,     0,
              0,     0,     0,     0,     0,     0, 0.372,     0,
              0,     0,     0,     0,     0,     0,     0, 4.283;
    Eigen::VectorXd xkp1 = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd Pkp1 = Eigen::MatrixXd::Zero(8, 8);

    // Successful Fine Alignment Velocity Update
    EXPECT_TRUE(init.fineAlignmentVelocityUpdate(velMeas, dtVel, sigVel, xk, Pk, xkp1, Pkp1));
    
    // Define Expected Solutions
    Eigen::VectorXd xkp1Sol(8);
    xkp1Sol << 3.452, 8.375, 5.285, 9.453, 3.582, 0.374, 0.0001688, 0.0000000;
    Eigen::MatrixXd Pkp1Sol(8, 8);
    Pkp1Sol << 4.675,     0,     0,     0,     0,     0,     0,     0,
                   0, 6.295,     0,     0,     0,     0,     0,     0,
                   0,     0, 8.294,     0,     0,     0,     0,     0,
                   0,     0,     0, 4.385,     0,     0,     0,     0,
                   0,     0,     0,     0, 7.284,     0,     0,     0,
                   0,     0,     0,     0,     0, 9.733,     0,     0,
                   0,     0,     0,     0,     0,     0,9.99e-06,     0,
                   0,     0,     0,     0,     0,     0,     0, 9.99e-06;
    
    // Check Results
    EXPECT_TRUE(xkp1.isApprox(xkp1Sol, 1e-6));
    EXPECT_TRUE(Pkp1.isApprox(Pkp1Sol, 1e-6));

}