//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Compensator Unit Testing                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Compensator Class Unit Tests.                                                       //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "compensator.hpp"
#include <Eigen/Dense>

// Set Accelerometer Errors: Incorrect Misalignment Dimensions
TEST(SetErrors, AccelerometerErrors)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d baEst = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d sfaEst = Eigen::Vector3d::Zero(3);
    Eigen::VectorXd maEst = Eigen::VectorXd::Zero(3);

    // Misalignment Vector has Incorrect Number of Columns
    ASSERT_FALSE(comp.setAccelerometerErrors(baEst, maEst, sfaEst));

}

// Set Gyro Errors: Incorrect Misalignment Dimensions
TEST(SetErrors, GyroErrors)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d bgEst = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d sfgEst = Eigen::Vector3d::Zero(3);
    Eigen::VectorXd mgEst = Eigen::VectorXd::Zero(3);

    // Misalignment Vector has Incorrect Number of Columns
    ASSERT_FALSE(comp.setGyroscopeErrors(bgEst, mgEst, sfgEst));

}



/*
// Set Accelerometer Errors: Set Errors Values
TEST(FilterPredict, ComputeFilterPrediction)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d baEst(3);
    baEst << 7.273, 8.272, 2.294;
    Eigen::Vector3d sfaEst(3);
    sfaEst << 0.273, 2.18, 7.284;
    Eigen::VectorXd maEst(6);
    maEst << 1.473, 8.183, 1.103, 7.183, 9.182, 3.183;

    // Successfully Set Accelerometer Errors 
    EXPECT_TRUE(comp.setAccelerometerErrors(baEst, maEst, sfaEst));

    // Set Results
    Eigen::Vector3d biasResult = comp.ba_;

    // Correct Values
    EXPECT_EQ(biasResult, baEst);

}
*/