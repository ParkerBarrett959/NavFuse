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
#include <string>

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

// Get Accelerometer Errors: Incorrect Misalignment Dimensions
TEST(GetErrors, AccelerometerErrors)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d baEst(3);
    Eigen::Vector3d sfaEst(3);
    Eigen::VectorXd maEst(3);

    // Misalignment Vector has Incorrect Number of Columns
    ASSERT_FALSE(comp.getAccelerometerErrors(baEst, maEst, sfaEst));

}

// Get Accelerometer Errors: Get Errors Values
TEST(GetErrors, AccelerometerErrorValues)
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
    Eigen::Vector3d ba(3);
    Eigen::Vector3d sfa(3);
    Eigen::VectorXd ma(6);

    // Successfully Set Accelerometer Errors 
    if (!comp.setAccelerometerErrors(baEst, maEst, sfaEst)) {
        return;
    }

    // Successfully Performed Accelerometer Error Getter 
    EXPECT_TRUE(comp.getAccelerometerErrors(ba, ma, sfa));

    // Check Results
    EXPECT_EQ(baEst, ba);
    EXPECT_EQ(sfaEst, sfa);
    EXPECT_EQ(maEst, ma);

}

// Get Gyroscope Errors: Incorrect Misalignment Dimensions
TEST(GetErrors, GyroscopeErrors)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d bgEst(3);
    Eigen::Vector3d sfgEst(3);
    Eigen::VectorXd mgEst(3);

    // Misalignment Vector has Incorrect Number of Columns
    ASSERT_FALSE(comp.getGyroscopeErrors(bgEst, mgEst, sfgEst));

}

// Get Gyroscope Errors: Get Errors Values
TEST(GetErrors, GyroscopeErrorValues)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d bgEst(3);
    bgEst << 7.273, 8.272, 2.294;
    Eigen::Vector3d sfgEst(3);
    sfgEst << 0.273, 2.18, 7.284;
    Eigen::VectorXd mgEst(6);
    mgEst << 1.473, 8.183, 1.103, 7.183, 9.182, 3.183;
    Eigen::Vector3d bg(3);
    Eigen::Vector3d sfg(3);
    Eigen::VectorXd mg(6);

    // Successfully Set Gyroscope Errors 
    if (!comp.setGyroscopeErrors(bgEst, mgEst, sfgEst)) {
        return;
    }

    // Successfully Performed Gyroscope Error Getter 
    EXPECT_TRUE(comp.getGyroscopeErrors(bg, mg, sfg));

    // Check Results
    EXPECT_EQ(bgEst, bg);
    EXPECT_EQ(sfgEst, sfg);
    EXPECT_EQ(mgEst, mg);

}

// Compensate Accelerometer
TEST(CompensateAccelerometer, CompensateAccelerometer)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d dV(3);
    dV << 7.1837, 9.3832, 6.2840;
    Eigen::Vector3d baEst(3);
    baEst << 0.00004, 00003, 0.00007;
    Eigen::Vector3d sfaEst(3);
    sfaEst << 1.00031, 1.00046, 1.00035;
    Eigen::VectorXd maEst(6);
    maEst << 0.00001, 0.00002, 0.00003, 0.00004, 0.00005, 0.00006;

    // Successfully Set Accelerometer Errors 
    if (!comp.setAccelerometerErrors(baEst, maEst, sfaEst)) {
        return;
    }

    // Successfully Performed Accelerometer Measurement Compensation 
    EXPECT_TRUE(comp.compensateAccelerometer(dV));

    // Define Expected Solutions
    Eigen::Vector3d dVSol(3);
    dVSol << 7.186077, 6.386603, 6.286872;

    // Check Results
    EXPECT_TRUE(dV.isApprox(dVSol, 1e-6));

}

// Compensate Gyroscope
TEST(CompensateGyroscope, CompensateGyroscope)
{

    // Create Compensator Object
    Compensator comp;

    // Initialize Variables
    Eigen::Vector3d dTh(3);
    dTh << 7.1837, 9.3832, 6.2840;
    Eigen::Vector3d bgEst(3);
    bgEst << 0.00004, 00003, 0.00007;
    Eigen::Vector3d sfgEst(3);
    sfgEst << 1.00031, 1.00046, 1.00035;
    Eigen::VectorXd mgEst(6);
    mgEst << 0.00001, 0.00002, 0.00003, 0.00004, 0.00005, 0.00006;

    // Successfully Set Gyroscope Errors 
    if (!comp.setGyroscopeErrors(bgEst, mgEst, sfgEst)) {
        return;
    }

    // Successfully Performed Gyroscope Measurement Compensation 
    EXPECT_TRUE(comp.compensateGyroscope(dTh));

    // Define Expected Solutions
    Eigen::Vector3d dThSol(3);
    dThSol << 7.186077, 6.386603, 6.286872;

    // Check Results
    EXPECT_TRUE(dTh.isApprox(dThSol, 1e-6));

}