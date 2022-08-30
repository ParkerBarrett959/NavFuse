//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Gravity Utilities Unit Testing                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Gravity Utility Class Unit Tests.                                                   //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "Gravity.hpp"
#include <Eigen/Dense>

// Compute Simple Gravity
TEST(SimpleGravity, ComputeSimpleGravity)
{

    // Create Gravity Object
    Gravity grav;

    // Initialize Variables
    Eigen::Vector3d rA(3);
    rA << 3682340.016, 3682136.045, 3682726.345;
    Eigen::Vector3d gA(3);

    // Successfully Compute Gravity 
    EXPECT_TRUE(grav.simpleGravity(rA, gA));

    // Define Expected Solutions
    Eigen::Vector3d gASol(3);
    gASol << -5.656592, -5.656278, -5.657185;

    // Check Results
    EXPECT_TRUE(gA.isApprox(gASol, 1e-6));

}

// Compute Gravity NED
TEST(GravityNed, ComputeGravityNed)
{

    // Create Gravity Object
    Gravity grav;

    // Initialize Variables
    double lat = 0.7505;
    double h = 345.293;
    Eigen::Vector3d gN(3);

    // Successfully Compute Gravity 
    EXPECT_TRUE(grav.gravityNed(lat, h, gN));

    // Define Expected Solutions
    Eigen::Vector3d gNSol(3);
    gNSol << -0.000003, 0.000000, 9.80332;

    // Check Results
    EXPECT_TRUE(gN.isApprox(gNSol, 1e-6));

}