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