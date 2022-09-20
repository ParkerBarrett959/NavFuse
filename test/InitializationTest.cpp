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
;

    // Check Results
    EXPECT_TRUE(RB2N.isApprox(RB2NSol, 1e-6));

}