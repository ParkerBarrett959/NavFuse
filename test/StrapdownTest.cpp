//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Strapdown Unit Testing                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Strapdown Class Unit Tests.                                                         //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "strapdown.hpp"
#include <Eigen/Dense>
#include <string>

// Strapdown Initialization
TEST(Strapdown, Initialize)
{

    // Create Strapdown Object
    Strapdown sd;

    // Initialize Variables
    Eigen::Vector3d lla = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d vNed = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d rph = Eigen::VectorXd::Zero(3);
    int64_t tov = 1664271553028564;

    // Successful Initialization
    ASSERT_TRUE(sd.initialize(lla, vNed, rph, tov));

}

// Strapdown Integration
TEST(Strapdown, Integrate)
{

    // Create Strapdown Object
    Strapdown sd;

    // Initialize Variables
    Eigen::Vector3d llaPrev(3);
    llaPrev << 0.739, -1.240, 1.456;
    Eigen::Vector3d vNedPrev(3);
    vNedPrev << 4.278, 8.364, 0.564;
    Eigen::Vector3d rphPrev = Eigen::VectorXd::Zero(3);
    int64_t tovPrev = 1664271553028564 - 1e6;
   
    // Successful Initialization
    ASSERT_TRUE(sd.initialize(llaPrev, vNedPrev, rphPrev, tovPrev));
   
    // Initialize Variables
    Eigen::Vector3d dV(3);
    dV << 0.002, 0.003, -9.798;
    Eigen::Vector3d dTh(3);
    dTh << 0.002, 0.374, 0.046;
    int64_t tov = 1664271553028564;
   
    // Successful Integration
    ASSERT_TRUE(sd.integrate(dV, dTh, tov));

    // Set Expected Outputs
    Eigen::Vector3d llaSol(3);
    llaSol << 0.739000, -1.239998, 0.551308;
    Eigen::Vector3d vNedSol(3);
    vNedSol << 0.700110, 8.303278, 1.245385;
    Eigen::Matrix3d RB2NSol(3, 3);
    RB2NSol << 0.9298401,  -0.0445495,  0.3652570,
               0.0452886,   0.9989524,  0.0065477,
              -0.3651661,   0.0104537,  0.9308837;

    // Check Results
    EXPECT_TRUE(sd.lla_.isApprox(llaSol, 1e-6));
    EXPECT_TRUE(sd.vNed_.isApprox(vNedSol, 1e-6));
    EXPECT_TRUE(sd.RB2N_.isApprox(RB2NSol, 1e-6));

}