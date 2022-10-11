//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Rotations Unit Testing                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    NRotations Class Unit Tests.                                                        //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "Rotations.hpp"
#include <Eigen/Dense>

// Compute RE2N: Latitutde Out of Range
TEST(ComputeRE2N, LatOutOfRange)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 1.6;
    double lon = 0.0;
    Eigen::Matrix3d RE2N(3, 3);

    // Latitude out of Range
    EXPECT_FALSE(rot.computeREcef2Ned(lat, lon, RE2N));

}

// Compute RE2N: Longitude Out of Range
TEST(ComputeRE2N, LonOutOfRange)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 0.0;
    double lon = 3.5;
    Eigen::Matrix3d RE2N(3, 3);

    // Longitude out of Range
    EXPECT_FALSE(rot.computeREcef2Ned(lat, lon, RE2N));

}

// Compute RE2N
TEST(ComputeRE2N, ComputeResult)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 0.731;
    double lon = -1.240;
    Eigen::Matrix3d RE2N(3, 3);

    // Compute Result
    EXPECT_TRUE(rot.computeREcef2Ned(lat, lon, RE2N));

    // Define Expected Solutions
    Eigen::Matrix3d RE2NSol(3, 3);
    RE2NSol << -0.2168387,   0.6314191,   0.7445072,
                0.9457839,   0.3247963,           0,
               -0.2418132,   0.7041429,  -0.6676145;

    // Check Results
    EXPECT_TRUE(RE2N.isApprox(RE2NSol, 1e-6));

}

// Compute RN2E: Latitude Out of Range
TEST(ComputeRN2E, LatOutOfRange)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 1.6;
    double lon = 0.0;
    Eigen::Matrix3d RN2E(3, 3);

    // Latitude out of Range
    EXPECT_FALSE(rot.computeRNed2Ecef(lat, lon, RN2E));

}

// Compute RN2E: Longitude Out of Range
TEST(ComputeRN2E, LonOutOfRange)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 0.0;
    double lon = 3.5;
    Eigen::Matrix3d RN2E(3, 3);

    // Longitude out of Range
    EXPECT_FALSE(rot.computeRNed2Ecef(lat, lon, RN2E));

}

// Compute RN2E
TEST(ComputeRN2E, ComputeResult)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 0.731;
    double lon = -1.240;
    Eigen::Matrix3d RN2E(3, 3);

    // Compute Result
    EXPECT_TRUE(rot.computeRNed2Ecef(lat, lon, RN2E));

    // Define Expected Solutions
    Eigen::Matrix3d RN2ESol(3, 3);
    RN2ESol << -0.2168387,   0.9457839,   -0.2418132,
                0.6314191,   0.3247963,    0.7041429,
                0.7445072,           0,   -0.6676145;

    // Check Results
    EXPECT_TRUE(RN2E.isApprox(RN2ESol, 1e-6));

}

// Compute ECEF2LLA
TEST(ComputeEcef2Lla, ComputeResult)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    Eigen::Vector3d rE(3);
    rE << 1545471.693, -4488375.702, 4245603.836;
    double lat;
    double lon;
    double alt;

    // Compute Result
    EXPECT_TRUE(rot.ecef2Lla(rE, lat, lon, alt));

    // Define Expected Solutions
    Eigen::Vector3d lla(3);
    lla << lat, lon, alt;
    Eigen::Vector3d llaSol(3);
    llaSol << 0.733038, -1.239184, -0.000753;

    // Check Results
    EXPECT_TRUE(lla.isApprox(llaSol, 1e-4));

}

// Compute lla2Ecef: Latitude Out of Range
TEST(ComputeLla2Ecef, LatOutOfRange)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 1.6;
    double lon = 0.0;
    double alt = 0.0;
    Eigen::Vector3d rE(3);

    // Latitude out of Range
    EXPECT_FALSE(rot.lla2Ecef(lat, lon, alt, rE));

}

// Compute lla2Ecef: Longitude Out of Range
TEST(ComputeLla2Ecef, LonOutOfRange)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 0.0;
    double lon = 3.5;
    double alt = 0.0;
    Eigen::Vector3d rE(3);

    // Longitude out of Range
    EXPECT_FALSE(rot.lla2Ecef(lat, lon, alt, rE));

}

// Compute lla2Ecef: Compute Result
TEST(ComputeLla2Ecef, ComputeResult)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    double lat = 0.731;
    double lon = -1.24;
    double alt = 0.0;
    Eigen::Vector3d rE(3);

    // Compute Result
    EXPECT_TRUE(rot.lla2Ecef(lat, lon, alt, rE));

    // Define Expected Result
    Eigen::Vector3d rESol(3);
    rESol << 1544623.562624, -4497835.476247, 4235955.233997;

    // Check Results
    EXPECT_TRUE(rE.isApprox(rESol, 1e-6));

}