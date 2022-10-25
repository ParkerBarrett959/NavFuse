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

// Compute RJ2E: Invalid Date Vector Size
TEST(ComputeRJ2E, InvalidDateVectorSize)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 0, 0, 0, 0, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RJ2E(3,3);

    // Incorrect Date Vector Size
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

}

// Compute RJ2E: EOP File Does Not Exist

// Compute RJ2E: Invalid Month
TEST(ComputeRJ2E, InvalidMonth)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 13, 0, 0, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RJ2E(3,3);

    // Month Value to High
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

    // Redefine Date
    dateVec = {2022, -1, 0, 0, 0, 0};

    // Month Value to Low
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

}

// Compute RJ2E: Invalid Day
TEST(ComputeRJ2E, InvalidDay)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 32, 0, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RJ2E(3,3);

    // Day Value to High
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

    // Redefine Date
    dateVec = {2022, 12, -1, 0, 0, 0};

    // Day Value to Low
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

}

// Compute RJ2E: Invalid Hour
TEST(ComputeRJ2E, InvalidHour)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 30, 24, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RJ2E(3,3);

    // Hour Value to High
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

    // Redefine Date
    dateVec = {2022, 12, 30, -1, 0, 0};

    // Hour Value to Low
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

}

// Compute RJ2E: Invalid Minute
TEST(ComputeRJ2E, InvalidMinute)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 30, 23, 60, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RJ2E(3,3);

    // Minute Value to High
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

    // Redefine Date
    dateVec = {2022, 12, 30, 23, -1, 0};

    // Minute Value to Low
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

}

// Compute RJ2E: Invalid Second
TEST(ComputeRJ2E, InvalidSecond)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 30, 23, 59, 60};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RJ2E(3,3);

    // Second Value to High
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

    // Redefine Date
    dateVec = {2022, 12, 30, 23, 59, -1};

    // Second Value to Low
    EXPECT_FALSE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

}

// Compute RJ2E: Compute Result
TEST(ComputeRJ2E, ComputeResult)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2020.0, 10.0, 2.0, 23.0, 41.0, 24.0};
    std::string eopFile = "../data/EOP-Last5Years.csv";
    Eigen::Matrix3d RJ2E(3,3);

    // Load EOPs
    EXPECT_TRUE(rot.getEops(eopFile));

    // Compute Result
    EXPECT_TRUE(rot.computeRJ2k2Ecef(dateVec, eopFile, RJ2E));

    // Define Expected Result
    Eigen::Matrix3d RJ2ESol(3, 3);
    RJ2ESol << 0.9920445,   0.1258720,  -0.0019661,
              -0.1258717,   0.9920465,   0.0002442,
               0.0019812,   5.244e-06,   0.9999980;
    
    // Check Results
    EXPECT_TRUE(RJ2E.isApprox(RJ2ESol, 1e-6));

}

// Compute RE2J: Invalid Date Vector Size
TEST(ComputeRE2J, InvalidDateVectorSize)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 0, 0, 0, 0, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RE2J(3,3);

    // Incorrect Date Vector Size
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

}

// Compute RE2J: EOP File Does Not Exist

// Compute RE2J: Invalid Month
TEST(ComputeRE2J, InvalidMonth)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 13, 0, 0, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RE2J(3,3);

    // Month Value to High
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

    // Redefine Date
    dateVec = {2022, -1, 0, 0, 0, 0};

    // Month Value to Low
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

}

// Compute RE2J: Invalid Day
TEST(ComputeRE2J, InvalidDay)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 32, 0, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RE2J(3,3);

    // Day Value to High
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

    // Redefine Date
    dateVec = {2022, 12, -1, 0, 0, 0};

    // Day Value to Low
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

}

// Compute RE2J: Invalid Hour
TEST(ComputeRE2J, InvalidHour)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 30, 24, 0, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RE2J(3,3);

    // Hour Value to High
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

    // Redefine Date
    dateVec = {2022, 12, 30, -1, 0, 0};

    // Hour Value to Low
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

}

// Compute RE2J: Invalid Minute
TEST(ComputeRE2J, InvalidMinute)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 30, 23, 60, 0};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RE2J(3,3);

    // Minute Value to High
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

    // Redefine Date
    dateVec = {2022, 12, 30, 23, -1, 0};

    // Minute Value to Low
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

}

// Compute RE2J: Invalid Second
TEST(ComputeRE2J, InvalidSecond)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2022, 12, 30, 23, 59, 60};
    std::string eopFile = "EOP-Last5Years.csv";
    Eigen::Matrix3d RE2J(3,3);

    // Second Value to High
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

    // Redefine Date
    dateVec = {2022, 12, 30, 23, 59, -1};

    // Second Value to Low
    EXPECT_FALSE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

}

// Compute RE2J: Compute Result
TEST(ComputeRE2J, ComputeResult)
{

    // Create Rotations Object
    Rotations rot;

    // Initialize Variables
    std::vector<double> dateVec{2020.0, 10.0, 2.0, 23.0, 41.0, 24.0};
    std::string eopFile = "../data/EOP-Last5Years.csv";
    Eigen::Matrix3d RE2J(3,3);

    // Load EOPs
    EXPECT_TRUE(rot.getEops(eopFile));

    // Compute Result
    EXPECT_TRUE(rot.computeREcef2J2k(dateVec, eopFile, RE2J));

    // Define Expected Result
    Eigen::Matrix3d RE2JSol(3, 3);
    RE2JSol << 0.9920445,  -0.1258717,   0.0019812,
               0.1258720,   0.9920465,   5.244e-06,
              -0.0019661,   0.0002442,   0.9999980;
    
    // Check Results
    EXPECT_TRUE(RE2J.isApprox(RE2JSol, 1e-6));

}