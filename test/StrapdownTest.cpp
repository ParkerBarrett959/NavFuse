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
TEST(Initialization, Initialize)
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