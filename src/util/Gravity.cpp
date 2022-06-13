//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Gravity Functions                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of gravity models of varying fidelity.     //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "Gravity.hpp"

// Compute DCM from Quaternion
bool Gravity::simpleGravity(Eigen::VectorXd &rA,
                            Eigen::VectorXd &gA) {

    // Verify Correct Dimensions
    if (rA.size() != 3) {
        // Add Logging
        return false;
    } else if (gA.size() != 3) {
        // Add Logging
        return false;
    }

    // Compute Magnitude of Position Vector
    double rSq = rA.squaredNorm();

    // Compute Gravity Vector
    if (rSq > 0) {
        gA = ((-G * mE) / rSq) * rA.normalized();
    } else {
        // Add Logging
        return false;
    }

    // Return Statement for Successful Initialization
    return true;

}