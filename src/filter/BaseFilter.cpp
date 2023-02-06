//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          Base Kalman Filter                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a robust and flexible Base Filter design. This   //   
//              class currently supports derived linear, extended and Unscented Kalman filters.     //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "BaseFilter.hpp"

// Filter Initialization
bool BaseFilter::filterInitialize(Eigen::VectorXd x0,
                                  Eigen::MatrixXd P0) {

    // Get Number of States
    int numStates = x0.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((P0.rows() != numStates) || (P0.cols() != numStates)) {
        std::cout << "[BaseFilter::filterInitialization] P0 has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << P0.rows() << "x" << P0.cols() << std::endl;
        return false;
    }

    // Resize Filter Class Variables
    filterState_.resize(numStates);
    filterCovariance_.resize(numStates, numStates);

    // Initialize Filter Class Variables
    filterState_ = x0;
    filterCovariance_ = P0;

    // Successful Return
    return true;

}