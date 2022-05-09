//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Batch Least Squares                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a variety of least squares estimators. The       //
//              linear unweighted batch least squares, linear weighted batch least squares, and     //
//              nonlinear batch least squares algorithms are included. Each algorithm computes      //
//              the state estimate, as well as the cost function value estimate.                    //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "BatchLeastSquares.hpp"

// Linear/Extended Kalman Filter Prediction
bool BatchLeastSquares::UnweightedLinearLeastSquares(Eigen::VectorXd &yk,
                                                     Eigen::MatrixXd &Hk,
                                                     Eigen::VectorXd &xk,
                                                     double &J) {
    
    // Get Dimension of State Vector
    int numStates = xk.size();
    int measBatchSize = yk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Hk.rows() != measBatchSize) || (Hk.cols() != numStates)) {
        // Add Logging
        return false;
    }

    // Compute Error Vector
    Eigen::VectorXd e = yk - (Hk * xk);

    // Compute Cost Function
    J = 0;
    J = 0.5 * e.dot(e);

    // Compute State Estimate
    xk = (Hk.transpose() * Hk).inverse() * Hk.transpose() * yk; 

    // Return Statement for Successful Prediction
    return true;

}
