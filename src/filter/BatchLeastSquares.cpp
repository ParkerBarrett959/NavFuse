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

// Linear Unweighted Batch Least Squares
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

    // Compute State Estimate
    xk = (Hk.transpose() * Hk).inverse() * Hk.transpose() * yk; 

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xk, J)) {
        // Add Logging
        return false;
    }

    // Return Statement for Successful Prediction
    return true;

}

// Linear Weighted Batch Least Squares
bool BatchLeastSquares::WeightedLinearLeastSquares(Eigen::VectorXd &yk,
                                                   Eigen::MatrixXd &Hk,
                                                   Eigen::MatrixXd &Wk,
                                                   Eigen::VectorXd &xk,
                                                   double &J) {
    
    // Get Dimension of State Vector
    int numStates = xk.size();
    int measBatchSize = yk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Hk.rows() != measBatchSize) || (Hk.cols() != numStates)) {
        // Add Logging
        return false;
    } else if ((Wk.rows() != measBatchSize) || (Wk.cols() != measBatchSize)) {
        //Add Logging
        return false;
    }

    // Compute State Estimate
    xk = (Hk.transpose() * Wk * Hk).inverse() * Hk.transpose() * Wk * yk; 

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xk, J)) {
        // Add Logging
        return false;
    }

    // Return Statement for Successful Prediction
    return true;

}

// Linear Weighted Batch Least Squares
bool BatchLeastSquares::ComputeCostFunction(Eigen::VectorXd &yk,
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

    // Return Statement for Successful Prediction
    return true;

}


