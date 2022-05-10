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
    
    // Get Dimension of State and Measurement Vector
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
    
    // Get Dimension of State and Measurement Vector
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

// Nonlinear Unweighted Batch Least Squares
bool BatchLeastSquares::UnweightedNonlinearLeastSquares(Eigen::VectorXd &yk,
                                                        Eigen::MatrixXd &Hk,
                                                        Eigen::VectorXd &xk,
                                                        double &eps,
                                                        Eigen::VectorXd &xkp1,
                                                        double &J) {
    
    // Get Dimension of State and Measurement Vector
    int numStates = xk.size();
    int measBatchSize = yk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Hk.rows() != measBatchSize) || (Hk.cols() != numStates)) {
        // Add Logging
        return false;
    } else if (xkp1.size() != numStates) {
        // Add Logging
        return false;
    } 

    // Initialize Predicted Measurment Vector
    Eigen::VectorXd yx;

    // Define Residual between Previous and Current State
    Eigen::VectorXd res = xkp1 - xk;

    // Loop Recursively until Convergence Criteria is Met
    while (res.norm() > eps) {

        // Compute Predicted Nonlinear Measurement
        if (!ComputePredictedNonlinearMeasurement(xk, yx)) {
            // add logging
            return false;
        }

        // Iterate State Estimate
        xkp1 = xk + ((Hk.transpose() * Hk).inverse() * Hk.transpose() * (yk - yx));

        // Compute New Residual
        res = xkp1 - xk;

        // Update Previous Estimate
        xk = xkp1;

    }

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xkp1, J)) {
        // Add Logging
        return false;
    }

    // Return Statement for Successful Prediction
    return true;

}

// Nonlinear Weighted Batch Least Squares
bool BatchLeastSquares::WeightedNonlinearLeastSquares(Eigen::VectorXd &yk,
                                                      Eigen::MatrixXd &Hk,
                                                      Eigen::VectorXd &xk,
                                                      Eigen::MatrixXd &Wk,
                                                      double &eps,
                                                      Eigen::VectorXd &xkp1,
                                                      double &J) {
    
    // Get Dimension of State and Measurement Vector
    int numStates = xk.size();
    int measBatchSize = yk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Hk.rows() != measBatchSize) || (Hk.cols() != numStates)) {
        // Add Logging
        return false;
    } else if (xkp1.size() != numStates) {
        // Add Logging
        return false;
    } 

    // Initialize Predicted Measurment Vector
    Eigen::VectorXd yx;

    // Define Residual between Previous and Current State
    Eigen::VectorXd res = xkp1 - xk;

    // Loop Recursively until Convergence Criteria is Met
    while (res.norm() > eps) {

        // Compute Predicted Nonlinear Measurement
        if (!ComputePredictedNonlinearMeasurement(xk, yx)) {
            // add logging
            return false;
        }

        // Iterate State Estimate
        xkp1 = xk + ((Hk.transpose() * Wk * Hk).inverse() * Hk.transpose() * Wk * (yk - yx));

        // Compute New Residual
        res = xkp1 - xk;

        // Update Previous Estimate
        xk = xkp1;

    }

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xkp1, J)) {
        // Add Logging
        return false;
    }

    // Return Statement for Successful Prediction
    return true;

}

// Utility Function: Compute Cost Value Estimate
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

// 
bool BatchLeastSquares::ComputePredictedNonlinearMeasurement(Eigen::VectorXd &xk,
                                                             Eigen::VectorXd &yx) {

    // Implement your nonlinear dynamics model of form yx = h(xk) here!

    // Return Statement
    return true;

}
