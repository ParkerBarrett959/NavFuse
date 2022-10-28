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
#include <iostream>
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
        std::cout << "[BatchLeastSquares::UnweightedLinearLeastSquares] Hk has incorrect dimensions: Expected " << 
                measBatchSize << "x" << numStates << ", Got " << Hk.rows() << "x" << Hk.cols() << std::endl;
        return false;
    } else if (Hk.cols() > Hk.rows()) {
        std::cout << "[BatchLeastSquares::UnweightedLinearLeastSquares] Hk is underdetermined (more columns than rows) " << std::endl;
        return false;
    }

    // Compute State Estimate
    xk = (Hk.transpose() * Hk).ldlt().solve(Hk.transpose() * yk); 

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xk, J)) {
        std::cout << "[BatchLeastSquares::UnweightedLinearLeastSquares] Failed to compute cost function" << std::endl;
        return false;
    }

    // Return Statement for Successful Least Squares Solve
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
        std::cout << "[BatchLeastSquares::WeightedLinearLeastSquares] Hk has incorrect dimensions: Expected " << 
                measBatchSize << "x" << numStates << ", Got " << Hk.rows() << "x" << Hk.cols() << std::endl;
        return false;
    } else if ((Wk.rows() != measBatchSize) || (Wk.cols() != measBatchSize)) {
        std::cout << "[BatchLeastSquares::WeightedLinearLeastSquares] Wk has incorrect dimensions: Expected " << 
                measBatchSize << "x" << measBatchSize << ", Got " << Wk.rows() << "x" << Wk.cols() << std::endl;
        return false;
    } else if (Hk.cols() > Hk.rows()) {
        std::cout << "[BatchLeastSquares::WeightedLinearLeastSquares] Hk is underdetermined (more columns than rows) " << std::endl;
        return false;
    }

    // Compute State Estimate
    xk = (Hk.transpose() * Wk * Hk).ldlt().solve(Hk.transpose() * Wk * yk);

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xk, J)) {
        std::cout << "[BatchLeastSquares::WeightedLinearLeastSquares] Failed to compute cost function" << std::endl;
        return false;
    }

    // Return Statement for Successful Least Squares Solve
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
        std::cout << "[BatchLeastSquares::UnweightedNonlinearLeastSquares] Hk has incorrect dimensions: Expected " << 
                measBatchSize << "x" << numStates << ", Got " << Hk.rows() << "x" << Hk.cols() << std::endl;
        return false;
    } else if (xkp1.size() != numStates) {
        std::cout << "[BatchLeastSquares::UnweightedNonlinearLeastSquares] xkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << "1" << ", Got " << xkp1.size() << "x" << "1" << std::endl;
        return false;
    } else if (Hk.cols() > Hk.rows()) {
        std::cout << "[BatchLeastSquares::UnweightedNonlinearLeastSquares] Hk is underdetermined (more columns than rows) " << std::endl;
        return false;
    }

    // Initialize Predicted Measurment Vector
    Eigen::VectorXd yx;

    // Define Residual between Previous and Current State
    Eigen::VectorXd res = xkp1 - xk;

    // Loop Recursively until Convergence Criteria is Met
    while (res.norm() > eps) {

        // Compute Predicted Nonlinear Measurement - REPLACE EXAMPLE CODE IN CLASS FUNCTION WITH YOUR OWN DYNAMICS
        if (!ComputePredictedNonlinearMeasurement(xk, yx)) {
            std::cout << "[BatchLeastSquares::UnweightedNonlinearLeastSquares] Unable to compute predicted measurement" << std::endl;
            return false;
        }

	    // Re-Linearize H - REPLACE EXAMPLE CODE IN CLASS FUNCTION WITH YOUR OWN DYNAMICS
	    if (!RelinearizeH(xk, Hk)) {
            std::cout << "[BatchLeastSquares::UnweightedNonlinearLeastSquares] Unable to re-linearize H" << std::endl;
	        return false;
	    }

        // Iterate State Estimate
        xkp1 = xk + (Hk.transpose() * Hk).ldlt().solve(Hk.transpose() * (yk - yx));

        // Compute New Residual
        res = xkp1 - xk;

        // Update Previous Estimate
        xk = xkp1;

    }

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xkp1, J)) {
        std::cout << "[BatchLeastSquares::UnweightedNonlinearLeastSquares] Failed to compute cost function" << std::endl;
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
        std::cout << "[BatchLeastSquares::WeightedNonlinearLeastSquares] Hk has incorrect dimensions: Expected " << 
                measBatchSize << "x" << numStates << ", Got " << Hk.rows() << "x" << Hk.cols() << std::endl;
        return false;
    } else if (xkp1.size() != numStates) {
        std::cout << "[BatchLeastSquares::WeightedNonlinearLeastSquares] xkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << "1" << ", Got " << xkp1.size() << "x" << "1" << std::endl;
        return false;
    } else if (Hk.cols() > Hk.rows()) {
        std::cout << "[BatchLeastSquares::WeightedNonlinearLeastSquares] Hk is underdetermined (more columns than rows) " << std::endl;
        return false;
    } else if ((Wk.rows() != measBatchSize) || (Wk.cols() != measBatchSize)) {
        std::cout << "[BatchLeastSquares::WeightedNonlinearLeastSquares] Wk has incorrect dimensions: Expected " << 
                measBatchSize << "x" << measBatchSize << ", Got " << Wk.rows() << "x" << Wk.cols() << std::endl;
        return false;
    }

    // Initialize Predicted Measurment Vector
    Eigen::VectorXd yx;

    // Define Residual between Previous and Current State
    Eigen::VectorXd res = xkp1 - xk;

    // Loop Recursively until Convergence Criteria is Met
    while (res.norm() > eps) {

        // Compute Predicted Nonlinear Measurement - REPLACE EXAMPLE CODE IN CLASS FUNCTION WITH YOUR OWN DYNAMICS
        if (!ComputePredictedNonlinearMeasurement(xk, yx)) {
            std::cout << "[BatchLeastSquares::WeightedNonlinearLeastSquares] Unable to compute predicted measurement" << std::endl;
            return false;
        }

        // Re-Linearize H - REPLACE EXAMPLE CODE IN CLASS FUNCTION WITH YOUR OWN DYNAMICS
	    if (!RelinearizeH(xk, Hk)) {
            std::cout << "[BatchLeastSquares: WeightedNonlinearLeastSquares] Unable to re-linearize H" << std::endl;
	        return false;
	    }

        // Iterate State Estimate
        xkp1 = xk + (Hk.transpose() * Wk * Hk).ldlt().solve(Hk.transpose() * Wk * (yk - yx));

        // Compute New Residual
        res = xkp1 - xk;

        // Update Previous Estimate
        xk = xkp1;

    }

    // Compute Cost Function
    if (!ComputeCostFunction(yk, Hk, xkp1, J)) {
        std::cout << "[BatchLeastSquares::WeightedNonlinearLeastSquares] Failed to compute cost function" << std::endl;
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
        std::cout << "[BatchLeastSquares::ComputeCostFunction] Hk has incorrect dimensions: Expected " << 
                measBatchSize << "x" << numStates << ", Got " << Hk.rows() << "x" << Hk.cols() << std::endl;
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

// Utility Function: Compute Predicted Nonlinear Measurement - REPLACE WITH YOUR SYSTEM DYNAMICS
bool BatchLeastSquares::ComputePredictedNonlinearMeasurement(Eigen::VectorXd &xk,
                                                             Eigen::VectorXd &yx) {

    // Implement your nonlinear dynamics model of form yx = h(xk) here!


    // Return Statement
    return true;

}

// Utility Function: Relinearize H - REPLACE WITH YOUR SYSTEM DYNAMICS
bool BatchLeastSquares::RelinearizeH(Eigen::VectorXd &xk,
                                     Eigen::MatrixXd &Hk) {

    // Implement your own dynamics model here!


    // Return Statement
    return true;

}
