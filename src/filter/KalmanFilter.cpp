//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            Kalman Filter                                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a robust and flexible Kalman Filter design.      //   
//              The model currently supports a high-bandwidth INS navigation error dynamics         //
//              formulation as the process model a tightly coupled GPS measurement update. Future   //
//              improvements will be made to include additional process models and measurement      //
//              modalities.                                                                         //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "KalmanFilter.hpp"

// Linear/Extended Kalman Filter Prediction
bool KalmanFilter::filterPredict(Eigen::VectorXd &xk,
                                 Eigen::MatrixXd &Pk,
                                 Eigen::MatrixXd &Phik,
                                 Eigen::MatrixXd &Qk,
                                 Eigen::VectorXd &xkp1,
                                 Eigen::MatrixXd &Pkp1) {
    
    // Get Dimension of State Vector
    int numStates = xk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pk.rows() != numStates) || (Pk.cols() != numStates)) {
        // Add Logging
        return false;
    } else if ((Phik.rows() != numStates) || (Phik.cols() != numStates)) {
        // Add Logging
        return false;
    } else if ((Qk.rows() != numStates) || (Qk.cols() != numStates)) {
        // Add Logging
        return false;
    } else if (xkp1.size() != numStates) {
        // Add Logging
        return false;
    } else if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        // Add Logging
        return false;
    }

    // Predict State Vector
    xkp1 = Phik * xk;

    // Predict Covariance
    Pkp1 = (Phik * xk * Phik.transpose()) + Qk;

    // Return Statement for Successful Prediction
    return true;

}


// Linear/Extended Kalman Filter Measurement Update
bool KalmanFilter::filterUpdate(Eigen::VectorXd &xk,
                                Eigen::MatrixXd &Pk,
                                Eigen::VectorXd &zk,
                                Eigen::MatrixXd &H,
                                Eigen::MatrixXd &Rk,
                                Eigen::MatrixXd &K,
                                Eigen::VectorXd &xkp1,
                                Eigen::MatrixXd &Pkp1) {
    
    // Get Dimension of State and Measurement Vector
    int numStates = xk.size();
    int numMeasDimensions = zk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pk.rows() != numStates) || (Pk.cols() != numStates)) {
        // Add Logging
        return false;
    } else if ((H.rows() != numMeasDimensions) || (H.cols() != numStates)) {
        // Add Logging
        return false;
    } else if ((Rk.rows() != numMeasDimensions) || (Rk.cols() != numMeasDimensions)) {
        // Add Logging
        return false;
    } else if ((K.rows() != numStates) || (K.cols() != numMeasDimensions)) {
        // Add Logging
        return false;
    } else if (xkp1.size() != numStates) {
        // Add Logging
        return false;
    } else if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        // Add Logging
        return false;
    }

    // Get Measurement Residual
    Eigen::VectorXd nu = zk - (H * xk);

    // Update State
    xkp1 = xk + (K * nu);

    // Update Covariance - Use Joseph Form for Stability
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(numStates, numStates) - (K * H);
    Pkp1 = (IKH * Pk * IKH.transpose()) + (K * Rk * K.transpose());

    // Return Statement for Successful Measurement Updated
    return true;
    
}


// Unscented Kalman Filter Prediction
bool KalmanFilter::filterUkfPredict(Eigen::VectorXd &Wi,
                                    Eigen::MatrixXd &yi,
                                    Eigen::VectorXd &xkp1,
                                    Eigen::MatrixXd &Pkp1) {
    
    // Get Dimension of State Vector
    int numStates = xkp1.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        // Add Logging
        return false;
    } else if (Wi.size() != (2*numStates + 1)) {
        // Add Logging
        return false;
    } else if ((yi.rows() != numStates) || (yi.cols() != (2*numStates + 1))) {
        // Add Logging
        return false;
    }

    // Predict State Vector
    

    // Predict Covariance
    

    // Return Statement for Successful Prediction
    return true;

}
