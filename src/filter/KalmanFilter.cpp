//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            Kalman Filter                                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a robust and flexible Kalman Filter design.      //   
//              This class currently supports linear and extended Kalman filters.                   //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "KalmanFilter.hpp"

// Kalman Filter Prediction
bool KalmanFilter::filterPredict(Eigen::MatrixXd Phik,
                                 Eigen::MatrixXd Qk) {
    
    // Get Dimension of State Vector
    int numStates = filterState_.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Phik.rows() != numStates) || (Phik.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterPredict] Phik has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Phik.rows() << "x" << Phik.cols() << std::endl;
        return false;
    } else if ((Qk.rows() != numStates) || (Qk.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterPredict] Qk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Qk.rows() << "x" << Qk.cols() << std::endl;
        return false;
    }

    // Predict State Vector
    filterState_ = Phik * filterState_;

    // Predict Covariance
    filterCovariance_ = (Phik * filterCovariance_ * Phik.transpose()) + Qk;

    // Return Statement for Successful Prediction
    return true;

}


// Kalman Filter Measurement Update
bool KalmanFilter::filterUpdate(Eigen::VectorXd zk,
                                Eigen::MatrixXd Hk,
                                Eigen::MatrixXd Rk) {
    
    // Get Dimension of State and Measurement Vector
    int numStates = filterState_.size();
    int numMeasDimensions = zk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Hk.rows() != numMeasDimensions) || (Hk.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUpdate] Hk has incorrect dimensions: Expected " << 
                numMeasDimensions << "x" << numStates << ", Got " << Hk.rows() << "x" << Hk.cols() << std::endl;
        return false;
    } else if ((Rk.rows() != numMeasDimensions) || (Rk.cols() != numMeasDimensions)) {
        std::cout << "[KalmanFilter::filterUpdate] Rk has incorrect dimensions: Expected " << 
                numMeasDimensions << "x" << numMeasDimensions << ", Got " << Rk.rows() << "x" << Rk.cols() << std::endl;
        return false;
    }

    // Get Measurement Residual
    Eigen::VectorXd nu = zk - (Hk * filterState_);

    // Compute Kalman Gain
    Eigen::MatrixXd K = filterCovariance_ * Hk.transpose() * (Hk * filterCovariance_ * Hk.transpose() + Rk).inverse();

    // Update State
    filterState_ = filterState_ + (K * nu);

    // Update Covariance - Use Joseph Form for Stability
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(numStates, numStates) - (K * Hk);
    filterCovariance_ = (IKH * filterCovariance_ * IKH.transpose()) + (K * Rk * K.transpose());

    // Return Statement for Successful Measurement Updated
    return true;
    
}