//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Unscented Kalman Filter                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a robust and flexible Unscented Kalman Filter    //   
//              design.                                                                             //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "UnscentedKalmanFilter.hpp"

// Filter Initialization
bool UnscentedKalmanFilter::filterInitialize(Eigen::VectorXd x0,
                                             Eigen::MatrixXd P0) {

    // Get Number of States
    int numStates = x0.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((P0.rows() != numStates) || (P0.cols() != numStates)) {
        std::cout << "[UnscentedKalmanFilter::filterInitialization] P0 has incorrect dimensions: Expected " << 
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

// Unscented Kalman Filter Prediction
bool UnscentedKalmanFilter::filterUkfPredict(Eigen::VectorXd Wi,
                                             Eigen::MatrixXd yi,
                                             Eigen::MatrixXd Qk) {
    
    // Get Dimension of State Vector
    int numStates = filterState_.size();
    
    // Verify Vectors/Matrices have Correct Dimensions
    if (Wi.size() != (2*numStates + 1)) {
        std::cout << "[UnscentedKalmanFilter::filterUkfPredict] Wi has incorrect dimensions: Expected " << 
                2*numStates + 1 << "x" << "1" << ", Got " << Wi.size() << "x" << "1" << std::endl;
        return false;
    } else if ((yi.rows() != numStates) || (yi.cols() != (2*numStates + 1))) {
        std::cout << "[UnscentedKalmanFilter::filterUkfPredict] yi has incorrect dimensions: Expected " << 
                numStates << "x" << 2*numStates+1 << ", Got " << yi.rows() << "x" << yi.cols() << std::endl;
        return false;
    } else if ((Qk.rows() != numStates) || (Qk.cols() != numStates)) {
        std::cout << "[UnscentedKalmanFilter::filterUkfPredict] Qk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Qk.rows() << "x" << Qk.cols() << std::endl;
        return false;
    }
    
    // Predict State Vector
    filterState_ = Eigen::VectorXd::Zero(numStates, 1);
    for (int i = 0; i < (2*numStates + 1); i++) {
        filterState_ += Wi(i) * yi.col(i);
    }

    // Predict Covariance
    filterCovariance_ = Qk;
    for (int i = 0; i < (2*numStates + 1); i++) {
        filterCovariance_ += Wi(i) * (yi.col(i) - filterState_) * (yi.col(i) - filterState_).transpose();
    }    

    // Return Statement for Successful Prediction
    return true;

}

// Unscented Kalman Filter Update
bool UnscentedKalmanFilter::filterUkfUpdate(Eigen::MatrixXd yi,
                                            Eigen::VectorXd Wi,
                                            Eigen::MatrixXd zi,
                                            Eigen::VectorXd zk,
                                            Eigen::MatrixXd Rk) {
    
    // Get Dimension of State and Measurement Vector
    int numStates = filterState_.size();
    int numMeasDimensions = zk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((yi.rows() != numStates) || (yi.cols() != (2*numStates + 1))) {
        std::cout << "[UnscentedKalmanFilter::filterUkfUpdate] yi has incorrect dimensions: Expected " << 
                numStates << "x" << 2*numStates+1 << ", Got " << yi.rows() << "x" << yi.cols() << std::endl;
        return false;
    } else if (Wi.size() != (2*numStates + 1)) {
        std::cout << "[UnscentedKalmanFilter::filterUkfUpdate] Wi has incorrect dimensions: Expected " << 
                2*numStates+1 << "x" << "1" << ", Got " << Wi.size() << "x" << "1" << std::endl;
        return false;
    } else if ((zi.rows() != numMeasDimensions) || (zi.cols() != (2*numStates + 1))) {
        std::cout << "[UnscentedKalmanFilter::filterUkfUpdate] zi has incorrect dimensions: Expected " << 
                numMeasDimensions << "x" << 2*numStates+1 << ", Got " << zi.rows() << "x" << zi.cols() << std::endl;
        return false;
    } else if ((Rk.rows() != numMeasDimensions) || (Rk.cols() != numMeasDimensions)) {
        std::cout << "[UnscentedKalmanFilter::filterUkfUpdate] Rk has incorrect dimensions: Expected " << 
                numMeasDimensions << "x" << numMeasDimensions << ", Got " << Rk.rows() << "x" << Rk.cols() << std::endl;
        return false;
    }
    
    // Compute Measurement Mean
    Eigen::VectorXd zhat = Eigen::VectorXd::Zero(numMeasDimensions);
    for (int i = 0; i < (2*numStates + 1); i++) {
        zhat += Wi(i) * zi.col(i);
    }   
    
    // Compute Measurement Covariance
    Eigen::MatrixXd S = Rk;
    for (int i = 0; i < (2*numStates + 1); i++) {
        S += Wi(i) * (zi.col(i) - zhat) * (zi.col(i) - zhat).transpose();
    }  
    
    // Compute Cross-Correlation Between Measurement and State Space
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(numStates, numMeasDimensions);
    for (int i = 0; i < (2*numStates + 1); i++) {
        T += Wi(i) * (yi.col(i) - filterState_) * (zi.col(i) - zhat).transpose();
    }  
    
    // Compute Kalman Gain
    Eigen::MatrixXd K = T * S.inverse();
    
    // Get Measurement Residual
    Eigen::VectorXd nu = zk - zhat;
    
    // Update State
    filterState_ = filterState_ + (K * nu);
    
    // Update Covariance
    filterCovariance_ = filterCovariance_ - (K * S * K.transpose());

    // Return Statement for Successful Update
    return true;

}

// Get Filter Covariance
Eigen::MatrixXd UnscentedKalmanFilter::getCovariance() {
    return filterCovariance_;
}

// Get Filter State
Eigen::MatrixXd UnscentedKalmanFilter::getState() {
    return filterState_;
}