//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            Kalman Filter                                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a robust and flexible Kalman Filter design.      //   
//              The model currently supports linear, extended and unscented Kalman Filter           //
//              variations.                                                                         //           
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
        std::cout << "[KalmanFilter::filterPredict] Pk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pk.rows() << "x" << Pk.cols() << std::endl;
        return false;
    } else if ((Phik.rows() != numStates) || (Phik.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterPredict] Phik has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Phik.rows() << "x" << Phik.cols() << std::endl;
        return false;
    } else if ((Qk.rows() != numStates) || (Qk.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterPredict] Qk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Qk.rows() << "x" << Qk.cols() << std::endl;
        return false;
    } else if (xkp1.size() != numStates) {
        std::cout << "[KalmanFilter::filterPredict] xkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << "1" << ", Got " << xkp1.size() << "x" << "1" << std::endl;
        return false;
    } else if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterPredict] Pkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pkp1.rows() << "x" << Pkp1.cols() << std::endl;
        return false;
    }

    // Predict State Vector
    xkp1 = Phik * xk;

    // Predict Covariance
    Pkp1 = (Phik * Pk * Phik.transpose()) + Qk;

    // Return Statement for Successful Prediction
    return true;

}


// Linear/Extended Kalman Filter Measurement Update
bool KalmanFilter::filterUpdate(Eigen::VectorXd &xk,
                                Eigen::MatrixXd &Pk,
                                Eigen::VectorXd &zk,
                                Eigen::MatrixXd &H,
                                Eigen::MatrixXd &Rk,
                                Eigen::VectorXd &xkp1,
                                Eigen::MatrixXd &Pkp1) {
    
    // Get Dimension of State and Measurement Vector
    int numStates = xk.size();
    int numMeasDimensions = zk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pk.rows() != numStates) || (Pk.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUpdate] Pk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pk.rows() << "x" << Pk.cols() << std::endl;
        return false;
    } else if ((H.rows() != numMeasDimensions) || (H.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUpdate] H has incorrect dimensions: Expected " << 
                numMeasDimensions << "x" << numStates << ", Got " << H.rows() << "x" << H.cols() << std::endl;
        return false;
    } else if ((Rk.rows() != numMeasDimensions) || (Rk.cols() != numMeasDimensions)) {
        std::cout << "[KalmanFilter::filterUpdate] Rk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Rk.rows() << "x" << Rk.cols() << std::endl;
        return false;
    } else if (xkp1.size() != numStates) {
        std::cout << "[KalmanFilter::filterUpdate] xkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << "1" << ", Got " << xkp1.size() << "x" << "1" << std::endl;
        return false;
    } else if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUpdate] Pkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pkp1.rows() << "x" << Pkp1.cols() << std::endl;
        return false;
    }

    // Get Measurement Residual
    Eigen::VectorXd nu = zk - (H * xk);

    // Compute Kalman Gain
    Eigen::MatrixXd K = Pk * H.transpose() * (H * Pk * H.transpose() + Rk).inverse();

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
                                    Eigen::MatrixXd &Qk,
                                    Eigen::VectorXd &xkp1,
                                    Eigen::MatrixXd &Pkp1) {
    
    // Get Dimension of State Vector
    int numStates = xkp1.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUkfPredict] Pkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pkp1.rows() << "x" << Pkp1.cols() << std::endl;
        return false;
    } else if (Wi.size() != (2*numStates + 1)) {
        std::cout << "[KalmanFilter::filterUkfPredict] Wi has incorrect dimensions: Expected " << 
                2*numStates + 1 << "x" << "1" << ", Got " << Wi.size() << "x" << "1" << std::endl;
        return false;
    } else if ((yi.rows() != numStates) || (yi.cols() != (2*numStates + 1))) {
        std::cout << "[KalmanFilter::filterUkfPredict] yi has incorrect dimensions: Expected " << 
                numStates << "x" << 2*numStates+1 << ", Got " << yi.rows() << "x" << yi.cols() << std::endl;
        return false;
    } else if ((Qk.rows() != numStates) || (Qk.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUkfPredict] Qk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Qk.rows() << "x" << Qk.cols() << std::endl;
        return false;
    }
    
    // Predict State Vector
    xkp1 = Eigen::VectorXd::Zero(numStates, 1);
    for (int i = 0; i < (2*numStates + 1); i++) {
        xkp1 += Wi(i) * yi.col(i);
    }

    // Predict Covariance
    Pkp1 = Qk;
    for (int i = 0; i < (2*numStates + 1); i++) {
        Pkp1 += Wi(i) * (yi.col(i) - xkp1) * (yi.col(i) - xkp1).transpose();
    }    

    // Return Statement for Successful Prediction
    return true;

}


// Unscented Kalman Filter Update
bool KalmanFilter::filterUkfUpdate(Eigen::VectorXd &xk,
                                   Eigen::MatrixXd &Pk,
                                   Eigen::MatrixXd &yi,
                                   Eigen::VectorXd &Wi,
                                   Eigen::MatrixXd &zi,
                                   Eigen::VectorXd &zk,
                                   Eigen::MatrixXd &Rk,
                                   Eigen::VectorXd &xkp1,
                                   Eigen::MatrixXd &Pkp1) {
    
    // Get Dimension of State and Measurement Vector
    int numStates = xk.size();
    int numMeasDimensions = zk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pk.rows() != numStates) || (Pk.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUkfUpdate] Pk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pk.rows() << "x" << Pk.cols() << std::endl;
        return false;
    } else if ((yi.rows() != numStates) || (yi.cols() != (2*numStates + 1))) {
        std::cout << "[KalmanFilter::filterUkfUpdate] yi has incorrect dimensions: Expected " << 
                numStates << "x" << 2*numStates+1 << ", Got " << yi.rows() << "x" << yi.cols() << std::endl;
        return false;
    } else if (Wi.size() != (2*numStates + 1)) {
        std::cout << "[KalmanFilter::filterUkfUpdate] Wi has incorrect dimensions: Expected " << 
                2*numStates+1 << "x" << "1" << ", Got " << Wi.size() << "x" << "1" << std::endl;
        return false;
    } else if ((zi.rows() != numMeasDimensions) || (zi.cols() != (2*numStates + 1))) {
        std::cout << "[KalmanFilter::filterUkfUpdate] zi has incorrect dimensions: Expected " << 
                numMeasDimensions << "x" << 2*numStates+1 << ", Got " << zi.rows() << "x" << zi.cols() << std::endl;
        return false;
    } else if ((Rk.rows() != numMeasDimensions) || (Rk.cols() != numMeasDimensions)) {
        std::cout << "[KalmanFilter::filterUkfUpdate] Rk has incorrect dimensions: Expected " << 
                numMeasDimensions << "x" << numMeasDimensions << ", Got " << Rk.rows() << "x" << Rk.cols() << std::endl;
        return false;
    } else if (xkp1.size() != numStates) {
        std::cout << "[KalmanFilter::filterUkfUpdate] xkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << "1" << ", Got " << xkp1.size() << "x" << "1" << std::endl;
        return false;
    } else if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        std::cout << "[KalmanFilter::filterUkfUpdate] Pkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pkp1.rows() << "x" << Pkp1.cols() << std::endl;
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
        T += Wi(i) * (yi.col(i) - xk) * (zi.col(i) - zhat).transpose();
    }  
    
    // Compute Kalman Gain
    Eigen::MatrixXd K = T * S.inverse();
    if ((K.rows() != numStates) || (K.cols() != numMeasDimensions)) {
        std::cout << "[KalmanFilter::filterUkfUpdate] K has incorrect dimensions: Expected " << 
                numStates << "x" << numMeasDimensions << ", Got " << K.rows() << "x" << K.cols() << std::endl;
        return false;
    }
    
    // Get Measurement Residual
    Eigen::VectorXd nu = zk - zhat;
    
    // Update State
    xkp1 = xk + (K * nu);
    
    // Update Covariance
    Pkp1 = Pk - (K * S * K.transpose());

    // Return Statement for Successful Update
    return true;

}