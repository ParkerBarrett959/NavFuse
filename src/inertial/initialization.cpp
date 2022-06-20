//////////////////////////////////////////////////////////////////////////////////////////////////////
//                         Inertial Navigation Initialization and Alignment                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines the inertial navigation system alignment and     //
//              initialization algorithms.                                                          //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "initialization.hpp"

// Coarse Attitude Initialization
bool Initialization::coarseAttitudeAlignment(double &lat,
                                             double &wE,
                                             double &g,
                                             Eigen::Vector3d &aB,
                                             Eigen::Vector3d &wIB_B,
                                             Eigen::Matrix3d &RB2N) {

    // Define Useful Quantities
    double slat = std::sin(lat);
    double clat = std::cos(lat);

    // Define A Matrix
    Eigen::Matrix3d A;
    A << 0.0,        0.0,         -g, 
         wE*clat,    0.0,         -wE*slat, 
         0.0,        g*wE*clat,    0.0;

    // Define B Matrix
    Eigen::Matrix3d B = Eigen::Matrix3d::Zero();
    B.row(0) = aB.transpose();
    B.row(1) = wIB_B.transpose();
    B.row(2) = (-aB.cross(wIB_B)).transpose();

    // Compute Rotation from Body to NED Navigation Frame
    RB2N = A.inverse() * B;

    // Return Statement for Successful Coarse Alignment
    return true;

}

// Fine Alignment Prediction Step
bool Initialization::fineAlignmentPredict(double &lat,
                                          double &wE,
                                          double &g,
                                          double &r,
                                          Eigen::VectorXd &xk,
                                          Eigen::MatrixXd &Pk,
                                          KalmanFilter &KF,
                                          Eigen::VectorXd &xkp1,
                                          Eigen::MatrixXd &Pkp1) {
     
    // Get Dimension of State Vector
    int numStates = xk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pk.rows() != numStates) || (Pk.cols() != numStates)) {
        std::cout << "[Initialization::fineAlignmentPredict] Pk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pk.rows() << "x" << Pk.cols() << std::endl;
        return false;
    } else if (xkp1.size() != numStates) {
        std::cout << "[Initialization::fineAlignmentPredict] xkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << "1" << ", Got " << xkp1.size() << "x" << "1" << std::endl;
        return false;
    } else if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        std::cout << "[Initialization::fineAlignmentPredict] Pkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pkp1.rows() << "x" << Pkp1.cols() << std::endl;
        return false;
    }

    // Define Useful Quantities
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double tlat = std::tan(lat);
     
    // Build State Transition Matrix
    Eigen::MatrixXd Phik;
    Phik << 0.0,     -wE*slat,  0.0,    -1.0,  0.0,  0.0,  0.0,          1.0/r,
            wE*slat,  0.0,      wE*clat, 0.0, -1.0,  0.0, -1.0/r,        0.0,
            0.0,     -wE*clat,  0.0,     0.0,  0.0, -1.0,  0.0,          tlat/r,
            0.0,      0.0,      0.0,     0.0,  0.0,  0.0,  0.0,          0.0,
            0.0,      0.0,      0.0,     0.0,  0.0,  0.0,  0.0,          0.0,
            0.0,      0.0,      0.0,     0.0,  0.0,  0.0,  0.0,          0.0,
            0.0,      g,        0.0,     0.0,  0.0,  0.0,  0.0,         -2.0*wE*slat,
            -g,       0.0,      0.0,     0.0,  0.0,  0.0,  2.0*wE*slat,  0.0;
     
     // Define Process Noise Matrix
     Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(numStates, numStates);

     // Filter Prediction
     if (!KF.filterPredict(xk, Pk, Phik, Qk, xkp1, Pkp1)) {
          std::cout << "[Initialization::fineAlignmentPredict] Unable to perform filter prediction" << std::endl;
          return false;
     }

    // Return Statement for Successful Fine Alignment Prediction
    return true;

}

// Fine Alignment Measurement Update
bool Initialization::fineAlignmentUpdate(bool &velValid,
                                         bool &azValid,
                                         Eigen::Vector2d velMeas,
                                         double &azMeas,
                                         double &dtVel,
                                         double &dtAz,
                                         double &azEst,
                                         double &sigVel,
                                         double &sigAz,
                                         Eigen::VectorXd &xk,
                                         Eigen::MatrixXd &Pk,
                                         KalmanFilter &KF,
                                         Eigen::VectorXd &xkp1,
                                         Eigen::MatrixXd &Pkp1) {

     // Get Dimension of State Vector
    int numStates = xk.size();

    // Verify Vectors/Matrices have Correct Dimensions
    if ((Pk.rows() != numStates) || (Pk.cols() != numStates)) {
        std::cout << "[Initialization::fineAlignmentUpdate] Pk has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pk.rows() << "x" << Pk.cols() << std::endl;
        return false;
    } else if (xkp1.size() != numStates) {
        std::cout << "[Initialization::fineAlignmentUpdate] xkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << "1" << ", Got " << xkp1.size() << "x" << "1" << std::endl;
        return false;
    } else if ((Pkp1.rows() != numStates) || (Pkp1.cols() != numStates)) {
        std::cout << "[Initialization::fineAlignmentUpdate] Pkp1 has incorrect dimensions: Expected " << 
                numStates << "x" << numStates << ", Got " << Pkp1.rows() << "x" << Pkp1.cols() << std::endl;
        return false;
    }

    // Compute Azimuth Measurement Residual - Velocity Estimated as Stationary
    double azRes = azEst - azMeas;

    // Compute Sigma Squared Values
    double sVSq = sigVel * sigVel;
    double sASq = sigAz * sigAz;

    // Assemble Measurement Jacobian and Vector
    Eigen::VectorXd z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    if (velValid && azValid) {
        z << velMeas[0], velMeas[1], azRes;
        H << 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,
             0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,
             0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0;
        R << sVSq*dtVel,     0.0,         0.0,
                0.0,      sVSq*dtVel,     0.0,
                0.0,         0.0,      sASq*dtAz;   
    } else if (velValid) {
        z << velMeas[0], velMeas[1];
        H << 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,
             0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,
             0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0;
        R << sVSq*dtVel,     0.0,         0.0,
                0.0,      sVSq*dtVel,     0.0,
                0.0,         0.0,         0.0;
    } else if (azValid) {
        z << azRes;
        H << 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
             0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
             0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0;
        R << 0.0,     0.0,      0.0,
             0.0,     0.0,      0.0,
             0.0,     0.0,    sASq*dtAz;
    } else {
        std::cout << "[Initialization::fineAlignmentUpdate] Unable to assemble measurement and Jacobian" << std::endl;
        return false;
    }

    // Perform Measurement Update
    if (!KF.filterUpdate(xk, Pk, z, H, R, xkp1, Pkp1)) {
        std::cout << "[Initialization::fineAlignmentUpdate] Unable to perform filter update" << std::endl;
        return false;
    }

    // Return Statement for Successful Fine Alignment Update
    return true;
               
}
