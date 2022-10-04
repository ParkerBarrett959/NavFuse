//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    Navigation Utility Functions                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of helpful navigation utility functions.   //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "NavUtils.hpp"

// Skew Symmetric Matrix Function
bool NavUtils::skewSymmetric(Eigen::Vector3d &vec,
                             Eigen::Matrix3d &vecX) {

    // Get Elements of Vector
    double v1 = vec[0];
    double v2 = vec[1];
    double v3 = vec[2];

    // Set Elements of Skew Symmetric
    vecX << 0.0, -v3, v2,
            v3,  0.0, -v1,
           -v2,  v1,  0.0;

    // Return Statement
    return true;

}

// Strapdown 4th Order Runge Kutta Position/Velocity Integration
bool NavUtils::strapdownRk4(Eigen::VectorXd &ykm1,
                            double &dt,
                            Eigen::Vector3d &dVN,
                            Eigen::Vector3d &gN,
                            Eigen::VectorXd &yk) {

    // Verify Correct Dimensions
    if (ykm1.size() != 6) {
        std::cout << "[NavUtils::strapdownRk4] ykm1 has incorrect dimensions: Expected " << 
                "6x1, Got " << ykm1.size() << "x1" << std::endl;
        return false;
    } else if (yk.size() != 6) {
        std::cout << "[NavUtils::strapdownRk4] yk has incorrect dimensions: Expected " << 
                "6x1, Got " << yk.size() << "x1" << std::endl;
        return false;
    }
    
    // Define K1
    Eigen::VectorXd y(6);
    Eigen::VectorXd f1(6);
    Eigen::VectorXd K1(6);
    y = ykm1;
    if (!strapdownDynamics(y, f1, dVN, gN, dt)) {
        std::cout << "[NavUtils::strapdownRk4] Unable to evaluate strapdown dynamics" << std::endl;
        return false;
    }
    K1 = dt * f1;
    
    // Define K2
    Eigen::VectorXd f2(6);
    Eigen::VectorXd K2(6);
    y = ykm1 + (K1 / 2.0);
    if (!strapdownDynamics(y, f2, dVN, gN, dt)) {
        std::cout << "[NavUtils::strapdownRk4] Unable to evaluate strapdown dynamics" << std::endl;
        return false;
    }
    K2 = dt * f2;
    
    // Define K3
    Eigen::VectorXd f3(6);
    Eigen::VectorXd K3(6);
    y = ykm1 + (K2 / 2.0);
    if (!strapdownDynamics(y, f3, dVN, gN, dt)) {
        std::cout << "[NavUtils::strapdownRk4] Unable to evaluate strapdown dynamics" << std::endl;
        return false;
    }
    K3 = dt * f3;
    
    // Define K4
    Eigen::VectorXd f4(6);
    Eigen::VectorXd K4(6);
    y = ykm1 + K3;
    if (!strapdownDynamics(y, f4, dVN, gN, dt)) {
        std::cout << "[NavUtils::strapdownRk4] Unable to evaluate strapdown dynamics" << std::endl;
        return false;
    }
    K4 = dt * f4;
    
    // Compute Integration
    yk = ykm1 + ((1.0/6.0) * K1) + ((1.0/3.0) * K2) + ((1.0/3.0) * K3)  + ((1.0/6.0) * K4);

    // Return True for Successful Integration
    return true;

}

// Strapdown Dynamics Function
bool NavUtils::strapdownDynamics(Eigen::VectorXd &y,
                                 Eigen::VectorXd &f,
                                 Eigen::Vector3d &dVN,
                                 Eigen::Vector3d &gN,
                                 double &dt) {

    // Unpack Previous States
    double lat = y[0];
    double lon = y[1];
    double h = y[2];
    double vN = y[3];
    double vE = y[4];
    double vD = y[5];

    // Get Earth Constants and Parameters
    double wE = Gravity.gravityParams.wE;
    double fE = Gravity.gravityParams.f;
    double a = Gravity.gravityParams.a;

    // Compute Useful Quantities
    Eigen::Vector3d aN = dVN / dt;
    double eSq = (2.0 * fE) - std::pow(fE, 2); 
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double N = a / (std::sqrt(1 - (eSq * std::pow(slat, 2))));
    double M = (a * (1 - eSq)) / std::pow(1 - (eSq * std::pow(slat, 2)), 1.5);
    double latDot = vN / (M + h);
    double lonDot = vE / ((N + h) * clat);
    double hDot = -vD;

    // Compute Dynamics
    f << latDot,
         lonDot,
         hDot,
         aN[0] + gN[0] - (2 * wE * slat * vE) + (latDot * vD) - (lonDot * slat * vE),
         aN[1] + gN[1] + (2 * wE * slat * vN) + (2 * wE * clat * vD) + (lonDot * slat * vN) + (lonDot * clat * vD),
         aN[2] + gN[2] - (2 * wE * clat * vE) - (lonDot * clat * vE) - (latDot * vN);

    // Return Statement
    return true;

}