//////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Inertial Measurement Unit Sensor Header                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial measuremet unit sensor class                               //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// IMU Sensor Class
class ImuSensor {

    // Public Class Members/Functions
    public:

        /* @strapdownIntegrate
            Inputs:
                dV: 3x1 dimensional vector of delta velocity measurements 
                dTh: 3x1 dimensional vector of delta theta measurments
                tov: measurement time of validity
            Outputs:
            Description:
                Function which takes in compensated IMU measuremnts and performs strapdown integration to
                compute the new position, velocity and attitude of the platform. The outputs are stored as 
                class variables and are not explicitly listed as outputs in this function.
        */
        bool strapdownIntegrate(Eigen::VectorXd &xk,
                           Eigen::MatrixXd &Pk,
                           Eigen::MatrixXd &Phik,
                           Eigen::MatrixXd &Qk,
                           Eigen::VectorXd &xkp1,
                           Eigen::MatrixXd &Pkp1);

        /* @compensateImu
            Inputs:
                Insert here
            Outputs:
            Description:
                Insert here
        */
        bool compensateImu(Eigen::VectorXd &xk,
                           Eigen::MatrixXd &Pk,
                           Eigen::MatrixXd &Phik,
                           Eigen::MatrixXd &Qk,
                           Eigen::VectorXd &xkp1,
                           Eigen::MatrixXd &Pkp1); 

    // Private Class Members/Function
    private:


};

