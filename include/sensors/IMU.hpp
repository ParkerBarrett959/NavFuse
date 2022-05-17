//////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Inertial Measurement Unit Sensor Header                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial measurement unit sensor class                              //           
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
        bool strapdownIntegrate(Eigen::VectorXd &dV,
                                Eigen::VectorXd &dTh,
                                double &tov);

        /* @compensateImu
            Inputs:
                dV: 3x1 dimensional vector of raw delta velocity measurements 
                dTh: 3x1 dimensional vector of raw delta theta measurments
                tov: measurement time of validity
                ba: 3x1 dimensional vector of accelerometr biases
                sfa: 3x1 dimensional vector of accelerometer scale factor errors
                ma: 6x1 dimensional vector of accelerometer misalignment errors
                bg: 3x1 dimensional vector of gyro biases
                sfg: 3x1 dimensional vector of gyro scale factor errors
                mg: 6x1 dimensional vector of gyro misalignment errors
            Outputs:
                dV: 3x1 dimensional vector of compensated delta velocity measurements 
                dTh: 3x1 dimensional vector of compensated delta theta measurments
            Description:
                Function which takes in raw IMU measurments and gyro/accelerometer estimates of bias,
                scale factor and misalignment. Measurmentds are compensated for tese effects and
                returned to be used in strapdown integration.
        */
        bool compensateImu(Eigen::VectorXd &dV,
                           Eigen::VectorXd &dTh,
                           double &tov,
                           Eigen::VectorXd &ba,
                           Eigen::VectorXd &sfa,
                           Eigen::VectorXd &ma,
                           Eigen::VectorXd &bg,
                           Eigen::VectorXd &sfg,
                           Eigen::VectorXd &mg); 

    // Private Class Members/Function
    private:

        // Position/Velocity/Attitude Values

        // Previous Strapdown Integration Quantities
        Eigen::VectorXd dV_prev_;
        Eigen::VectorXd dTh_prev_;
        double tov_prev_;

        /* @compensateMeasurement
            Inputs:
                meas: 3x1 dimensional vector of raw measurement values 
                tov: measurement time of validity
                b: 3x1 dimensional vector of biases
                sf: 3x1 dimensional vector of scale factor errors
                mis: 6x1 dimensional vector of misalignment errors
            Outputs:
                meas: 3x1 dimensional vector of compensated measurement values 
            Description:
                Function which takes in a raw measurement and applies the compensation algorithm
                to correct for bias, scale factor and misalignment.
        */
        bool compensateMeasurement(Eigen::VectorXd &meas,
                                   double &tov,
                                   Eigen::VectorXd &b,
                                   Eigen::VectorXd &sf,
                                   Eigen::VectorXd &mis);  


};

