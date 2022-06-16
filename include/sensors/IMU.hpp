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
#include <math.h>
#include "NavUtils.hpp"
#include "Gravity.hpp"
using Eigen::MatrixXd;
using Eigen::VectorXd;

// IMU Sensor Class
class ImuSensor {

    // Public Class Members/Functions
    public:

        // Position/Velocity/Attitude Values
        Eigen::Vector3d rI_;
        Eigen::Vector3d vI_;;
        Eigen::VectorXd qB2I_;
        int64_t tov_;
        double dt_;

        /* @strapdownInit
            Inputs:
                rInit: 3x1 dimensional vector of initial position  
                vInit: 3x1 dimensional vector of initial velocity
                qB2IInit: 4x1 dimensional vector of initial attitude quaternion
                tovInit: Initial time of validity
            Outputs:
            Description:
                Function which takes in initial values of the position, velocity, attitude and time of
                validity and sets the class variables.
        */
        bool strapdownInit(Eigen::Vector3d &rInit,
                           Eigen::Vector3d &vInit,
                           Eigen::VectorXd &qB2IInit,
                           int64_t &tovInit);

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
        bool strapdownIntegrate(Eigen::Vector3d &dV,
                                Eigen::Vector3d &dTh,
                                int64_t &tov);

    // Private Class Members/Function
    private:

        // Position/Velocity/Attitude Values
        Eigen::Vector3d rI_prev_;
        Eigen::Vector3d vI_prev_;
        Eigen::VectorXd qB2I_prev_;

        // Strapdown Integration Quantities
        int64_t tov_prev_;

        // Compensator Quantities
        Eigen::Vector3d ba_;
        Eigen::Vector3d sfa_;
        Eigen::VectorXd ma_;
        Eigen::Vector3d bg_;
        Eigen::Vector3d sfg_;
        Eigen::VectorXd mg_;

        /* @compensateImu
            Inputs:
                dV: 3x1 dimensional vector of raw delta velocity measurements 
                dTh: 3x1 dimensional vector of raw delta theta measurments
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
        bool compensateImu(Eigen::Vector3d &dV,
                           Eigen::Vector3d &dTh,
                           Eigen::Vector3d &ba,
                           Eigen::Vector3d &sfa,
                           Eigen::VectorXd &ma,
                           Eigen::Vector3d &bg,
                           Eigen::Vector3d &sfg,
                           Eigen::VectorXd &mg);         

        /* @computeqPrev2Curr
            Inputs:
                dTh: 3x1 dimensional vector of raw delta theta measurments 
            Outputs:
                qBprev2Bcurr: 4x1 dimensional quaternion relating the previous body frame to the current
            Description:
                Function which takes in a gyroscope delta theta measurmeent and computes the quaternion
                relating the previous body frame to the current body frame.
        */
        bool computeqPrev2Curr(Eigen::Vector3d &dTh,
                               Eigen::VectorXd &qBprev2Bcurr);

        /* @updateAttitude
            Inputs:
                qBprev2Bcurr: 4x1 dimensional quaternion relating the previous body frame to the current
            Outputs:
            Description:
                Function which uses the previous estimate of attitude and the change in attitude quaternion
                over the interval and computes the updates attitude.
        */
        bool updateAttitude(Eigen::VectorXd &qBprev2Bcurr); 

        /* @compensateMeasurement
            Inputs:
                meas: 3x1 dimensional vector of raw measurement values 
                b: 3x1 dimensional vector of biases
                sf: 3x1 dimensional vector of scale factor errors
                mis: 6x1 dimensional vector of misalignment errors
            Outputs:
                meas: 3x1 dimensional vector of compensated measurement values 
            Description:
                Function which takes in a raw measurement and applies the compensation algorithm
                to correct for bias, scale factor and misalignment.
        */
        bool compensateMeasurement(Eigen::Vector3d &meas,
                                   Eigen::Vector3d &b,
                                   Eigen::Vector3d &sf,
                                   Eigen::VectorXd &mis);  


};

