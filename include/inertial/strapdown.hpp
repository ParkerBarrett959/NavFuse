//////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Inertial Navigation Strapdown Integration                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial navigation strapdown class.                                //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <Eigen/Dense>
#include "NavUtils.hpp"
#include "Gravity.hpp"

// Inertial Navigation Strapdown Class
class Strapdown {

    // Public Class Members/Functions
    public:

        // Public Class Members
        Eigen::Vector3d lla_;
        Eigen::Vector3d vNed_;
        Eigen::Vector3d rph_;
        int64_t tov_;
        double dt_;

        /* @initialize
            Inputs:
                lla: 3x1 vector of geodetic position [rad, rad, m]
                vNed: 3x1 vector of NED velocities [m/s]
                rph: 3x1 vector of attitude - roll, pitch, heading [rad] 
                tov: scalar int64 representing initialization time of validity [micro-seconds Unix] 
            Outputs:
            Description:
                Function which initializes the position, velocity and attitude for the strapdown
                inertial navigator.
        */
        bool initialize(Eigen::Vector3d &lla,
                        Eigen::Vector3d &vNed,
                        Eigen::Vector3d &rph,
                        int64_t &tov);

        /* @integrate
            Inputs:
                dV: 3x1 vector of change in velocity measurement [m/s^2]
                dTh: 3x1 vector of angle measurement [rad]
                tov: scalar int64 representing measurement time of validity [micro-seconds Unix] 
            Outputs:
            Description:
                Function which takes in the IMU measurement and performs strapdown integration. The
                algorithm begins with updating the attitude from the gyroscope measurement. The 
                accelerometer measurements are rotated to the updated frame and integrated to get 
                velocity and position.
        */
        bool integrate(Eigen::Vector3d &dV,
                       Eigen::Vector3d &dTh,
                       int64_t &tov);

    // Private Class Members/Function
    private:

        // Private Class Members
        Eigen::Vector3d llaPrev_;
        Eigen::Vector3d vNedPrev_;
        Eigen::Vector3d rphPrev_;
        int64_t tovPrev_;

        // Utility Object Instantiations
        NavUtils NavUtil;
        Gravity Gravity;

};
