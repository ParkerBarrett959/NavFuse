//////////////////////////////////////////////////////////////////////////////////////////////////////
//                         Inertial Navigation Initialization and Alignment                         //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for inertial navigation initialization and alignment class.             //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <cmath>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Inertial Navigation Initialization and Alignment Class
class Initialization {

    // Public Class Members/Functions
    public:

        /* @coarseAttitudeAlignment
            Inputs:
                lat: scalar double representing the geodetic latitude [rad]
                wE: scalar double representing Earth rotational rate magnitude [rad/s]
                g: scalar double representing gravitational acceleration magnitude [m/s^2]
                aB: 3x1 vector of accelerometer specific force outputs [m/s^2]
                wIB_B: 3x1 vector angular rate of body wrt inertial expressed in the body frame
            Outputs:
                RB2N: 3x3 dimensional direction cosines matrix from Body to NED Navigation frame
            Description:
                Function which takes in geodetic latitude, earth parameters (rotation rate and
                gravitational acceleration), body angular rate with respect to the inertial frame,
                and the accelerometer outputs in the body frame, and outputs the direction cosines
                matrix relating the body frame to the NED navigation frame. The coarse attitude 
                alignment assumes the INS is stationary on the surface of the earth and does not 
                account for accelerometer and gyroscope errors.
        */
        bool coarseAttitudeAlignment(double &lat,
                                     double &wE,
                                     double &g,
                                     Eigen::Vector3d &aB,
                                     Eigen::Vector3d &wIB_B,
                                     Eigen::Matrix3d &RB2N);

        /* @fineAttitudeAlignment
            Inputs:
                rA:3x1 dimensional position vector in arbitrary reference frame A
            Outputs:
                gA: 3x1 dimensional gravitational acceleration vector in reference frame A
            Description:
                Function which takes in a position vector in an arbitrary reference frame and computes
                a simple approximation of the gravity vector in this frame. For geodetic applications, 
                the sipmlified approximation is accurate to approximately 1-2%.
        */

    // Private Class Members/Function
    private:

        

};

