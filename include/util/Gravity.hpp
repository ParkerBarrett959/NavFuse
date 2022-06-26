//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Gravity Model Functions                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for gravity model class.                                                //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Gravity Class
class Gravity {

    // Public Class Members/Functions
    public:

        // Gravity Constants and Parameters
        struct {
            double a = 6378137;                 // Earth Equatorial Radius [m]
            double b = 6356752.3141;            // Earth Ellipsoid Semi-Major Axis [m]
            double J2 = 1.08263 * 1e-3;         // Earth Dynamical Flattening Term
            double kM = 3.986005 * 1e14;        // Newtonian Gravitational Constant [m^3/s^2]
            double wE = 7.292115 * 1e-5;        // Earth Rotation Rate [rad/s]
            double G = 6.674 * 1e-11;           // Universal Gravitation (N m^2 kg^-2)
            double mE = 5.972 * 1e24;           // Mass of Earth (kg) 
            double gEq = 9.7803267715;          // Normal Gravity at Equator [m/s^2]
            double gPole = 9.8321863685;        // Normal Gravity at Poles [m/s^2]
            double f = 1.0 / 298.257222101;     // Earth Ellipsoid Flattening
        } gravityParams;

        /* @simpleGravity
            Inputs:
                rA:3x1 dimensional position vector in arbitrary reference frame A
            Outputs:
                gA: 3x1 dimensional gravitational acceleration vector in reference frame A
            Description:
                Function which takes in a position vector in an arbitrary reference frame and computes
                a simple approximation of the gravity vector in this frame. For geodetic applications, 
                the sipmlified approximation is accurate to approximately 1-2%.
        */
        bool simpleGravity(Eigen::Vector3d &rA,
                           Eigen::Vector3d &gA);

        /* @gravityNed
            Inputs:
                lat: scalar double representing latitude of the body
                h: scalar double representing the body's height above the earth reference ellipsoid 
            Outputs:
                gN: 3x1 dimensional gravitational acceleration vector in the NED Frame
            Description:
                Function which takes in a geodetic latitude and altitude, and computes the gravity 
                vector in the NED reference frame. The normal gravity is computed on the ellipsoid 
                using the Somigliana formula, followed by an approximation to ccount for gravity. A
                final approximation of the normal gravity in the North direction is included, which
                becomes more significant at higher altitudes. The model is accurate to approximately
                10^-6 m/s^2 at altitudes of up to 20km, and more accurate at lower altitudes.
        */
        bool gravityNed(double &lat,
                        double &h,
                        Eigen::Vector3d &gN);

    // Private Class Members/Function
    private:

};

