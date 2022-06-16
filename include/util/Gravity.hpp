//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Gravity Model Functions                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for gravity model class.                                                //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Gravity Class
class Gravity {

    // Public Class Members/Functions
    public:

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

    // Private Class Members/Function
    private:

        // Gravity Constants and Parameters
        double G = 6.674e-11;   // Universal Gravitation (N m^2 kg^-2)
        double mE = 5.972e24;   // Mass of Earth (kg) 

};

