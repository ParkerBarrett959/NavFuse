//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            GPS Sensor Model                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for GPS sensor model class.                                                    //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// GPS Sensor Class
class Gps {

    // Public Class Members/Functions
    public:

        /* @looselyCoupledGpsJ2k
            Inputs:
                x: nx1 dimensional filter state vector
                idx: 6x1 dimensional vector of indices corresponding to locations of position/velocity states
                velValid: Boolean indicating whether the supplied velocity measurements are valid
            Outputs:
                H: pxn dimensional linearized measurement model
            Description:
                Function which takes in the state vector, indices corresponding to the location of the position
                and velocity terms and a boolean indicating whether the velocity measurement is valid. The
                position and velocity states are expected to be in J2K inertial reference frame. The loosely-coupled
                GPS H matrix is computed.
        */
        bool looselyCoupledGpsJ2k(Eigen::VectorXd &x,
                                  Eigen::VectorXd &idx,
                                  bool &velValid,
                                  Eigen::MatrixXd &H);

    // Private Class Members/Function
    private:

};