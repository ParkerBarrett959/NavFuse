//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Base Kalman Filter Header                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the Base Kalman Filter class                                        //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <Eigen/Dense>

// Kalman Filter Class
class BaseFilter {

    // Public Class Members/Functions
    public:

        /* @filterInitialize
            Inputs:
                x0: nx1 dimensional initial state vector
                P0: nxn dimensional initial covariance matrix
            Outputs:
            Description:
                Virtual function which takes in the initial state vector and covariance matrix and
                sets the filter class members
        */
        virtual bool filterInitialize(Eigen::VectorXd x0,
                                      Eigen::MatrixXd P0);

        /* @getCovariance
            Inputs:
            Outputs:
                Pk: nxn dimensional covariance matrix
            Description:
                Virtual function which returns the current filter covariance matrix.
        */
        virtual Eigen::MatrixXd getCovariance() {return filterCovariance_;};

        /* @getState
            Inputs:
            Outputs:
                xk: nx1 dimensional state vector
            Description:
                Virtual function which returns the current filter state vector.
        */
        virtual Eigen::MatrixXd getState() {return filterState_;};

    // Protected Class Members/Function
    protected:

        // Filter State and Covariance
        Eigen::MatrixXd filterCovariance_;      // nxn dimensional filter covariance matrix
        Eigen::VectorXd filterState_;           // nx1 dimensional filter state vector

};

