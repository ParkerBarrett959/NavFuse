//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Navigation Rotations Functions                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for rotations class.                                                    //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "Gravity.hpp"

// Rotations Class
class Rotations {

    // Public Class Members/Functions
    public:

        /* @computeREcef2Ned
            Inputs:
                lat: scalar geodetic latitude  [rad]
                lon: scalar geodetic longitude [rad]
            Outputs:
                RE2N: 3x3 direction cosines matrix relating ECEF to the NED frame.
            Description:
                Function which takes in latitude and longitude and computes the direction cosines matrix
                from relating the Earth Centered Earth Fixed (ECEF) reference frame to the North-East-Down
                (NED) reference frame.
        */
        bool computeREcef2Ned(double &lat,
                              double &lon,
                              Eigen::Matrix3d &RE2N);

        /* @computeRNed2Ecef
            Inputs:
                lat: scalar geodetic latitude  [rad]
                lon: scalar geodetic longitude [rad]
            Outputs:
                Rn2E: 3x3 direction cosines matrix relating NED to the ECEF frame.
            Description:
                Function which takes in latitude and longitude and computes the direction cosines matrix
                from relating the North-East-Down (NED) reference frame to the Earth Centered Earth Fixed 
                (ECEF) reference frame.
        */
        bool computeRNed2Ecef(double &lat,
                              double &lon,
                              Eigen::Matrix3d &RN2E);

        /* @ecef2Lla
            Inputs:
                rE: 3x1 dimensional ECEF position vector [m]
            Outputs:
                lat: scalar geodetic latitude  [rad]
                lon: scalar geodetic longitude [rad]
                alt: scalar altitude above earth [m]
            Description:
                Function which takes in a position vector expressed in the earth-centered earth-fixed
                reference frame and computes the latitude, longitude and altitude of the position. The
                WGS-84 earth model is assumed.
        */
        bool ecef2Lla(Eigen::Vector3d &rE,
                      double &lat,
                      double &lon,
                      double &alt);

        /* @lla2Ecef
            Inputs:
                lat: scalar geodetic latitude  [rad]
                lon: scalar geodetic longitude [rad]
                alt: scalar altitude above earth [m]
            Outputs:
                rE: 3x1 dimensional ECEF position vector [m]
            Description:
                Function which takes in a the latitude, longitude and altitude of the position and. 
                computes the corresponding position vector in the earth-centered earth-fixed reference frame.
                The WGS-84 earth model is assumed.
        */
        bool lla2Ecef(double &lat,
                      double &lon,
                      double &alt,
                      Eigen::Vector3d &rE);

        /* @computeRJ2k2Ecef
            Inputs:
                dateVec: 6x1 date vector of format [YYYY, MM, DD, HH, MM, SS]
                eopFile: String Filepath+Filename of Earth Orientation Parameter .txt file
            Outputs:
                RJ2E: 3x3 direction cosines matrix relating J2K frame to the ECEF frame.
            Description:
                Function which takes in the date and a path to the earth orientation parameters .txt 
                file and computes the direction cosines matrix relating the J2K inertial frame to the 
                Earth-Centered Earth-Fixed reference frame.
            External Data:
                Earth orientation parameter files can be downloaded from here: https://celestrak.org/SpaceData/
                Downloading the latest csv file is recommended. Parsing of the csv is handled internally by 
                private class functions.
            Reference:
                Meysam Mahooti (2022). ECI2ECEF & ECEF2ECI Transformations
                (https://www.mathworks.com/matlabcentral/fileexchange/61957-eci2ecef-ecef2eci-transformations),
                MATLAB Central File Exchange. Retrieved October 13, 2022. 
        */
        bool computeRJ2k2Ecef(std::vector<int> &dateVec,
                              std::string &eopFile,
                              Eigen::Matrix3d &RJ2E);

        /* @computeREcef2J2k
            Inputs:
                dateVec: 6x1 date vector of format [YYYY, MM, DD, HH, MM, SS]
                eopFile: String Filepath+Filename of Earth Orientation Parameter .txt file
            Outputs:
                RE2J: 3x3 direction cosines matrix relating J2K frame to the ECEF frame.
            Description:
                Function which takes in the date and a path to the earth orientation parameters .txt 
                file and computes the direction cosines matrix relating the Earth-Centered Earth-Fixed 
                reference frame to the J2K Inertial reference frame.
            External Data:
                Earth orientation parameter files can be downloaded from here: https://celestrak.org/SpaceData/
                Downloading the latest csv file is recommended. Parsing of the csv is handled internally by 
                private class functions.
            Reference:
                Meysam Mahooti (2022). ECI2ECEF & ECEF2ECI Transformations
                (https://www.mathworks.com/matlabcentral/fileexchange/61957-eci2ecef-ecef2eci-transformations),
                MATLAB Central File Exchange. Retrieved October 13, 2022. 
        */
        bool computeREcef2J2k(std::vector<int> &dateVec,
                              std::string &eopFile,
                              Eigen::Matrix3d &RE2J);

    // Private Class Members/Function
    private:

        /* @convertDatevec2Mjd
            Inputs:
                dateVec: 6x1 date vector of format [YYYY, MM, DD, HH, MM, SS]
            Outputs:
                mjd: Scalar Modified Julian Date
            Description:
                Function which takes in a 6x1 date vector and computes the Modified Julian Date.
        */
        bool convertDatevec2Mjd(std::vector<int> &dateVec,
                                double &mjd);

        // Utility Class Instantiations
        Gravity Gravity_;

};