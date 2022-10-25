//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Navigation Rotations Functions                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Class implementation which defines a set of helpful navigation rotation functions.  //          
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include "Rotations.hpp"

// Compute Rotation from ECEF to NED
bool Rotations::computeREcef2Ned(double &lat,
                                 double &lon,
                                 Eigen::Matrix3d &RE2N) {

    // Check for Lat/Lon Out of Range
    if (std::abs(lat) > (M_PI / 2.0)) {
        std::cout << "[Rotations::computeREcef2Ned] latitude provided is outside acceptable range. " << 
                "Expected: [-pi/2, pi/2], Got: " << lat << std::endl;
        return false;
    } else if (std::abs(lon) > M_PI) {
        std::cout << "[Rotations::computeREcef2Ned] longitude provided is outside acceptable range. " << 
                "Expected: [-pi, pi], Got: " << lon << std::endl;
        return false;
    }

    // Compute Halpful Quantities
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double slon = std::sin(lon);
    double clon = std::cos(lon);

    // Compute Rotation
    RE2N << -slat*clon,   -slat*slon,   clat,
                 -slon,         clon,      0,
            -clat*clon,   -clat*slon,  -slat;
    
    // Successful Return
    return true;

}

// Compute Rotation from NED to ECEF
bool Rotations::computeRNed2Ecef(double &lat,
                                 double &lon,
                                 Eigen::Matrix3d &RN2E) {

    // Check for Lat/Lon Out of Range
    if (std::abs(lat) > (M_PI / 2.0)) {
        std::cout << "[Rotations::computeRNed2Ecef] latitude provided is outside acceptable range. " << 
                "Expected: [-pi/2, pi/2], Got: " << lat << std::endl;
        return false;
    } else if (std::abs(lon) > M_PI) {
        std::cout << "[Rotations::computeRNed2Ecef] longitude provided is outside acceptable range. " << 
                "Expected: [-pi, pi], Got: " << lon << std::endl;
        return false;
    }

    // Compute Halpful Quantities
    double slat = std::sin(lat);
    double clat = std::cos(lat);
    double slon = std::sin(lon);
    double clon = std::cos(lon);

    // Compute Rotation
    RN2E << -slat*clon,   -slon,   -clat*clon,
            -slat*slon,    clon,   -clat*slon,
                  clat,       0,        -slat;
    
    // Successful Return
    return true;

}

// Compute Lat/Lon/Alt from ECEF Position
bool Rotations::ecef2Lla(Eigen::Vector3d &rE,
                         double &lat,
                         double &lon,
                         double &alt) {
    
    // Unpack Inputs
    double x = rE[0];
    double y = rE[1];
    double z = rE[2];
    
    // Compute WGS-84 Model Quantities
    double a = Gravity_.gravityParams.a;
    double b = Gravity_.gravityParams.b;
    double f = Gravity_.gravityParams.f;
    double e = std::sqrt(1 - std::pow(1-f, 2));
    double ep = std::sqrt((std::pow(a, 2) - std::pow(b, 2)) / std::pow(b, 2));
    double p = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    double th = std::atan2(a * z, b * p);

    // Compute Longitude
    lon = std::atan2(y, x);

    // Compute Latitude
    double latY = z + (std::pow(ep, 2) * b * std::pow(std::sin(th), 3));
    double latX = p - (std::pow(e, 2) * a * std::pow(std::cos(th), 3));
    lat = std::atan2(latY, latX);

    // Compute Altitude
    double N = a / std::sqrt(1 - (std::pow(e, 2) * std::pow(std::sin(lat), 2)));
    alt = (p / std::cos(lat)) - N;

    // Check Longitude Range [-pi, pi]
    lon = std::fmod(lon, 2.0 * M_PI);
    if (lon > M_PI) {
        lon -= (2.0 * M_PI);
    }
    
    // Successful Return
    return true;

}

// Compute ECEF Position from Lat/Lon/Alt
bool Rotations::lla2Ecef(double &lat,
                         double &lon,
                         double &alt,
                         Eigen::Vector3d &rE) {
    
    // Check for Lat/Lon Out of Range
    if (std::abs(lat) > (M_PI / 2.0)) {
        std::cout << "[Rotations::lla2Ecef] latitude provided is outside acceptable range. " << 
                "Expected: [-pi/2, pi/2], Got: " << lat << std::endl;
        return false;
    } else if (std::abs(lon) > M_PI) {
        std::cout << "[Rotations::lla2Ecef] longitude provided is outside acceptable range. " << 
                "Expected: [-pi, pi], Got: " << lon << std::endl;
        return false;
    }
    
    // Compute WGS-84 Model Quantities
    double a = Gravity_.gravityParams.a;
    double f = Gravity_.gravityParams.f;
    double e = std::sqrt(1 - std::pow(1-f, 2));

    // Compute Prime Vertical Radius of Curvature
    double N = a / std::sqrt(1 - (std::pow(e, 2) * std::pow(std::sin(lat), 2)));

    // Compute ECEF Position
    rE[0] = (N + alt) * std::cos(lat) * std::cos(lon);
    rE[1] = (N + alt) * std::cos(lat) * std::sin(lon);
    rE[2] = (((1 - std::pow(e, 2)) * N) + alt) * std::sin(lat);
    
    // Successful Return
    return true;

}

// Compute Rotation from J2K Inertial to ECEF
bool Rotations::computeRJ2k2Ecef(std::vector<double> &dateVec,
                                 std::string &eopFile,
                                 Eigen::Matrix3d &RJ2E) {
    
    // Verify Date Vector is in Correct Format
    if (dateVec.size() != 6) {
        std::cout << "[Rotations::ccomputeRJ2k2Ecef] Incorrect Date Vector Size" << std::endl;
        return false;
    }
    
    // Convert Date Vector to UTC Modified Julian Date
    double mjdUtc;
    if (!convertDatevec2Mjd(dateVec, mjdUtc)) {
        std::cout << "[Rotations::computeRJ2k2Ecef] Failed to compute Modified Julian Date" << std::endl;
        return false;
    }
    
    // Extract IERS Earth Orientation Parameters
    double xPole, yPole, Ut1_Utc, lod, Tai_Utc;
    if (!getCurrentEop(mjdUtc, xPole, yPole, Ut1_Utc, lod, Tai_Utc)) {
        std::cout << "[Rotations::computeRJ2k2Ecef] Failed to get EOPs" << std::endl;
        return false;
    }

    // Compute Time Differences
    double Tt_Tai = 32.184;
    double Utc_Tai = -Tai_Utc;
    double Tt_Utc = Tt_Tai - Utc_Tai;
    double Mjd_Ut1 = mjdUtc + (Ut1_Utc / 86400.0);
    double Mjd_Tt = mjdUtc + (Tt_Utc / 86400.0);

    // Compute Precession Matrix
    Eigen::Matrix3d RPrecession(3, 3);
    if (!computePrecession(astroConst.mjdJ2000, Mjd_Tt, RPrecession)) {
        std::cout << "[Rotations::computeRJ2k2Ecef] Failed to compute precession" << std::endl;
        return false;
    }

    // Compute Nutation Matrix
    Eigen::Matrix3d RNutation(3, 3);
    if (!computeNutation(Mjd_Tt, RNutation)) {
        std::cout << "[Rotations::computeRJ2k2Ecef] Failed to compute nutation" << std::endl;
        return false;
    }

    // Compute Greenwich Hour Angle Matrix
    Eigen::Matrix3d RGha(3, 3);
    if (!computeGreenwichHourAngle(Mjd_Ut1, Mjd_Tt, RGha)) {
        std::cout << "[Rotations::computeRJ2k2Ecef] Failed to compute Greenwich Hour Angle Matrix" << std::endl;
        return false;
    }

    // Compute Polar Motion
    Eigen::Matrix3d RPole(3, 3);
    if (!polarRotation(xPole, yPole, RPole)) {
        std::cout << "[Rotations::computeRJ2k2Ecef] Failed to compute Polar Rotation Matrix" << std::endl;
        return false;
    }

    // Compute Rotation from J2K to ECEF
    RJ2E = RPole * RGha * RNutation * RPrecession;
    
    // Successful Return
    return true;

}

// Compute Rotation from ECEF to J2K Inertial
bool Rotations::computeREcef2J2k(std::vector<double> &dateVec,
                                 std::string &eopFile,
                                 Eigen::Matrix3d &RE2J) {
    
    // Verify Date Vector is in Correct Format
    if (dateVec.size() != 6) {
        std::cout << "[Rotations::computeREcef2J2k] Incorrect Date Vector Size" << std::endl;
        return false;
    }
    
    // Convert Date Vector to UTC Modified Julian Date
    double mjdUtc;
    if (!convertDatevec2Mjd(dateVec, mjdUtc)) {
        std::cout << "[Rotations::computeREcef2J2k] Failed to compute Modified Julian Date" << std::endl;
        return false;
    }

    // Extract IERS Earth Orientation Parameters
    double xPole, yPole, Ut1_Utc, lod, Tai_Utc;
    if (!getCurrentEop(mjdUtc, xPole, yPole, Ut1_Utc, lod, Tai_Utc)) {
        std::cout << "[Rotations::computeREcef2J2k] Failed to get EOPs" << std::endl;
        return false;
    }

    // Compute Time Differences
    double Tt_Tai = 32.184;
    double Utc_Tai = -Tai_Utc;
    double Tt_Utc = Tt_Tai - Utc_Tai;
    double Mjd_Ut1 = mjdUtc + (Ut1_Utc / 86400.0);
    double Mjd_Tt = mjdUtc + (Tt_Utc / 86400.0);

    // Compute Precession Matrix
    Eigen::Matrix3d RPrecession(3, 3);
    if (!computePrecession(astroConst.mjdJ2000, Mjd_Tt, RPrecession)) {
        std::cout << "[Rotations::computeREcef2J2k] Failed to compute precession" << std::endl;
        return false;
    }

    // Compute Nutation Matrix
    Eigen::Matrix3d RNutation(3, 3);
    if (!computeNutation(Mjd_Tt, RNutation)) {
        std::cout << "[Rotations::computeREcef2J2k] Failed to compute nutation" << std::endl;
        return false;
    }

    // Compute Greenwich Hour Angle Matrix
    Eigen::Matrix3d RGha(3, 3);
    if (!computeGreenwichHourAngle(Mjd_Ut1, Mjd_Tt, RGha)) {
        std::cout << "[Rotations::computeREcef2J2k] Failed to compute Greenwich Hour Angle Matrix" << std::endl;
        return false;
    }

    // Compute Polar Motion
    Eigen::Matrix3d RPole(3, 3);
    if (!polarRotation(xPole, yPole, RPole)) {
        std::cout << "[Rotations::computeREcef2J2k] Failed to compute Polar Rotation Matrix" << std::endl;
        return false;
    }

    // Compute Rotation from ECEF to J2K
    RE2J = (RPole * RGha * RNutation * RPrecession).transpose();
    
    // Successful Return
    return true;

}

// Get Earth Orientation Parameters
bool Rotations::getEops(const std::string eop) {

    // Initialize Outputs
    std::vector<double> mjd;
    std::vector<double> xPole;
    std::vector<double> yPole;
    std::vector<double> Ut1_Utc;
    std::vector<double> lod;
    std::vector<double> Tai_Utc;
    
    // Open CSV File
    std::fstream file;
    file.open(eop);
    std::string line, data;

    // Get Rows
    double element;
    std::getline(file, line, '\n');
    while (std::getline(file, line, '\n')) {

        // Set String Stream
        int temp = 0;
        std::stringstream str(line);

        // Get Columns
        while (std::getline(str, data, ',')) {

            // Get Data
            if (temp == 1) {
                element = std::stod(data);
                mjd.push_back(element); 
            } else if (temp == 2) {
                element = std::stod(data);
                xPole.push_back(element);
            } else if (temp == 3) {
                element = std::stod(data);
                yPole.push_back(element);
            } else if (temp == 4) {
                element = std::stod(data);
                Ut1_Utc.push_back(element);
            } else if (temp == 5) {
                element = std::stod(data);
                lod.push_back(element);
            } else if (temp == 10) {
                element = std::stod(data);
                Tai_Utc.push_back(element);
            }

            // Increment Temporary Iterator
            temp++;
        }
    }

    // Close File
    file.close();

    // Set EOPs to Class Variable
    EOPs.mjd = mjd;
    EOPs.xPole = xPole;
    EOPs.yPole = yPole;
    EOPs.Ut1_Utc = Ut1_Utc;
    EOPs.lod = lod;
    EOPs.Tai_Utc = Tai_Utc;

    // Set Flag for EOPs Set
    eopsSet_ = true;

    // Successful Return
    return true;

}

// Convert Date Vector to Modified Julian Date
bool Rotations::convertDatevec2Mjd(std::vector<double> &dateVec,
                                   double &mjd) {

    // Unpacks Inputs
    double year = dateVec[0];
    double month = dateVec[1];
    double day = dateVec[2];
    double hour = dateVec[3];
    double min = dateVec[4];
    double sec = dateVec[5];

    // Check for Invalid Inputs
    if ((month > 12) || (month < 0)) {
        std::cout << "[Rotations::convertDatevec2Mjd] Invalid Month" << std::endl;
        return false;
    } else if ((day > 31) || (day < 0)) {
        std::cout << "[Rotations::convertDatevec2Mjd] Invalid Day" << std::endl;
        return false;
    } else if ((hour > 23) || (hour < 0)) {
        std::cout << "[Rotations::convertDatevec2Mjd] Invalid Hour" << std::endl;
        return false;
    } else if ((min > 59) || (min < 0)) {
        std::cout << "[Rotations::convertDatevec2Mjd] Invalid Minute" << std::endl;
        return false;
    } else if ((sec > 59) || (sec < 0)) {
        std::cout << "[Rotations::convertDatevec2Mjd] Invalid Second" << std::endl;
        return false;
    }

    // Month Wrap-Around
    if (month <= 2.0) {
        month += 12.0;
        year -= 1.0;
    }

    // Compute b
    double b;
    if (((10000.0 * year) + (100.0 * month) + day) <= 15821004.0) {
        b = -2.0 + std::floor((year + 4716.0) / 4.0) - 1179.0;
    } else {
        b = std::floor(year / 400.0) - std::floor(year / 100.0) + std::floor(year / 4.0);
    }
    
    // Compute Whole Day Modified Julian Date
    double MjdMidnight = (365.0 * year) - 679004.0 + b + std::floor(30.6001 * (month + 1.0)) + day;
    
    // Compute Fractional Day
    double fracDay = (hour + (min / 60.0) + (sec / 3600.0)) / 24.0;
    
    // Compute Modified Julian Date
    mjd = MjdMidnight + fracDay;
    
    // Successful Return
    return true;

}

// Compute Precession Matrix
bool Rotations::computePrecession(double &mjd1,
                                  double &mjd2,
                                  Eigen::Matrix3d &RPrecession) {

    // Compute Helpful Quantities
    double T = (mjd1 - astroConst.mjdJ2000) / 36525.0;
    double dT = (mjd2 - mjd1) / 36525.0;
    
    // Compute Pecession Angles
    double zeta = ((2306.2181 + (1.39656 - 0.000139 * T) * T)+ ((0.30188 - 0.000344 * T)
        + 0.017998 * dT) * dT) * dT / astroConst.Arcs;
    double z =  zeta + ((0.79280 + 0.000411 * T) + 0.000205 * dT) * dT * dT / astroConst.Arcs;
    double theta = ((2004.3109 - (0.85330 + 0.000217 * T) * T) - ((0.42665 + 0.000217 * T)
        + 0.041833 * dT) * dT) * dT / astroConst.Arcs;
    
    // Compute Precession Rotations
    Eigen::Matrix3d Rz1(3, 3), Ry(3, 3), Rz2(3, 3);
    Rz1 << std::cos(-zeta),  std::sin(-zeta),  0.0,
          -std::sin(-zeta),  std::cos(-zeta),  0.0,
                       0.0,              0.0,  1.0;
    Ry << std::cos(theta),  0.0, -std::sin(theta),
                      0.0,  1.0,              0.0,
          std::sin(theta),  0.0,  std::cos(theta);
    Rz2 << std::cos(-z),  std::sin(-z),  0.0,
          -std::sin(-z),  std::cos(-z),  0.0,
                    0.0,           0.0,  1.0;
    
    // Compute Precession Matrix
    RPrecession = Rz2 * Ry * Rz1;

    // Successful Return
    return true;

}

// Compute Nutation Matrix Matrix
bool Rotations::computeNutation(double &Mjd_Tt,
                                Eigen::Matrix3d &RNutation) {

    // Compute Mean Obliquity of Ecliptic
    double T = (Mjd_Tt - astroConst.mjdJ2000) / 36525.0;
    double ep = astroConst.rad * (23.43929111 - (46.8150 + (0.00059 - 0.001813 * T) * T)
        * T / 3600.0);

    // Get Nutation Angles
    double dpsi, deps;
    if (!nutationAngles(Mjd_Tt, dpsi, deps)) {
        std::cout << "[Rotations::computeNutation] Unable to compute nutation angles" << std::endl;
        return false;
    }
    
    // Compute Nutation Rotations
    Eigen::Matrix3d Rx1(3, 3), Rx2(3, 3), Rz(3, 3);
    Rx1 << 1.0,           0.0,          0.0,
           0.0,  std::cos(ep), std::sin(ep),
           0.0, -std::sin(ep), std::cos(ep); 
    Rx2 << 1.0,                 0.0,                0.0,
           0.0,  std::cos(-ep-deps), std::sin(-ep-deps),
           0.0, -std::sin(-ep-deps), std::cos(-ep-deps); 
    Rz << std::cos(-dpsi),  std::sin(-dpsi),  0.0,
         -std::sin(-dpsi),  std::cos(-dpsi),  0.0,
                      0.0,              0.0,  1.0;

    // Compute Precession Matrix
    RNutation = Rx2 * Rz * Rx1;

    // Successful Return
    return true;

}

// Compute Greenwich Hour Angle Matrix
bool Rotations::computeGreenwichHourAngle(double &Mjd_Ut1,
                                          double &Mjd_Tt,
                                          Eigen::Matrix3d &RGha) {
    
    // Compute Greenwich Mean Sidereal Time
    double secs = 86400.0;
    double mjd0 = std::floor(Mjd_Ut1);
    double Ut1 = secs * (Mjd_Ut1 - mjd0);
    double t0 = (mjd0 - astroConst.mjdJ2000) / 36525.0;
    double t = (Mjd_Ut1 - astroConst.mjdJ2000) / 36525.0;
    double gmst = 24110.54841 + 8640184.812866 * t0 + 1.002737909350795 * Ut1 + (0.093104 - 6.2e-6 * t) * t * t;
    double gmsTime = 2.0 * M_PI * ((gmst / secs) - std::floor(gmst / secs));

    // Compute Equations of Equinox
    double dpsi, deps;
    if (!nutationAngles(Mjd_Tt, dpsi, deps)) {
        std::cout << "[Rotations::computeGreenwichHourAngle] Unable to compute nutation angles" << std::endl;
        return false;
    }
    double T = (Mjd_Tt - astroConst.mjdJ2000) / 36525.0;
    double meanObliquity = astroConst.rad * (23.43929111 - (46.8150 + (0.00059 - 0.001813 * T) * T) * T / 3600.0);
    double EqE = dpsi * std::cos(meanObliquity);

    
    // Compute Greenwich Apparent Sidereal Time
    double gsTime = std::fmod(gmsTime + EqE, 2.0 * M_PI);

    // Compute Greenwich Hour Angle Matrix
    RGha << std::cos(gsTime),  std::sin(gsTime),  0.0,
           -std::sin(gsTime),  std::cos(gsTime),  0.0,
                         0.0,               0.0,  1.0;

    // Successful Return
    return true;
                                          
}

// Compute Polar Motion Rotation
bool Rotations::polarRotation(double &xPole,
                              double &yPole,
                              Eigen::Matrix3d &RPole) {

    // Compute Polar Rotations
    Eigen::Matrix3d Rx(3, 3), Ry(3, 3);
    Rx << 1.0,               0.0,              0.0,
          0.0,  std::cos(-yPole), std::sin(-yPole),
          0.0, -std::sin(-yPole), std::cos(-yPole); 
    Ry << std::cos(-xPole),  0.0, -std::sin(-xPole),
                       0.0,  1.0,               0.0,
          std::sin(-xPole),  0.0,  std::cos(-xPole);

    // Compute Polar Motion Rotation
    RPole = Ry * Rx;
    
    // Successful Return
    return true;

}

// Compute Nutation Angles
bool Rotations::nutationAngles(double &Mjd_Tt,
                               double &dpsi,
                               double &deps) {

    // Compute Nutation in Longitude and Obliquity
    double T = (Mjd_Tt - astroConst.mjdJ2000) / 36525.0;
    double T2 = T*T;
    double T3 = T2*T;
    double rev = 360.0 * 3600.0;
    int N_coeff = 106;
    int C[106][9] = {
        { 0, 0, 0, 0, 1,-1719960,-1742,  920250,   89},
        { 0, 0, 0, 0, 2,   20620,    2,   -8950,    5},
        {-2, 0, 2, 0, 1,     460,    0,    -240,    0},
        { 2, 0,-2, 0, 0,     110,    0,       0,    0},
        {-2, 0, 2, 0, 2,     -30,    0,      10,    0},
        { 1,-1, 0,-1, 0,     -30,    0,       0,    0},
        { 0,-2, 2,-2, 1,     -20,    0,      10,    0},
        { 2, 0,-2, 0, 1,      10,    0,       0,    0},
        { 0, 0, 2,-2, 2, -131870,  -16,   57360,  -31},
        { 0, 1, 0, 0, 0,   14260,  -34,     540,   -1},
        { 0, 1, 2,-2, 2,   -5170,   12,    2240,   -6},
        { 0,-1, 2,-2, 2,    2170,   -5,    -950,    3},
        { 0, 0, 2,-2, 1,    1290,    1,    -700,    0},
        { 2, 0, 0,-2, 0,     480,    0,      10,    0},
        { 0, 0, 2,-2, 0,    -220,    0,       0,    0},
        { 0, 2, 0, 0, 0,     170,   -1,       0,    0},
        { 0, 1, 0, 0, 1,    -150,    0,      90,    0},
        { 0, 2, 2,-2, 2,    -160,    1,      70,    0},
        { 0,-1, 0, 0, 1,    -120,    0,      60,    0},
        {-2, 0, 0, 2, 1,     -60,    0,      30,    0},
        { 0,-1, 2,-2, 1,     -50,    0,      30,    0},
        { 2, 0, 0,-2, 1,      40,    0,     -20,    0},
        { 0, 1, 2,-2, 1,      40,    0,     -20,    0},
        { 1, 0, 0,-1, 0,     -40,    0,       0,    0},
        { 2, 1, 0,-2, 0,      10,    0,       0,    0},
        { 0, 0,-2, 2, 1,      10,    0,       0,    0},
        { 0, 1,-2, 2, 0,     -10,    0,       0,    0},
        { 0, 1, 0, 0, 2,      10,    0,       0,    0},
        {-1, 0, 0, 1, 1,      10,    0,       0,    0},
        { 0, 1, 2,-2, 0,     -10,    0,       0,    0},
        { 0, 0, 2, 0, 2,  -22740,   -2,    9770,   -5},
        { 1, 0, 0, 0, 0,    7120,    1,     -70,    0},
        { 0, 0, 2, 0, 1,   -3860,   -4,    2000,    0},
        { 1, 0, 2, 0, 2,   -3010,    0,    1290,   -1},
        { 1, 0, 0,-2, 0,   -1580,    0,     -10,    0},
        {-1, 0, 2, 0, 2,    1230,    0,    -530,    0},
        { 0, 0, 0, 2, 0,     630,    0,     -20,    0},
        { 1, 0, 0, 0, 1,     630,    1,    -330,    0},
        {-1, 0, 0, 0, 1,    -580,   -1,     320,    0},
        {-1, 0, 2, 2, 2,    -590,    0,     260,    0},
        { 1, 0, 2, 0, 1,    -510,    0,     270,    0},
        { 0, 0, 2, 2, 2,    -380,    0,     160,    0},
        { 2, 0, 0, 0, 0,     290,    0,     -10,    0},
        { 1, 0, 2,-2, 2,     290,    0,    -120,    0},
        { 2, 0, 2, 0, 2,    -310,    0,     130,    0},
        { 0, 0, 2, 0, 0,     260,    0,     -10,    0},
        {-1, 0, 2, 0, 1,     210,    0,    -100,    0},
        {-1, 0, 0, 2, 1,     160,    0,     -80,    0},
        { 1, 0, 0,-2, 1,    -130,    0,      70,    0},
        {-1, 0, 2, 2, 1,    -100,    0,      50,    0},
        { 1, 1, 0,-2, 0,     -70,    0,       0,    0},
        { 0, 1, 2, 0, 2,      70,    0,     -30,    0},
        { 0,-1, 2, 0, 2,     -70,    0,      30,    0},
        { 1, 0, 2, 2, 2,     -80,    0,      30,    0},
        { 1, 0, 0, 2, 0,      60,    0,       0,    0},
        { 2, 0, 2,-2, 2,      60,    0,     -30,    0},
        { 0, 0, 0, 2, 1,     -60,    0,      30,    0},
        { 0, 0, 2, 2, 1,     -70,    0,      30,    0},
        { 1, 0, 2,-2, 1,      60,    0,     -30,    0},
        { 0, 0, 0,-2, 1,     -50,    0,      30,    0},
        { 1,-1, 0, 0, 0,      50,    0,       0,    0},
        { 2, 0, 2, 0, 1,     -50,    0,      30,    0},
        { 0, 1, 0,-2, 0,     -40,    0,       0,    0},
        { 1, 0,-2, 0, 0,      40,    0,       0,    0},
        { 0, 0, 0, 1, 0,     -40,    0,       0,    0},
        { 1, 1, 0, 0, 0,     -30,    0,       0,    0},
        { 1, 0, 2, 0, 0,      30,    0,       0,    0},
        { 1,-1, 2, 0, 2,     -30,    0,      10,    0},
        {-1,-1, 2, 2, 2,     -30,    0,      10,    0},
        {-2, 0, 0, 0, 1,     -20,    0,      10,    0},
        { 3, 0, 2, 0, 2,     -30,    0,      10,    0},
        { 0,-1, 2, 2, 2,     -30,    0,      10,    0},
        { 1, 1, 2, 0, 2,      20,    0,     -10,    0},
        {-1, 0, 2,-2, 1,     -20,    0,      10,    0},
        { 2, 0, 0, 0, 1,      20,    0,     -10,    0},
        { 1, 0, 0, 0, 2,     -20,    0,      10,    0},
        { 3, 0, 0, 0, 0,      20,    0,       0,    0},
        { 0, 0, 2, 1, 2,      20,    0,     -10,    0},
        {-1, 0, 0, 0, 2,      10,    0,     -10,    0},
        { 1, 0, 0,-4, 0,     -10,    0,       0,    0},
        {-2, 0, 2, 2, 2,      10,    0,     -10,    0},
        {-1, 0, 2, 4, 2,     -20,    0,      10,    0},
        { 2, 0, 0,-4, 0,     -10,    0,       0,    0},
        { 1, 1, 2,-2, 2,      10,    0,     -10,    0},
        { 1, 0, 2, 2, 1,     -10,    0,      10,    0},
        {-2, 0, 2, 4, 2,     -10,    0,      10,    0},
        {-1, 0, 4, 0, 2,      10,    0,       0,    0},
        { 1,-1, 0,-2, 0,      10,    0,       0,    0},
        { 2, 0, 2,-2, 1,      10,    0,     -10,    0},
        { 2, 0, 2, 2, 2,     -10,    0,       0,    0},
        { 1, 0, 0, 2, 1,     -10,    0,       0,    0},
        { 0, 0, 4,-2, 2,      10,    0,       0,    0},
        { 3, 0, 2,-2, 2,      10,    0,       0,    0},
        { 1, 0, 2,-2, 0,     -10,    0,       0,    0},
        { 0, 1, 2, 0, 1,      10,    0,       0,    0},
        {-1,-1, 0, 2, 1,      10,    0,       0,    0},
        { 0, 0,-2, 0, 1,     -10,    0,       0,    0},
        { 0, 0, 2,-1, 2,     -10,    0,       0,    0},
        { 0, 1, 0, 2, 0,     -10,    0,       0,    0},
        { 1, 0,-2,-2, 0,     -10,    0,       0,    0},
        { 0,-1, 2, 0, 1,     -10,    0,       0,    0},
        { 1, 1, 0,-2, 1,     -10,    0,       0,    0},
        { 1, 0,-2, 2, 0,     -10,    0,       0,    0},
        { 2, 0, 0, 2, 0,      10,    0,       0,    0},
        { 0, 0, 2, 4, 2,     -10,    0,       0,    0},
        { 0, 1, 0, 1, 0,      10,    0,       0,    0}
    };  
    double l  = std::fmod (485866.733 + (1325.0 * rev +  715922.633) * T + 31.310 * T2 + 0.064 * T3, rev);
    if (l < 0) {
        l += rev;
    } 
    double lp = std::fmod (1287099.804 + (99.0 * rev + 1292581.224) * T - 0.577 * T2 - 0.012 * T3, rev);
    if (lp < 0) {
        lp += rev;
    } 
    double F  = std::fmod (335778.877 + (1342.0 * rev +  295263.137) * T - 13.257 * T2 + 0.011 * T3, rev);
    if (F < 0) {
        F += rev;
    } 
    double D  = std::fmod (1072261.307 + (1236.0 * rev + 1105601.328) * T -  6.891 * T2 + 0.019 * T3, rev);
    if (D < 0) {
        D += rev;
    } 
    double Om = std::fmod (450160.280 - (5.0 * rev +  482890.539) * T + 7.455 * T2 + 0.008 * T3, rev);
    if (Om < 0) {
        Om += rev;
    } 

    // Compute Nutation Angles
    dpsi = 0.0;
    deps = 0.0;
    double arg = 0.0;
    for (int i=0; i < N_coeff; i++) {
        arg  =  (C[i][0] * l + C[i][1] * lp + C[i][2] * F + C[i][3] * D + C[i][4] * Om ) / astroConst.Arcs;
        dpsi += (C[i][5] + C[i][6] * T) * std::sin(arg);
        deps += (C[i][7] + C[i][8] * T) * std::cos(arg);
    }  
    dpsi = 1e-5 * dpsi / astroConst.Arcs;
    deps = 1e-5 * deps / astroConst.Arcs;

    // Successful Return
    return true;

}

// Get Current Earth Orientation Parameters
bool Rotations::getCurrentEop(double &mjd,
                              double &xPole,
                              double &yPole,
                              double &Ut1_Utc,
                              double &lod,
                              double &Tai_Utc) {

    // Find Index of Current MJD Start of Day
    double mjdStart = std::floor(mjd);
    int idx = -1;
    for (int ii = 0; ii < EOPs.mjd.size(); ii++) {
        if (EOPs.mjd[ii] == mjdStart) {
            idx = ii;
            break;
        }
    }

    // Check for MJD Out of Range
    if (idx == -1) {
        std::cout << "[Rotations::getCurrentEop] Current EOP data out of range" << std::endl;
        return false;
    }

    // Compute Pre and Post Indices
    double preXPole = EOPs.xPole[idx];
    double preYPole = EOPs.yPole[idx];
    double preUt1_Utc = EOPs.Ut1_Utc[idx];
    double preLod = EOPs.lod[idx];
    double preTai_Utc = EOPs.Tai_Utc[idx];
    if (!(idx == EOPs.mjd.size() - 1)) {
        idx += 1;
    }
    double postXPole = EOPs.xPole[idx];
    double postYPole = EOPs.yPole[idx];
    double postUt1_Utc = EOPs.Ut1_Utc[idx];
    double postLod = EOPs.lod[idx];

    // Get Interpolation Stepsize
    double fixf = mjd - mjdStart;

    // Interpolate EOPS
    xPole = (preXPole + (postXPole - preXPole) * fixf) / astroConst.Arcs;
    yPole = (preYPole + (postYPole - preYPole) * fixf) / astroConst.Arcs;
    Ut1_Utc = preUt1_Utc + (postUt1_Utc - preUt1_Utc) * fixf;
    lod = preLod + (postLod- preLod) * fixf;
    Tai_Utc = preTai_Utc;
    
    // Successful Return
    return true;

}