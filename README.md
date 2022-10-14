![image](https://user-images.githubusercontent.com/89047457/192758897-683c1385-aa26-4ac6-8651-56016523001a.png)

# A Robust Navigation and Sensor Fusion C++ Library
NavFuse is a robust, unit tested C++ library for navigation and sensor fusion applications. The library contains fully tested Kalman Filtering and other state estimation/sensor fusion algorithm classes. It also features a variety of strapdown inertial navigation functionality and commonly used navigation utility functions including gravity models, attitude representations and rotations.

[![AutomatedTests Actions Status](https://github.com/ParkerBarrett959/NavFuse/workflows/NavFuse-master/badge.svg)](https://github.com/ParkerBarrett959/NavFuse/actions)

# Dependencies
* C++ 11 (or greater) <br />
* CMake (3.22.0 or greater) <br />
* Eigen (3.3 or greater) <br />

# Build
```
mkdir build
cd build
cmake ..
make
```
# Run Unit Tests
```
./test_nav_fuse
```
# External Data
NavFuse requires external data, primary time-varying earth orientation parameters to effectively compute rotations. The earth orientation parameters can be downloaded and used as follows:

* Step 1: Navigate to the CelesTrak Space Data Page: https://celestrak.org/SpaceData/
* Step 2: In the "Earth Orientation Parameter (EOP) Data" Box, download the latest csv file
* Step 3: Move the downloaded csv file to the /NavFuse/data/ directory

Note: NavFuse looks for the EOP file specified at runtime - you do not need to rebuild if you change files or did not follow these instructions during the initial build
