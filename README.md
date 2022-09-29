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
./test_navigation_library
```
