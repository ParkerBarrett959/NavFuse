############################################################
#              Cmake Build of Static Library               #
############################################################

# Project Setup
cmake_minimum_required(VERSION 3.24.0)
project(navigation_library)
set(CMAKE_CXX_STANDARD 11)

# Define Variables
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

# External Dependencies
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add Header List
set(HEADER_LIST ${PROJECT_SOURCE_DIR}/include/filter/BatchLeastSquares.hpp
                ${PROJECT_SOURCE_DIR}/include/filter/KalmanFilter.hpp
                ${PROJECT_SOURCE_DIR}/include/inertial/compensator.hpp
                ${PROJECT_SOURCE_DIR}/include/inertial/errorDynamics.hpp
                ${PROJECT_SOURCE_DIR}/include/inertial/initialization.hpp
                ${PROJECT_SOURCE_DIR}/include/inertial/strapdown.hpp
                ${PROJECT_SOURCE_DIR}/include/util/Gravity.hpp
                ${PROJECT_SOURCE_DIR}/include/util/NavUtils.hpp)

# Add Library
add_library(navigation STATIC ${PROJECT_SOURCE_DIR}/src/filter/BatchLeastSquares.cpp
                              ${PROJECT_SOURCE_DIR}/src/filter/KalmanFilter.cpp
                              ${PROJECT_SOURCE_DIR}/src/inertial/compensator.cpp
                              ${PROJECT_SOURCE_DIR}/src/inertial/errorDynamics.cpp
                              ${PROJECT_SOURCE_DIR}/src/inertial/initialization.cpp
                              ${PROJECT_SOURCE_DIR}/src/inertial/strapdown.cpp
                              ${PROJECT_SOURCE_DIR}/src/util/Gravity.cpp
                              ${PROJECT_SOURCE_DIR}/src/util/NavUtils.cpp ${HEADER_LIST})

# Target Include Directories
target_include_directories(navigation PUBLIC ${PROJECT_SOURCE_DIR}/include/filter
                                      PUBLIC ${PROJECT_SOURCE_DIR}/include/inertial
                                      PUBLIC ${PROJECT_SOURCE_DIR}/include/util
                                      PUBLIC ${EIGEN_INSTALL_PATH})
