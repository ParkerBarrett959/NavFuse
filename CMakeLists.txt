# Cmake Build of Static Library
cmake_minimum_required(VERSION 3.24.0)
project(navigation_library)

# Set Source File 
set(SOURCE_FILES main.cpp)

# Added Executable
add_executable(main ${SOURCE_FILES}) 
