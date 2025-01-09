
cmake_minimum_required(VERSION 3.14)  # Updated to at least 3.14 or whatever your environment supports
project(KinectCode)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Locate libraries
find_package(k4a REQUIRED)
find_package(k4abt REQUIRED)

# Optional: Include CUDA if your project needs GPU acceleration
# find_package(CUDA REQUIRED)  # Uncomment if using CUDA

# Add the directory containing the source files
add_subdirectory(src)
