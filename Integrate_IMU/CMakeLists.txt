cmake_minimum_required(VERSION 3.10)
project(IMU_Integrate)

# Find the GTSAM package
find_package(GTSAM REQUIRED)

# Add your source files
add_executable(IMU_Integrate IMU_Integrate.cpp)

# Link the GTSAM library to your executable
target_link_libraries(IMU_Integrate PRIVATE gtsam)
