cmake_minimum_required(VERSION 3.8)

project(dust3r)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    rospy
    bnerf_utils
)

catkin_package(
    CATKIN_DEPENDS
    bnerf_utils
)