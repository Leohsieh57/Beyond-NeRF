cmake_minimum_required(VERSION 3.8)

project(bnerf_utils)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
add_definitions("-D SOPHUS_USE_BASIC_LOGGING")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Glog REQUIRED)
find_package(Sophus REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pcl_ros
    sensor_msgs
    eigen_conversions
    tf2_ros
    bnerf_msgs
    visualization_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

file(GLOB CPP_FILES "src/*.cpp")
foreach(file ${CPP_FILES})
    get_filename_component(FNAME ${file} NAME)
    set(SRC_FILES ${SRC_FILES} src/${FNAME})
endforeach()

add_library(
    ${PROJECT_NAME}
    ${SRC_FILES}
)

add_dependencies(
    ${PROJECT_NAME} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${GLOG_LIBRARY}
    ${Sophus_LIBRARIES}
    ${catkin_LIBRARIES}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 

    CFG_EXTRAS
    utils-extras.cmake

    CATKIN_DEPENDS
    pcl_ros
    bnerf_msgs
    visualization_msgs
)
