cmake_minimum_required(VERSION 2.8.3)
project(kitti_to_rosbag)

# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# find_package(Glog REQUIRED)
# find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS 
  core
  highgui
  imgcodecs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  eigen_conversions
  tf_conversions
  minkindr
  cv_bridge
  image_transport
  tf
  glog
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  minkindr
  glog
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/kitti_parser.cpp
  src/kitti_ros_conversions.cpp
  src/kitti_common.cpp
)

add_dependencies(
  ${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLOG_LIBRARY}
)

############
# BINARIES #
############
add_executable(kitti_live_node
  src/kitti_live_node.cpp
)
target_link_libraries(kitti_live_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(kitti_rosbag_converter
  src/kitti_rosbag_converter.cpp
)
target_link_libraries(kitti_rosbag_converter ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(tiny_disp_view
  src/tiny_disp_view.cpp
)
target_link_libraries(tiny_disp_view ${PROJECT_NAME} ${OpenCV_LIBRARIES})
