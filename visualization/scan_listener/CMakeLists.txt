cmake_minimum_required(VERSION 3.0.2)
project(scan_listener)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  bnerf_msgs/msg/GraphIterationStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(
  PROGRAMS scripts/your_python_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

