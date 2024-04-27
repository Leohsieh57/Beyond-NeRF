cmake_minimum_required(VERSION 3.0.2)
project(pcl_processing)


find_package(catkin REQUIRED COMPONENTS
  rospy
  pcl_ros
  sensor_msgs
  std_msgs
  bnerf_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy pcl_ros sensor_msgs bnerf_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

