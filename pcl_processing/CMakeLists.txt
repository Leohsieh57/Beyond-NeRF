cmake_minimum_required(VERSION 3.0.2)
project(pcl_processing)


find_package(catkin REQUIRED COMPONENTS
  rospy
  pcl_ros
  sensor_msgs
  std_srvs
  message_generation
  std_msgs
)

add_service_files(
  FILES
  pcl_srv.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_srvs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy pcl_ros sensor_msgs std_srvs message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

