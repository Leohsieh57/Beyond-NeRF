cmake_minimum_required(VERSION 3.8)

project(bnerf_msgs)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
)


set(SUBDIRS "srv" "msg")
foreach(DIR ${SUBDIRS})
    file(GLOB ${DIR}_TMPS ${DIR}/*.${DIR})
    foreach(file ${${DIR}_TMPS})
        get_filename_component(FNAME ${file} NAME)
        set(${DIR}_FILES ${${DIR}_FILES} ${FNAME})
    endforeach()
endforeach()


add_message_files(
    FILES
    ${msg_FILES}
)


add_service_files(
  FILES
  ${srv_FILES}
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
)