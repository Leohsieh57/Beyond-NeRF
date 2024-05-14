cmake_minimum_required(VERSION 3.8)

project(prior_factor_manager)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    bnerf_utils
    geodesy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
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
    ${catkin_LIBRARIES}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    bnerf_utils
)

add_executable(${PROJECT_NAME}_node main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})