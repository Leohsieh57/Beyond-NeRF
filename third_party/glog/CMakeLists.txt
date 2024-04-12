cmake_minimum_required(VERSION 2.8.3)
project(glog)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)


set(LIBS ${LIBS} ${GLOG_LIBRARY})

if (GFLAGS_USE_TARGET_NAMESPACE)
  if (TARGET gflags::gflags)
    #target_link_libraries(${PROJECT_NAME} gflags::gflags)
    set(LIBS ${LIBS} gflags::gflags)
  else ()
    message(FATAL_ERROR "gflags::gflags target not defined")
  endif ()
else ()
  if (TARGET gflags)
    #target_link_libraries(${PROJECT_NAME} gflags)
    set(LIBS ${LIBS} gflags)
  else ()
    message(FATAL_ERROR "gflags target not defined")
  endif ()
endif ()

catkin_package(
  INCLUDE_DIRS ${Glog_INCLUDE_DIR}
  LIBRARIES ${LIBS}
)