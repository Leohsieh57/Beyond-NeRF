cmake_minimum_required(VERSION 2.8.3)
project(minkindr)

# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

find_package(Threads)

# add_library(${PROJECT_NAME}
#   test/test-main.cc
#   test/test-rotation.cc
#   test/test-sim3.cc
#   test/test-transformation.cc
#   test/test-transformation-2d.cc
# )
# target_link_libraries(
#   ${PROJECT_NAME} 
#   # ${catkin_EXPORTED_TARGETS}
#   ${catkin_LIBRARIES} 
#   ${CMAKE_THREAD_LIBS_INIT}
#   Eigen3::Eigen
# )

# Add all files to show up in QtCreator.
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

# cs_install()
# cs_export()
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME} 

  CATKIN_DEPENDS
  roscpp
  eigen_conversions
)
