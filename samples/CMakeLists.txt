cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(perception)

# set build type to release
set(CMAKE_BUILD_TYPE "Release")

# explicitly set c++11
set(CMAKE_CXX_STANDARD 11)

# configure CUDA
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# configure PCL
find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# configure OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})

# check for VOXEL_LIB_DIR environment variable
IF(NOT DEFINED ENV{VOXEL_LIB_DIR})
    MESSAGE(SEND_ERROR "VOXEL_LIB_DIR not defined")
ENDIF(NOT DEFINED ENV{VOXEL_LIB_DIR})

# check for VOXEL_INCLUDE_DIR environment variable
IF(NOT DEFINED ENV{VOXEL_INCLUDE_DIR})
    MESSAGE(SEND_ERROR "VOXEL_INCLUDE_DIR not defined")
ENDIF(NOT DEFINED ENV{VOXEL_INCLUDE_DIR})

# load additional include and link directories
include_directories($ENV{VOXEL_INCLUDE_DIR})
link_directories($ENV{VOXEL_LIB_DIR})

# build voxel samples
add_executable(voxel_read voxel_read.cpp)
target_link_libraries(voxel_read ${PCL_LIBRARIES} voxel)
add_executable(voxel_save voxel_save.cpp)
target_link_libraries(voxel_save ${PCL_LIBRARIES} ${OpenCV_LIBS} voxel)
add_executable(voxel_viewer voxel_viewer.cpp)
target_link_libraries(voxel_viewer ${PCL_LIBRARIES} ${OpenCV_LIBS} voxel)
