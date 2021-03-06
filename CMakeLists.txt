cmake_minimum_required(VERSION 2.8.3)
project(coloc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_CUDA "Enable CUDA features" OFF)
option(USE_ROS "Enable ROS integration" OFF)
option(USE_STREAM "Enable ROS image streaming" OFF)

if(UNIX)
  #add_definitions(-DUSE_ROS)
  set(OpenMVG_DIR "/home/sai/lib/openMVG/install/share/openMVG/cmake")
endif(UNIX)

if(MSVC)
  #set(OpenMVG_DIR "D:/Code/Libraries/openMVG_install/share/openMVG/cmake")
  #set(OpenCV_DIR "D:/Code/Libraries/opencv-3.4.0/opencv-3.4.0/build")
  set(OpenMVG_DIR "C:/Code/Libraries/openMVG/install_release/share/openMVG/cmake")
  set(OpenCV_DIR "C:/Code/Libraries/opencv/install_release")
endif(MSVC)

add_subdirectory(lib/dlib dlib_build)

FIND_PACKAGE(OpenMVG REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)


if(USE_CUDA)
  add_definitions(-DUSE_CUDA)
  FIND_PACKAGE(CUDA REQUIRED) 
  SET(CUDA_NVCC_FLAGS "-arch=sm_50" CACHE STRING "nvcc flags" FORCE)
  SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
  SET(LIB_TYPE STATIC) 
  include_directories(include)
  CUDA_ADD_LIBRARY(koral ${LIB_TYPE} src/CUDALERP.cu src/CLATCH.cu src/CUDAK2NN.cu)
endif(USE_CUDA)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -march=native -fpermissive -mavx2 -ftree-vectorize -funroll-all-loops")

include_directories(
if(MSVC)
  include
endif(MSVC)
  ${OPENMVG_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

if(MSVC)
  file(
    GLOB_RECURSE
    coloc_headers
    include/coloc/*.hpp
  )
  file(
  GLOB_RECURSE
  coloc_files
  src/coloc_node.cpp
  )

  add_executable(coloc ${coloc_headers} ${coloc_files})
endif(MSVC)

if(UNIX)
  add_executable(coloc src/coloc_node.cpp)
endif(UNIX)

target_link_libraries(coloc dlib::dlib)

if(USE_CUDA)
  add_dependencies(coloc koral)
endif(USE_CUDA)

target_link_libraries(coloc
  ${OpenCV_LIBS}
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_multiview
  openMVG_matching
  openMVG_matching_image_collection
  openMVG_sfm
  openMVG_geometry
  ceres
  stlplus
  easyexif
  vlsift
  dlib::dlib
 )

if(USE_CUDA)
  target_link_libraries(coloc koral)
endif(USE_CUDA)
