CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(coloc C CXX)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

if(MSVC)
    set(OpenMVG_DIR "D:/Code/Libraries/openMVG_install/share/openMVG/cmake")
    set(PYTHON_LIBRARY "C:/Python27/libs/python27.lib")
    set(PYTHON_INCLUDE_DIR "C:/Python27/include;C:/Python27/Lib/site-packages/numpy/core/include")
endif(MSVC)

if(UNIX)
endif(UNIX)

find_package(OpenMVG REQUIRED)
find_package(Threads REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

include_directories(
    ${OPENMVG_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS} 
    ${CERES_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}    
)

file(
  GLOB_RECURSE
  coloc_headers
  src/*.hpp
)
file(
  GLOB_RECURSE
  coloc_files
  src/coloc.cpp
)

add_executable(coloc ${coloc_headers} ${coloc_files})
target_link_libraries(coloc
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_multiview
  openMVG_matching_image_collection
  openMVG_sfm
  openMVG_geometry
  ceres
  stlplus
  easyexif
  vlsift
  ${PYTHON_LIBRARIES})

add_executable(updatetest src/updatetest.cpp)
target_link_libraries(updatetest
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_multiview
  openMVG_matching_image_collection
  openMVG_sfm
  openMVG_geometry
  ceres
  stlplus
  easyexif
  vlsift
  ${PYTHON_LIBRARIES})