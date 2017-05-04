CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(colloc C CXX)

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

set(OpenMVG_DIR "D:/Github/openMVG_install/share/openMVG/cmake/")

find_package (OpenMVG REQUIRED)
find_package( Threads REQUIRED)

include_directories(
    ${OPENMVG_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(lister src/ImageListing.cpp)
target_link_libraries(lister
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  easyexif)

add_executable(detector src/ComputeFeatures.cpp)
target_link_libraries(detector
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_multiview
  openMVG_sfm
  stlplus
  vlsift)

add_executable(matcher src/ComputeMatches.cpp)
target_link_libraries(matcher
  openMVG_system
  openMVG_features
  openMVG_multiview
  openMVG_sfm
  openMVG_matching_image_collection
  stlplus)

add_executable(reconstructor src/Reconstruct.cpp)
target_link_libraries(reconstructor
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus)
