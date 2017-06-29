file(
  GLOB_RECURSE
  sfm_files_header
  *.hpp
)
file(
  GLOB_RECURSE
  sfm_files_cpp
  *.cpp
)

set(OpenMVG_DIR "/home/sai/software/installMVG/share/openMVG/cmake")

find_package (OpenMVG REQUIRED)

include_directories(
    ${OPENMVG_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_library(coloc_sfm ${sfm_files_header} ${sfm_files_cpp})
target_link_libraries(coloc_sfm
  openMVG_sfm
  openMVG_multiview
  stlplus
  ceres
  cxsparse
  glog
  openMVG_lInftyComputerVision
  openMVG_system
  openMVG_matching
)

target_include_directories(
  coloc_sfm
  PRIVATE
  ${CERES_INCLUDE_DIRS})
