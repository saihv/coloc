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

set(OpenMVG_DIR "C:/Code/Libraries/openMVG_install/share/openMVG/cmake/")

find_package (OpenMVG REQUIRED)

include_directories(
    ${OPENMVG_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)


add_library(coloc_sfm ${sfm_files_header} ${sfm_files_cpp})
target_link_libraries(coloc_sfm
  openMVG_multiview
  stlplus
  ceres
  glog
  cxsparse
  openMVG_lInftyComputerVision
  openMVG_system
  openMVG_matching
)

target_include_directories(
  coloc_sfm
  PRIVATE
  ${CERES_INCLUDE_DIRS})
