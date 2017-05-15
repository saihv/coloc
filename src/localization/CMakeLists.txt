file(
  GLOB_RECURSE
  loc_files_header
  *.hpp
)
file(
  GLOB_RECURSE
  loc_files_cpp
  *.cpp
)

set(OpenMVG_DIR "D:/Github/openMVG_install/share/openMVG/cmake/")

find_package (OpenMVG REQUIRED)

include_directories(
    ${OPENMVG_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)


add_library(coloc_loc ${loc_files_header} ${loc_files_cpp})
target_link_libraries(coloc_loc
  openMVG_multiview
  stlplus
  ${CERES_LIBRARIES}
  openMVG_lInftyComputerVision
  openMVG_system
  openMVG_matching
)

target_include_directories(
  coloc_loc
  PRIVATE
  ${CERES_INCLUDE_DIRS})
