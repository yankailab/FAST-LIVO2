# Used by scripts/build_pcl_eigen5.sh. Patch a copy of official PCL 1.15.1;
# never modify the caller's original checkout or change arbitrary PCL versions.
cmake_minimum_required(VERSION 3.16.3)

if(NOT IS_DIRECTORY "${PCL_SOURCE_DIR}" OR
   NOT DEFINED PCL_PREPARED_SOURCE_DIR OR PCL_PREPARED_SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "PCL_SOURCE_DIR and PCL_PREPARED_SOURCE_DIR are required.")
endif()

# Check the explicitly selected directory before find_package can fall back to
# another installation. A version file is safe to evaluate in CMake script mode;
# the package configuration itself creates imported targets and is not.
function(check_selected_eigen5)
  if(NOT EXISTS "${EIGEN_CMAKE_DIR}/Eigen3ConfigVersion.cmake")
    message(FATAL_ERROR "EIGEN_CMAKE_DIR must contain Eigen3ConfigVersion.cmake.")
  endif()
  set(PACKAGE_FIND_VERSION "5")
  set(PACKAGE_FIND_VERSION_MAJOR 5)
  set(PACKAGE_FIND_VERSION_MINOR 0)
  set(PACKAGE_FIND_VERSION_PATCH 0)
  set(PACKAGE_FIND_VERSION_TWEAK 0)
  set(PACKAGE_FIND_VERSION_COUNT 1)
  include("${EIGEN_CMAKE_DIR}/Eigen3ConfigVersion.cmake")
  if(NOT PACKAGE_VERSION_COMPATIBLE OR PACKAGE_VERSION VERSION_LESS "5" OR
     PACKAGE_VERSION VERSION_GREATER_EQUAL "6")
    message(FATAL_ERROR
      "EIGEN_CMAKE_DIR selects Eigen ${PACKAGE_VERSION}; Eigen 5.x is required.")
  endif()
  message(STATUS "Selected Eigen ${PACKAGE_VERSION} in ${EIGEN_CMAKE_DIR}")
endfunction()
check_selected_eigen5()

get_filename_component(pcl_source_path "${PCL_SOURCE_DIR}" REALPATH)
get_filename_component(pcl_prepared_path "${PCL_PREPARED_SOURCE_DIR}" REALPATH)
string(FIND "${pcl_prepared_path}/" "${pcl_source_path}/" pcl_source_overlap)
string(FIND "${pcl_source_path}/" "${pcl_prepared_path}/" pcl_prepared_overlap)
if(pcl_source_overlap EQUAL 0 OR pcl_prepared_overlap EQUAL 0)
  message(FATAL_ERROR "Original and prepared PCL source directories must not overlap.")
endif()

file(READ "${PCL_SOURCE_DIR}/CMakeLists.txt" pcl_root_cmake)
string(FIND "${pcl_root_cmake}" "project(PCL VERSION 1.15.1)" pcl_version_position)
if(pcl_version_position EQUAL -1)
  message(FATAL_ERROR "This helper requires official PCL 1.15.1 source.")
endif()

# Validate both expected replacements before copying or modifying any files.
foreach(pcl_file IN ITEMS CMakeLists.txt PCLConfig.cmake.in)
  file(READ "${PCL_SOURCE_DIR}/${pcl_file}" pcl_contents)
  string(REGEX MATCHALL "find_package\\(Eigen3 3\\.3 REQUIRED NO_MODULE\\)"
    pcl_eigen_requests "${pcl_contents}")
  list(LENGTH pcl_eigen_requests pcl_eigen_request_count)
  if(NOT pcl_eigen_request_count EQUAL 1)
    message(FATAL_ERROR
      "Expected exactly one original Eigen 3.3 request in ${pcl_file}. "
      "Use pristine official PCL 1.15.1 source.")
  endif()
endforeach()

file(COPY "${PCL_SOURCE_DIR}/" DESTINATION "${PCL_PREPARED_SOURCE_DIR}")
foreach(pcl_file IN ITEMS CMakeLists.txt PCLConfig.cmake.in)
  file(READ "${PCL_PREPARED_SOURCE_DIR}/${pcl_file}" pcl_contents)
  # Eigen 5 intentionally retains the Eigen3 package and imported target names.
  string(REPLACE "find_package(Eigen3 3.3 REQUIRED NO_MODULE)"
    "find_package(Eigen3 5 REQUIRED NO_MODULE)" pcl_contents "${pcl_contents}")
  file(WRITE "${PCL_PREPARED_SOURCE_DIR}/${pcl_file}" "${pcl_contents}")
endforeach()
message(STATUS "Prepared PCL 1.15.1 with Eigen 5 in ${PCL_PREPARED_SOURCE_DIR}")
