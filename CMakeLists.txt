cmake_minimum_required(VERSION 3.16)

# Keep the upstream catkin application available without making ROS a library
# dependency. Select this explicitly when building in a ROS 1 workspace.
option(FAST_LIVO2_BUILD_ROS "Build the original ROS 1 application instead of the standalone library" OFF)
if(FAST_LIVO2_BUILD_ROS)
  include(cmake/ROS.cmake)
  return()
endif()

project(fast_livo2 VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
include(CTest)

option(FAST_LIVO2_BUILD_EXAMPLES "Build the standalone synthetic sensor example" ON)
set(FAST_LIVO2_NUM_THREADS 4 CACHE STRING "Maximum OpenMP threads used by the estimator")
if(NOT FAST_LIVO2_NUM_THREADS MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "FAST_LIVO2_NUM_THREADS must be a positive integer")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Eigen 5 retains the Eigen3 package/target names. Select the intended source
# installations with Eigen3_DIR and OpenCV_DIR; never substitute Eigen 3/OpenCV 4.
find_package(Eigen3 5 REQUIRED CONFIG)
if(Eigen3_VERSION VERSION_LESS 5 OR NOT Eigen3_VERSION VERSION_LESS 6)
  message(FATAL_ERROR "FAST-LIVO2 requires Eigen 5.x; selected ${Eigen3_VERSION}")
endif()
set(FAST_LIVO2_EIGEN_VERSION "${Eigen3_VERSION}")
get_filename_component(_fast_livo2_eigen_config_dir "${Eigen3_DIR}" REALPATH)
find_package(OpenCV 5 REQUIRED CONFIG COMPONENTS core imgproc geometry)
if(NOT OpenCV_VERSION_MAJOR EQUAL 5)
  message(FATAL_ERROR "FAST-LIVO2 requires OpenCV 5.x; selected ${OpenCV_VERSION}")
endif()
find_package(PCL 1.8 REQUIRED CONFIG COMPONENTS common filters)
# Older PCL packages request Eigen 3.3 even if an Eigen 5 target already exists.
# Reject their fallback rather than mixing Eigen versions and header paths.
get_filename_component(_fast_livo2_eigen_config_after_pcl "${Eigen3_DIR}" REALPATH)
if(NOT Eigen3_VERSION VERSION_EQUAL FAST_LIVO2_EIGEN_VERSION OR
   NOT _fast_livo2_eigen_config_after_pcl STREQUAL _fast_livo2_eigen_config_dir)
  message(FATAL_ERROR
    "PCL changed the selected Eigen ${FAST_LIVO2_EIGEN_VERSION} package "
    "(${_fast_livo2_eigen_config_dir}) to ${Eigen3_DIR}. "
    "Build PCL against Eigen 5 using scripts/build_pcl_eigen5.sh and select its PCL_DIR.")
endif()
message(STATUS "FAST-LIVO2 Eigen ${FAST_LIVO2_EIGEN_VERSION}: ${Eigen3_DIR}")
message(STATUS "FAST-LIVO2 OpenCV ${OpenCV_VERSION}: ${OpenCV_DIR}")
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(Threads REQUIRED)

add_library(fast_livo2 SHARED
  src/IMU_Processing.cpp
  src/voxel_map.cpp
  src/vio.cpp
  src/frame.cpp
  src/visual_point.cpp
  src/standalone/vision.cpp
  src/standalone/slam.cpp
)
add_library(fast_livo2::fast_livo2 ALIAS fast_livo2)
generate_export_header(fast_livo2
  EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/include/fast_livo2/export.hpp"
)
target_compile_features(fast_livo2 PUBLIC cxx_std_17)
set_target_properties(fast_livo2 PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  CXX_EXTENSIONS OFF
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)
target_compile_definitions(fast_livo2 PRIVATE
  FAST_LIVO2_STANDALONE
  MP_PROC_NUM=${FAST_LIVO2_NUM_THREADS}
  ROOT_DIR=""
)
if(FAST_LIVO2_NUM_THREADS GREATER 1)
  target_compile_definitions(fast_livo2 PRIVATE MP_EN)
endif()
target_include_directories(fast_livo2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_include_directories(fast_livo2 SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(fast_livo2 PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(fast_livo2
  PUBLIC Eigen3::Eigen opencv_core
  PRIVATE
    opencv_imgproc opencv_geometry
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX Threads::Threads
)

if(FAST_LIVO2_BUILD_EXAMPLES)
  add_executable(fast_livo2_example examples/standalone.cpp)
  target_link_libraries(fast_livo2_example PRIVATE fast_livo2::fast_livo2)
endif()

if(BUILD_TESTING)
  add_executable(fast_livo2_standalone_test tests/standalone_test.cpp)
  target_link_libraries(fast_livo2_standalone_test PRIVATE fast_livo2::fast_livo2)
  add_test(NAME fast_livo2.standalone COMMAND fast_livo2_standalone_test)
  add_executable(fast_livo2_vision_test tests/standalone/vision_helpers.cpp src/standalone/vision.cpp)
  target_compile_features(fast_livo2_vision_test PRIVATE cxx_std_17)
  target_include_directories(fast_livo2_vision_test PRIVATE include)
  target_link_libraries(fast_livo2_vision_test PRIVATE
    Eigen3::Eigen opencv_core opencv_imgproc opencv_geometry)
  add_test(NAME fast_livo2.vision_helpers COMMAND fast_livo2_vision_test)
endif()

install(TARGETS fast_livo2 EXPORT fast_livo2Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES include/fast_livo2/slam.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fast_livo2)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/fast_livo2/export.hpp"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fast_livo2)
install(FILES LICENSE docs/Standalone.md DESTINATION ${CMAKE_INSTALL_DOCDIR})

set(fast_livo2_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/fast_livo2")
configure_package_config_file(cmake/fast_livo2Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/fast_livo2Config.cmake"
  INSTALL_DESTINATION "${fast_livo2_INSTALL_CMAKEDIR}"
)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/fast_livo2ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(EXPORT fast_livo2Targets NAMESPACE fast_livo2::
  DESTINATION "${fast_livo2_INSTALL_CMAKEDIR}"
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/fast_livo2Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/fast_livo2ConfigVersion.cmake"
  DESTINATION "${fast_livo2_INSTALL_CMAKEDIR}"
)
