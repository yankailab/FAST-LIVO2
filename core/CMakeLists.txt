cmake_minimum_required(VERSION 3.22)
project(fastlivo2_core LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FASTLIVO2_ROOT ${CMAKE_CURRENT_LIST_DIR}/..) # points to FAST-LIVO2 repo root

# ---- deps ----
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(MPI REQUIRED)
#find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
#find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(yaml-cpp REQUIRED)
find_package(Sophus REQUIRED) # should find your installed non-templated Sophus

# ---- Sophus target/vars compatibility ----
set(SOPHUS_LINK "")
if(TARGET Sophus::Sophus)
  set(SOPHUS_LINK Sophus::Sophus)
elseif(TARGET Sophus::sophus)
  set(SOPHUS_LINK Sophus::sophus)
elseif(DEFINED Sophus_LIBRARIES)
  set(SOPHUS_LINK ${Sophus_LIBRARIES})
elseif(DEFINED SOPHUS_LIBRARIES)
  set(SOPHUS_LINK ${SOPHUS_LIBRARIES})
else()
  message(FATAL_ERROR "Sophus found but no target or Sophus_LIBRARIES exported. Check Sophus install / Sophus_DIR.")
endif()


# ---- fetch/build vikit_common (no ROS) ----
include(FetchContent)

FetchContent_Declare(rpg_vikit
  GIT_REPOSITORY https://github.com/xuankuzcr/rpg_vikit.git
  GIT_TAG master
)

FetchContent_GetProperties(rpg_vikit)
if(NOT rpg_vikit_POPULATED)
  FetchContent_Populate(rpg_vikit)

  # ---- Patch vikit_common to NOT force USE_ROS TRUE ----
  set(_vk_cmake "${rpg_vikit_SOURCE_DIR}/vikit_common/CMakeLists.txt")
  file(READ "${_vk_cmake}" _vk_txt)

  # vikit_common does: SET(USE_ROS TRUE)  (hard-coded)  :contentReference[oaicite:2]{index=2}
  if(_vk_txt MATCHES "SET\\(USE_ROS TRUE\\)")
    string(REPLACE
      "SET(USE_ROS TRUE)"
      "option(USE_ROS \"Build vikit_common with ROS/catkin\" OFF)"
      _vk_txt
      "${_vk_txt}"
    )
    file(WRITE "${_vk_cmake}" "${_vk_txt}")
  endif()
endif()

# Ensure it's OFF in cache (now that vikit_common uses option())
set(USE_ROS OFF CACHE BOOL "Build vikit_common with ROS/catkin" FORCE)

add_subdirectory("${rpg_vikit_SOURCE_DIR}/vikit_common"
  "${rpg_vikit_BINARY_DIR}/vikit_common")


# ---- API .so ----
add_library(fastlivo2_core SHARED
  src/FastLivo2Core.cpp
  src/Engine.cpp
)

include(GNUInstallDirs)

target_include_directories(fastlivo2_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
  ${FASTLIVO2_ROOT}/include
  ${rpg_vikit_SOURCE_DIR}/vikit_common/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(fastlivo2_core
  PRIVATE Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  Boost::thread
  yaml-cpp::yaml-cpp
  vikit_common
  ${SOPHUS_LINK}
)
# fastlivo2_core is a shared lib; consumers don't need to link its private deps.
# Make sure the exported target has NO requirement on internal targets like vikit_common.
#set_property(TARGET fastlivo2_core PROPERTY INTERFACE_LINK_LIBRARIES "")
#set_property(TARGET fastlivo2_core PROPERTY INTERFACE_LINK_LIBRARIES_DIRECT "")
#set_property(TARGET fastlivo2_core PROPERTY INTERFACE_LINK_LIBRARIES_DIRECT_EXCLUDE "")



#target_compile_definitions(fastlivo2_core PRIVATE FASTLIVO2_CORE_BUILD)
target_compile_definitions(fastlivo2_core
  PRIVATE
  FASTLIVO2_STANDALONE=1
  ROOT_DIR="${FASTLIVO2_ROOT}/"
)

set_target_properties(fastlivo2_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME "fastlivo2_core"
  VERSION 0.2.0
  SOVERSION 0
)

# ---- upstream FAST-LIVO2 algorithm (Phase2.0) ----
# We compile the ROS-free-ish modules first. We intentionally DO NOT compile:
#  - src/LIVMapper.cpp (ROS node/orchestration) :contentReference[oaicite:7]{index=7}
#  - src/preprocess.cpp (ROS msgs + livox_ros_driver::CustomMsg) :contentReference[oaicite:8]{index=8}
add_library(fastlivo2_upstream OBJECT
  ${FASTLIVO2_ROOT}/src/vio.cpp
  ${FASTLIVO2_ROOT}/src/frame.cpp
  ${FASTLIVO2_ROOT}/src/visual_point.cpp
  ${FASTLIVO2_ROOT}/src/voxel_map.cpp
  ${FASTLIVO2_ROOT}/src/IMU_Processing.cpp
)

set_property(TARGET fastlivo2_upstream PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(fastlivo2_upstream
  PRIVATE
  ${FASTLIVO2_ROOT}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rpg_vikit_SOURCE_DIR}/vikit_common/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Provide ROOT_DIR like upstream does (used by their DEBUG_FILE_DIR macro, etc.) :contentReference[oaicite:9]{index=9}
target_compile_definitions(fastlivo2_upstream
  PRIVATE
  FASTLIVO2_STANDALONE=1
  ROOT_DIR="${FASTLIVO2_ROOT}/"
)

target_link_libraries(fastlivo2_upstream
  PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} Boost::thread vikit_common ${SOPHUS_LINK}
)

# Link upstream object code into your .so
target_sources(fastlivo2_core PRIVATE $<TARGET_OBJECTS:fastlivo2_upstream>)



#if(NOT TARGET Sophus::Sophus AND NOT TARGET Sophus::sophus)
#  if(DEFINED Sophus_INCLUDE_DIRS)
#    target_include_directories(fastlivo2_core PRIVATE ${Sophus_INCLUDE_DIRS})
#    target_include_directories(fastlivo2_upstream PRIVATE ${Sophus_INCLUDE_DIRS})
#  endif()
#endif()


# ---- install / export ----
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Give the target a versioned soname if you haven’t already
# (adjust PROJECT_VERSION if your project() doesn’t set it)
if(NOT PROJECT_VERSION)
  set(PROJECT_VERSION "0.2.0")
endif()

set_target_properties(fastlivo2_core PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 0
  EXPORT_NAME fastlivo2_core
)

# Install the shared library
install(TARGETS fastlivo2_core
  EXPORT fastlivo2_coreTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# vikit_common is built via FetchContent; fastlivo2_core links against it.
# Export it too so install(EXPORT ...) is happy.
if(TARGET vikit_common)
  install(TARGETS vikit_common
    EXPORT fastlivo2_coreTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  # Install vikit headers (so downstream can include vikit/* if needed)
  install(DIRECTORY ${rpg_vikit_SOURCE_DIR}/vikit_common/include/vikit
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
endif()



# Install public headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/fastlivo2_core
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export targets for find_package(fastlivo2_core)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/fastlivo2_core)

install(EXPORT fastlivo2_coreTargets
  FILE fastlivo2_coreTargets.cmake
  NAMESPACE fastlivo2_core::
  DESTINATION ${INSTALL_CONFIGDIR}
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/fastlivo2_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/fastlivo2_coreConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/fastlivo2_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/fastlivo2_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/fastlivo2_coreConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

