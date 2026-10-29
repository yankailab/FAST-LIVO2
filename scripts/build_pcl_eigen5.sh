#!/usr/bin/env bash
# Build the PCL modules used by FAST-LIVO2 against the selected Eigen 5.x.
# Eigen 5 still exports Eigen3Config.cmake and the Eigen3::Eigen target. PCL
# 1.15.1's request for Eigen 3.3 must therefore be changed to Eigen 5 in both
# its build and exported package configuration. Only a copied source is patched.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: build_pcl_eigen5.sh SOURCE_DIR BUILD_DIR INSTALL_PREFIX EIGEN_CMAKE_DIR [extra CMake options...]

SOURCE_DIR must contain pristine PCL 1.15.1 source. The source is copied to
BUILD_DIR/source and built in BUILD_DIR/cmake. EIGEN_CMAKE_DIR is the directory
containing the selected Eigen 5.x installation's Eigen3Config.cmake.

Builds shared Release libraries with C++17, then installs to INSTALL_PREFIX.
No downloads or sudo are performed. Use a writable prefix for staging.
CMAKE_BUILD_PARALLEL_LEVEL controls build jobs (default: 2).
Additional CMake options override defaults, except the supplied source/build,
Eigen directory, and installation prefix. For example: -G Ninja

Example:
  scripts/build_pcl_eigen5.sh /path/to/pcl-1.15.1 build/pcl-eigen5 \
    "$PWD/build/pcl-eigen5-install" /usr/local/share/eigen3/cmake
EOF
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  usage
  exit 0
fi
if (( $# < 4 )); then
  usage >&2
  exit 1
fi

command -v cmake >/dev/null || fail 'cmake is required.'
command -v realpath >/dev/null || fail 'realpath (coreutils) is required.'

source_dir=$(realpath -e -- "$1")
build_dir=$(realpath -m -- "$2")
install_prefix=$(realpath -m -- "$3")
eigen_cmake_dir=$(realpath -e -- "$4")
shift 4

[[ -d "$source_dir" ]] || fail 'SOURCE_DIR must be a directory.'
[[ -f "$eigen_cmake_dir/Eigen3Config.cmake" ]] ||
  fail 'EIGEN_CMAKE_DIR must contain Eigen3Config.cmake from Eigen 5.x.'
case "$build_dir/" in
  "$source_dir/"*) fail 'BUILD_DIR must be outside SOURCE_DIR.' ;;
esac
case "$source_dir/" in
  "$build_dir/"*) fail 'SOURCE_DIR must be outside BUILD_DIR.' ;;
esac
case "$install_prefix/" in
  "$source_dir/"*) fail 'INSTALL_PREFIX must be outside SOURCE_DIR.' ;;
esac
[[ ! -L "$build_dir/source" && ! -L "$build_dir/cmake" ]] ||
  fail 'BUILD_DIR/source and BUILD_DIR/cmake must not be symbolic links.'

build_jobs=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
[[ "$build_jobs" =~ ^[1-9][0-9]*$ ]] ||
  fail 'CMAKE_BUILD_PARALLEL_LEVEL must be a positive integer.'

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cmake \
  "-DPCL_SOURCE_DIR=$source_dir" \
  "-DPCL_PREPARED_SOURCE_DIR=$build_dir/source" \
  "-DEIGEN_CMAKE_DIR=$eigen_cmake_dir" \
  -P "$script_dir/../cmake/PreparePclEigen5.cmake"

# Keep the default CPU ABI consistent with FAST-LIVO2. PCL's automatic AVX
# selection can change Eigen allocation/alignment rules across the boundary.
# Full point-type instantiations include FAST-LIVO2's PointXYZINormal.
cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DPCL_SHARED_LIBS=ON \
  -DPCL_ONLY_CORE_POINT_TYPES=OFF \
  -DPCL_NO_PRECOMPILE=OFF \
  -DPCL_ENABLE_SSE=OFF \
  -DPCL_ENABLE_AVX=OFF \
  -DPCL_ENABLE_MARCHNATIVE=OFF \
  -DBUILD_common=ON \
  -DBUILD_filters=ON \
  -DBUILD_search=ON \
  -DBUILD_kdtree=ON \
  -DBUILD_octree=ON \
  -DBUILD_sample_consensus=ON \
  -DBUILD_2d=OFF \
  -DBUILD_apps=OFF \
  -DBUILD_benchmarks=OFF \
  -DBUILD_CUDA=OFF \
  -DBUILD_examples=OFF \
  -DBUILD_features=OFF \
  -DBUILD_geometry=OFF \
  -DBUILD_global_tests=OFF \
  -DBUILD_GPU=OFF \
  -DBUILD_io=OFF \
  -DBUILD_keypoints=OFF \
  -DBUILD_ml=OFF \
  -DBUILD_outofcore=OFF \
  -DBUILD_people=OFF \
  -DBUILD_recognition=OFF \
  -DBUILD_registration=OFF \
  -DBUILD_segmentation=OFF \
  -DBUILD_simulation=OFF \
  -DBUILD_stereo=OFF \
  -DBUILD_surface=OFF \
  -DBUILD_tools=OFF \
  -DBUILD_tracking=OFF \
  -DBUILD_visualization=OFF \
  -DWITH_CUDA=OFF \
  -DWITH_DOCS=OFF \
  -DWITH_OPENNI=OFF \
  -DWITH_OPENNI2=OFF \
  -DWITH_ENSENSO=OFF \
  -DWITH_DAVIDSDK=OFF \
  -DWITH_DSSDK=OFF \
  -DWITH_RSSDK=OFF \
  -DWITH_RSSDK2=OFF \
  -DWITH_LIBUSB=OFF \
  -DWITH_PNG=OFF \
  -DWITH_QHULL=OFF \
  -DWITH_GLEW=OFF \
  -DWITH_VTK=OFF \
  -DWITH_QT=NO \
  -DWITH_PCAP=OFF \
  -DWITH_OPENGL=OFF \
  -DWITH_SYSTEM_CJSON=OFF \
  "$@" \
  "-DEigen3_DIR=$eigen_cmake_dir" \
  "-DCMAKE_INSTALL_PREFIX=$install_prefix" \
  -S "$build_dir/source" -B "$build_dir/cmake"

cmake --build "$build_dir/cmake" --config Release --parallel "$build_jobs"
cmake --install "$build_dir/cmake" --config Release --prefix "$install_prefix"

printf '\nPCL built and installed. Configure FAST-LIVO2 with:\n'
printf '  -DPCL_DIR=%q\n' "$install_prefix/share/pcl-1.15"
printf '  -DEigen3_DIR=%q\n' "$eigen_cmake_dir"
