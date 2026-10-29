# FAST-LIVO2 standalone C++ library

The default build produces a shared `libfast_livo2` library with a C++17 API in
[`fast_livo2/slam.hpp`](../include/fast_livo2/slam.hpp). It runs the existing IMU
propagation, LiDAR deskewing, voxel plane mapping, LiDAR state update, and direct
visual state update without ROS. Camera fusion is optional. There is no viewer,
ROS message conversion, sensor driver, or background processing thread.

This is FAST-LIVO2 odometry and local mapping. It does not add loop closure,
global pose graph optimization, map relocalization, or a saved-map localization
mode. `saveMap()` exports geometry, not a restorable estimator checkpoint.

## Dependencies

- CMake 3.16.3 or newer and a C++17 compiler (including the PCL source build).
- Eigen 5.x. This system's source installation is Eigen **5.0.1**.
  Eigen retains the CMake package name `Eigen3`, target `Eigen3::Eigen`, and
  installation directory name `eigen3` in version 5.
- A source-built OpenCV **5.x** installation with `core`, `imgproc`, and
  `geometry`. This system provides OpenCV **5.0.0**. The `geometry` module
  replaces the `calib3d` module used by the older OpenCV API.
- PCL built against the same Eigen 5 installation: `common` and `filters`,
  with their dependencies (including Boost and FLANN). The helper below uses
  PCL **1.15.1**. No PCL visualization component is requested.
- OpenMP and the platform threading library. GCC includes OpenMP support;
  Clang installations may need `libomp-dev`.

On Ubuntu/Debian, install the build tools and PCL's Boost/FLANN dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build git pkg-config \
  libboost-iostreams-dev libflann-dev liblz4-dev zlib1g-dev
```

Use the selected source installations for Eigen and OpenCV, rather than the
distribution's Eigen/OpenCV development packages. The paths on this system are:

| Dependency | Version | CMake package directory |
| --- | --- | --- |
| Eigen | 5.0.1 | `/usr/local/share/eigen3/cmake` |
| OpenCV | 5.0.0 | `/usr/local/lib/cmake/opencv5` |

Pass `Eigen3_DIR` and `OpenCV_DIR` explicitly when configuring FAST-LIVO2 and
applications that link it. Change these paths to select another source build;
avoid combining headers and libraries from different installations. No ROS
distribution, catkin, `pcl_ros`, `cv_bridge`, Sophus, or vikit is required.

### Build PCL with Eigen 5

PCL also uses Eigen in its compiled interfaces. A distribution PCL binary built
with Eigen 3 must be rebuilt for this configuration. PCL 1.15.1 additionally
requests Eigen 3 in its CMake files; the helper below updates that dependency
request in a copy of the source and builds it using the selected Eigen 5
installation. It leaves the supplied PCL source tree unchanged.

From the FAST-LIVO2 repository root:

```bash
git clone --branch pcl-1.15.1 --depth 1 \
  https://github.com/PointCloudLibrary/pcl.git build/pcl-1.15.1
bash scripts/build_pcl_eigen5.sh \
  build/pcl-1.15.1 build/pcl-eigen5 "$PWD/build/pcl-eigen5-install" \
  /usr/local/share/eigen3/cmake
sudo cmake --install build/pcl-eigen5/cmake --prefix /usr/local
sudo ldconfig
```

If the PCL source is already available, pass its directory instead of cloning
it. [`build_pcl_eigen5.sh`](../scripts/build_pcl_eigen5.sh) installs to the writable
staging prefix before the separate system installation command. It performs no
downloads or privileged operations. Set `CMAKE_BUILD_PARALLEL_LEVEL=1` before
invoking the script if its default of two compiler jobs exceeds available memory.
Additional CMake options may follow the four required arguments.

For a private PCL installation, use a writable installation prefix in the third
argument and omit the two `sudo` commands. Set `PCL_DIR` to that prefix's
`share/pcl-1.15` directory in the commands below, and make its `lib` directory
available to the runtime loader, for example through `LD_LIBRARY_PATH`.

## Build and verify

From the repository root, use a fresh build directory so that an earlier Eigen 3
or distribution OpenCV selection is not retained in CMake's cache:

```bash
cmake -S . -B build/standalone-eigen5 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DEigen3_DIR=/usr/local/share/eigen3/cmake \
  -DOpenCV_DIR=/usr/local/lib/cmake/opencv5 \
  -DPCL_DIR=/usr/local/share/pcl-1.15
cmake --build build/standalone-eigen5 --parallel 2
cmake -E chdir build/standalone-eigen5 ctest --output-on-failure
./build/standalone-eigen5/fast_livo2_example
```

The example feeds a synthetic stationary IMU and three planar surfaces into the
real estimator. It checks the integration path without a dataset; it does not
measure real-world trajectory accuracy. Eigen-heavy translation units can use
substantial memory; reduce `--parallel` if the compiler runs out of memory.

Useful CMake options:

| Option | Default | Purpose |
| --- | --- | --- |
| `FAST_LIVO2_BUILD_ROS` | `OFF` | Select the original catkin application instead of the library. |
| `FAST_LIVO2_BUILD_EXAMPLES` | `ON` | Build the synthetic streaming example. |
| `BUILD_TESTING` | `ON` | Build and register standalone API/estimator checks. |
| `FAST_LIVO2_NUM_THREADS` | `4` | Maximum OpenMP threads used by estimator loops; use `1` for serial loops. |
| `CMAKE_INSTALL_PREFIX` | `/usr/local` | Standard installation prefix. |
| `Eigen3_DIR` | discovered | Directory containing the selected Eigen 5 `Eigen3Config.cmake`. |
| `OpenCV_DIR` | discovered | Directory containing the selected OpenCV 5 `OpenCVConfig.cmake`. |
| `PCL_DIR` | discovered | Directory containing the selected PCL `PCLConfig.cmake`. |
| `CMAKE_PREFIX_PATH` | unset | Prefixes containing nonstandard dependency installations. |

No CPU-specific `-march=native` or `-ffast-math` flags are forced by the standalone
build. The ROS build keeps its upstream compiler settings.

Verified on this system with Eigen 5.0.1 and OpenCV 5.0.0 from the `/usr/local`
installations above, plus PCL 1.15.1 rebuilt against Eigen 5. Both test suites
passed, covering LiDAR, IMU, and camera fusion. The example and a separate
application consuming a staged installation also built and ran successfully.
Configuration checks rejected Eigen 3.4 and a PCL package that tried to select
it. These checks use synthetic data; recorded-dataset accuracy was not measured.

## Install

Install to the default system prefix:

```bash
sudo cmake --install build/standalone-eigen5
sudo ldconfig
```

CMake's GNU installation directories place the public header under
`include/fast_livo2`, the versioned shared library under `lib` (or the platform's
library directory), the package configuration under `lib/cmake/fast_livo2`, and
this guide and the license under `share/doc/fast_livo2`.

For a user-owned prefix, configure it before building:

```bash
cmake -S . -B build/standalone-eigen5 \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local" \
  -DEigen3_DIR=/usr/local/share/eigen3/cmake \
  -DOpenCV_DIR=/usr/local/lib/cmake/opencv5 \
  -DPCL_DIR=/usr/local/share/pcl-1.15
cmake --build build/standalone-eigen5 --parallel 2
cmake --install build/standalone-eigen5
```

A private prefix must be discoverable both by CMake and the dynamic loader.
Pass `-DCMAKE_PREFIX_PATH="$HOME/.local"` to the consuming project; use its
appropriate runtime search path or `LD_LIBRARY_PATH` for private library
locations. PCL and OpenCV runtime libraries must remain installed.

## Link from another framework

The public interface uses standard C++ types, Eigen poses/vectors, and OpenCV
images. It does not expose PCL, ROS, or internal estimator classes.

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_slam_application LANGUAGES CXX)
find_package(fast_livo2 CONFIG REQUIRED)
add_executable(my_slam_application main.cpp)
target_link_libraries(my_slam_application PRIVATE fast_livo2::fast_livo2)
```

The imported target supplies C++17, include paths, Eigen, and OpenCV core. PCL is a
private implementation dependency of the shared library.

Configure the consuming project with the same dependency installations:

```bash
cmake -S /path/to/application -B /path/to/application/build \
  -DCMAKE_PREFIX_PATH=/usr/local \
  -DEigen3_DIR=/usr/local/share/eigen3/cmake \
  -DOpenCV_DIR=/usr/local/lib/cmake/opencv5
cmake --build /path/to/application/build --parallel 2
```

Use the FAST-LIVO2 installation prefix in `CMAKE_PREFIX_PATH` if it differs from
`/usr/local`. Because Eigen types and `cv::Mat` cross the library interface,
the installed CMake package requires the exact Eigen and OpenCV versions used
to build the library. Applications must also use compatible compiler options;
select the same dependency installations in the application and FAST-LIVO2.

Frameworks such as OpenKAI can own one `fast_livo2::Slam`, translate their input
packets into the following sample types, call `process()` on their worker
thread, and display returned poses and points using their own visualization.

```cpp
#include <fast_livo2/slam.hpp>

fast_livo2::Config config;
config.T_imu_lidar = Eigen::Isometry3d::Identity(); // set measured extrinsics
fast_livo2::Slam slam(config);

// Feed actual sensor samples in strictly increasing order within each stream:
// slam.insertImu(imu_sample);
// slam.insertLidar(lidar_scan);
// slam.insertImage(image_sample); // only when config.use_camera is true

const auto result = slam.process();
if (result.status == fast_livo2::Status::Updated) {
  const Eigen::Isometry3d world_from_lidar = result.estimate->T_world_lidar;
  const auto& world_points = result.estimate->scan_world;
  // Copy/share the immutable estimate into the application's pose/map viewer.
}
```

[`examples/standalone.cpp`](../examples/standalone.cpp) is a complete compilable
example. Configuration is supplied through `Config`; the wrapper does not parse
ROS YAML files or depend on a parameter server.

## Sensor conventions and synchronization

All streams use the same monotonic sensor clock in **seconds**. Apply sensor
clock offsets before insertion. Each individual stream must have strictly
increasing timestamps; LiDAR scan intervals must not overlap. Inputs are copied
on insertion, including image pixels, so the caller can reuse its buffers.

- `ImuSample`: acceleration in m/s² including gravity, angular velocity in rad/s,
  both expressed in the IMU frame. Keep the device stationary while the initial
  samples estimate gravity and the acceleration scale; bias estimation continues
  during subsequent filtering. A stationary Z-up IMU reads
  approximately `(0, 0, +9.81)` m/s². `imu_initialization_samples` controls this
  initialization, which can consume several scans before producing a pose.
- `LidarScan`: `timestamp` is scan **start**. XYZ are meters in the LiDAR frame;
  intensity is passed through. Every point carries a nonnegative `time_offset`
  in seconds from scan start. Convert driver-specific microsecond/nanosecond
  offsets before insertion. Zero offsets are suitable for an instantaneous
  depth frame; they cannot describe motion within a spinning scan. Range and
  stride filtering and voxel downsampling are configured by `Config`.
- `ImageSample`: timestamp is exposure time, with `CV_8UC1` grayscale or `CV_8UC3`
  BGR pixels at the configured camera resolution. `CameraConfig` supplies
  pinhole intrinsics and OpenCV distortion coefficients `(k1, k2, p1, p2, k3)`.
  Images are rectified internally before the direct visual update. Fisheye and
  other camera models must be rectified by the caller into a pinhole image with
  matching intrinsics and zero distortion coefficients. Set
  `use_camera=true` and provide an IMU stream to enable fusion.

Transforms use `T_destination_source`: `T_imu_lidar * p_lidar` produces IMU-frame
coordinates; `camera.T_camera_lidar * p_lidar` produces camera-frame coordinates.
Camera axes follow the OpenCV convention (X right, Y down, Z forward). Set both
extrinsics from calibration; identity defaults are useful for synthetic input,
not a substitute for calibration. World coordinates start at the initialized
IMU pose, optionally with gravity aligned to world Z.

`process()` is synchronous and performs at most one synchronized update. In
LiDAR/IMU mode it updates at scan end. In LiDAR/IMU/camera mode it splits buffered
scans at image times and updates at those times. Supply IMU samples bracketing
each update time, including a sample at or beyond it: the wrapper does not
extrapolate missing IMU data. Feed both continuing scans and images when camera
fusion is enabled.

| Status | Caller action |
| --- | --- |
| `WaitingForLidar` | Supply another LiDAR scan. |
| `WaitingForImu` | Supply IMU coverage through the next update time. |
| `WaitingForImage` | Supply a synchronized image and sufficient LiDAR coverage. |
| `Initializing` | The update was consumed for IMU initialization; continue feeding data. |
| `Updated` | Consume `result.estimate`. |
| `NoPoints` | A consumed update had no usable points; continue. |
| `Finished` | End-of-input processing has stopped. |

Call `process()` again while it returns `Updated`, `Initializing`, or `NoPoints`
to consume queued work. On a waiting status, feed the required inputs before
retrying. An external worker loop can use this behavior without callbacks or
library-managed scheduling.

`finish()` rejects subsequent insertions and allows `process()` to drain all
remaining synchronized updates. A tail that lacks IMU, LiDAR, or camera coverage
is not extrapolated; processing then returns `Finished`. Inspect
`pendingLidarScans()` and `pendingImages()` to detect buffered incomplete data.
`reset()` clears the state, maps, queues, and finish flag while retaining
configuration. Construct a new instance to change configuration.

Invalid configuration or input throws `std::invalid_argument`; full bounded
queues throw `std::length_error`. Keep processing as data arrives instead of
buffering an entire long recording. Serialize all calls on each instance;
there are no callbacks or internal queue mutexes. Separate instances own
separate estimator state.

## Estimates and map output

`Estimate` supplies IMU and LiDAR world poses, world velocity, IMU biases,
gravity, the IMU pose covariance, LiDAR and visual correspondence counts, and the processed
scan in LiDAR and world coordinates. Point offsets in output scans are zero:
the scan is deskewed to the estimate timestamp. The covariance ordering is
rotation followed by position, with radians/meters respectively.

`visual_update` means an image was processed; it does not guarantee that the
image supplied accepted visual constraints. `visual_correspondences` reports
retrieved visual patches, which need not all pass the update's acceptance checks.
`latestEstimate()` returns null
until the first successful update. Returned estimates are immutable shared
snapshots that remain valid after subsequent processing or a reset.

`mapPoints()` copies the current voxel map's plane centers and unmodeled samples
in world coordinates. It is not a dense archive of all observations; enable
sliding-map options to bound the spatial map extent. `saveMap("map.pcd")` writes
that snapshot as ASCII PCD and throws on an I/O failure. No files are written
unless explicitly requested.

Sliding-map configuration bounds the LiDAR map's spatial extent. The visual
feature map retains upstream accumulation behavior and is not pruned by the
LiDAR sliding window; memory can therefore grow during long camera-fusion
sessions. `reset()` releases both maps and starts a new trajectory.

## Upstream ROS compatibility and license

The ROS sources, launch files, configuration, messages, and other upstream
assets remain in place. The original CMake logic is preserved in
[`cmake/ROS.cmake`](../cmake/ROS.cmake). The README's historical ROS prerequisites
describe that upstream environment, not the standalone Eigen 5/OpenCV 5 setup.
To select the ROS application in an existing, compatible ROS 1/catkin environment:

```bash
catkin_make -DFAST_LIVO2_BUILD_ROS=ON
```

Standalone-only changes are isolated behind `FAST_LIVO2_STANDALONE` or in the
new wrapper/helper sources where practical. The standalone package follows the
repository's existing [GPLv2 license](../LICENSE).
