#pragma once

#include <fast_livo2/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/version.hpp>
#if EIGEN_MAJOR_VERSION != 5
#error "FAST-LIVO2 requires Eigen 5.x headers; select the same Eigen installation used to build the library."
#endif
#if CV_VERSION_MAJOR != 5
#error "FAST-LIVO2 requires OpenCV 5.x headers; select the same OpenCV installation used to build the library."
#endif
#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace fast_livo2 {

// All timestamps use the same monotonic sensor clock in seconds.
struct ImuSample {
  double timestamp = 0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(); // m/s^2, includes gravity
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero(); // rad/s
};

struct Point {
  float x = 0, y = 0, z = 0, intensity = 0; // meters, arbitrary intensity units
  double time_offset = 0; // seconds from LidarScan::timestamp; zero for a depth frame
};
struct LidarScan {
  double timestamp = 0; // scan start, not scan end
  std::vector<Point> points;
};
struct ImageSample {
  double timestamp = 0; // exposure time; apply sensor time offsets before insertion
  cv::Mat image; // calibrated size, CV_8UC1 grayscale or CV_8UC3 BGR; copied on insert
};

struct CameraConfig {
  int width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  // OpenCV radial/tangential model. Images are rectified internally for direct VIO.
  std::array<double, 5> distortion{}; // k1, k2, p1, p2, k3
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
};
struct Config {
  bool use_imu = true;
  bool use_camera = false; // camera fusion requires IMU
  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  CameraConfig camera;
  double min_range = 0.1;
  double max_range = 200.0;
  std::size_t point_stride = 1;
  double scan_voxel_size = 0.1;
  int imu_initialization_samples = 30; // keep device stationary during initialization
  double accelerometer_covariance = 0.5, gyroscope_covariance = 0.3;
  double accelerometer_bias_covariance = 0.0001, gyroscope_bias_covariance = 0.0001;
  bool estimate_gravity = true, estimate_bias = true;
  bool align_gravity = true; // initialize world Z against gravity
  double map_voxel_size = 0.5;
  int map_max_layer = 2, map_max_points = 50, lidar_iterations = 5;
  std::vector<int> layer_initialization_points{5, 5, 5};
  double plane_eigenvalue_threshold = 0.0025;
  double range_error = 0.02, beam_error_degrees = 0.05, plane_sigma = 3.0;
  bool slide_local_map = false;
  int half_map_size = 100; // voxels
  double map_sliding_distance = 8.0; // meters
  int visual_iterations = 5, grid_size = 32, patch_size = 8, pyramid_levels = 3;
  double image_point_covariance = 100, image_outlier_threshold = 1000;
  bool use_normal = true, inverse_composition = false, raycast = false;
  bool estimate_exposure = true;
  double inverse_exposure_covariance = 0.1;
  // Full queues throw std::length_error instead of silently dropping sensor data.
  std::size_t max_imu_queue = 20000, max_lidar_queue = 200, max_image_queue = 200;
};

enum class Status { WaitingForLidar, WaitingForImu, WaitingForImage, Initializing,
                    Updated, NoPoints, Finished };
struct Estimate {
  double timestamp = 0;
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_world_lidar = Eigen::Isometry3d::Identity();
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_world = Eigen::Vector3d::Zero();
  // IMU error-state covariance order: rotation (radians), position (meters).
  Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Matrix<double, 6, 6>::Zero();
  std::size_t lidar_correspondences = 0;
  std::size_t visual_correspondences = 0;
  bool visual_update = false; // image processed; does not guarantee visual constraints
  std::vector<Point> scan_lidar; // downsampled, deskewed to timestamp; offsets zero
  std::vector<Point> scan_world; // same points after final LIO/VIO pose update
};
struct ProcessResult {
  Status status = Status::WaitingForLidar;
  std::shared_ptr<const Estimate> estimate; // set only for Updated
};

// Synchronous, per-instance estimator. Serialize all calls on an instance.
// Insertions copy data; each stream must have strictly increasing timestamps.
// LiDAR scans must not overlap in time. Invalid input throws invalid_argument.
// No background threads, ROS runtime, viewer, callbacks, or implicit file writes.
class FAST_LIVO2_EXPORT Slam {
public:
  explicit Slam(const Config& config = Config{});
  ~Slam();
  Slam(const Slam&) = delete;
  Slam& operator=(const Slam&) = delete;
  Slam(Slam&&) noexcept;
  Slam& operator=(Slam&&) noexcept;

  void insertImu(const ImuSample& sample);
  void insertLidar(const LidarScan& scan);
  void insertImage(const ImageSample& sample);
  // One synchronized update. LIO uses scan ends; LIVO cuts scans at image times.
  // Supply an IMU at/beyond the update time, and one at/before it. No extrapolation.
  ProcessResult process();
  std::shared_ptr<const Estimate> latestEstimate() const;
  // Snapshot of the estimator's current voxel map: plane centers and unmodeled
  // samples, in world coordinates. This is not an archive of all input scans.
  std::vector<Point> mapPoints() const;
  void saveMap(const std::string& pcd_path) const; // ASCII PCD; throws on I/O failure
  void reset(); // clears queues, state, map, and finish flag; keeps configuration
  void finish(); // disallows insertions; process() drains synchronized updates,
                 // then returns Finished; incomplete queues remain inspectable
  std::size_t pendingLidarScans() const;
  std::size_t pendingImages() const;
  const Config& config() const;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fast_livo2
