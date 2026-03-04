#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace fastlivo2_core
{

  struct ImuSample
  {
    double t = 0.0;                   // seconds
    Eigen::Vector3d acc = {0, 0, 0};  // m/s^2
    Eigen::Vector3d gyro = {0, 0, 0}; // rad/s
  };

  struct LidarPoint
  {
    float x = 0, y = 0, z = 0; // meters
    float intensity = 0;
    uint8_t tag = 0;
    double t = 0.0; // absolute time seconds (or relative; decide later)
  };

  struct LidarScan
  {
    double t0 = 0.0; // scan start time
    double t1 = 0.0; // scan end time
    std::vector<LidarPoint> pts;
  };

  struct ImageFrame
  {
    double t = 0.0; // seconds
    int width = 0, height = 0;

    // BGR packed, size = width*height*3
    std::vector<uint8_t> bgr;
  };

  struct ColoredPoint
  {
    float x = 0, y = 0, z = 0;
    uint8_t r = 0, g = 0, b = 0;
  };

  struct Pose
  {
    double t = 0.0;                                        // seconds (LIO/VIO update time)
    Eigen::Vector3d p = Eigen::Vector3d::Zero();           // position (world)
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); // orientation (world)
  };

  struct Biases
  {
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  // bias_g
    Eigen::Vector3d accel = Eigen::Vector3d::Zero(); // bias_a
  };

  struct StateDebug
  {
    double t = 0.0;

    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();

    Biases biases;
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

    double inv_expo_time = 0.0; // _state.inv_expo_time
    bool is_first_frame = false;

    // From LidarMeasureGroup::lio_vio_flg (WAIT/LIO/VIO/LO)
    int lio_vio_flg = 0;

    // Debug: covariance diagonal (size depends on StatesGroup definition)
    Eigen::VectorXd cov_diag;
  };

} // namespace fastlivo2_core
