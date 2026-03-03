#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace fastlivo2_core {

struct ImuSample {
  double t = 0.0;                  // seconds
  Eigen::Vector3d acc = {0,0,0};   // m/s^2
  Eigen::Vector3d gyro = {0,0,0};  // rad/s
};

struct LidarPoint {
  float x=0, y=0, z=0;     // meters
  float intensity=0;
  uint8_t tag=0;
  double t=0.0;            // absolute time seconds (or relative; decide later)
};

struct LidarScan {
  double t0=0.0;                 // scan start time
  double t1=0.0;                 // scan end time
  std::vector<LidarPoint> pts;
};

struct ImageFrame {
  double t=0.0;                  // seconds
  int width=0, height=0;

  // BGR packed, size = width*height*3
  std::vector<uint8_t> bgr;
};

struct ColoredPoint {
  float x=0,y=0,z=0;
  uint8_t r=0,g=0,b=0;
};

} // namespace fastlivo2_core

