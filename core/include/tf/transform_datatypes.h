#pragma once
#include <cmath>
#include <geometry_msgs/Quaternion.h>

namespace tf {

inline geometry_msgs::Quaternion createQuaternionMsgFromRollPitchYaw(
    double roll, double pitch, double yaw) {
  // Minimal correct math (so debug output isn’t garbage)
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

} // namespace tf
