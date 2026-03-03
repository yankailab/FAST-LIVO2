#pragma once
#include <memory>

namespace sensor_msgs {

struct Imu {
  struct Header { double stamp = 0.0; } header;
  struct Vec3 { double x=0.0, y=0.0, z=0.0; };
  Vec3 angular_velocity;
  Vec3 linear_acceleration;
};

using ImuConstPtr = std::shared_ptr<const Imu>;

} // namespace sensor_msgs

