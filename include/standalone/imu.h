#pragma once

#include <memory>

namespace fast_livo2 {
namespace detail {

// Internal measurement representation. The public wrapper accepts ordinary
// timestamped vectors and converts them here, without any ROS dependency.
struct Imu
{
  struct Vector3
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };
  struct Stamp
  {
    double seconds = 0.0;
    double toSec() const { return seconds; }
  };
  struct Header
  {
    Stamp stamp;
  };

  using ConstPtr = std::shared_ptr<const Imu>;
  Header header;
  Vector3 angular_velocity;
  Vector3 linear_acceleration;
};

using ImuConstPtr = Imu::ConstPtr;

} // namespace detail
} // namespace fast_livo2
