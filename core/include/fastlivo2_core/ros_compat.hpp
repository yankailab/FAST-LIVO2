#pragma once
#include <memory>

#ifdef FASTLIVO2_STANDALONE

// Minimal shims to satisfy type declarations appearing in headers.
// Keep these as "empty" unless compilation forces you to add fields.

namespace sensor_msgs {
struct Imu {
  struct Vec3 { double x=0, y=0, z=0; };
  Vec3 angular_velocity;
  Vec3 linear_acceleration;
};
using ImuConstPtr = std::shared_ptr<const Imu>;

struct Image {};
struct PointCloud2 {};
}  // namespace sensor_msgs

namespace nav_msgs { struct Odometry {}; }
namespace geometry_msgs { struct PoseStamped {}; }

namespace tf {
class TransformBroadcaster {};
class TransformListener {};
}  // namespace tf

namespace ros {
struct Time { double toSec() const { return 0.0; } };
}  // namespace ros

#endif  // FASTLIVO2_STANDALONE

