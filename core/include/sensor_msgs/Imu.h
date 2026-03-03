#pragma once
#include <memory>
#include <ros/ros.h>

namespace sensor_msgs {

struct Imu {
  struct Header { ros::Time stamp; } header;
  struct Vec3 { double x=0.0, y=0.0, z=0.0; };
  Vec3 angular_velocity;
  Vec3 linear_acceleration;

  using Ptr      = std::shared_ptr<Imu>;
  using ConstPtr = std::shared_ptr<const Imu>;
};

// Non-nested ROS1-style aliases used by some code:
using ImuPtr      = std::shared_ptr<Imu>;
using ImuConstPtr = std::shared_ptr<const Imu>;

} // namespace sensor_msgs
