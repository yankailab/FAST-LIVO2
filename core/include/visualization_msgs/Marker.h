#pragma once
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

namespace visualization_msgs {

struct Marker {
  struct Header {
    std::string frame_id;
    ros::Time stamp;
  } header;

  struct Position { double x=0.0, y=0.0, z=0.0; };
  struct Pose {
    Position position;
    geometry_msgs::Quaternion orientation;
  } pose;

  struct Scale { double x=1.0, y=1.0, z=1.0; } scale;
  struct Color { float r=1.f, g=1.f, b=1.f, a=1.f; } color;

  int id = 0;
  std::string ns;
  int type = 0;
  int action = 0;
  ros::Duration lifetime;

  // Minimal enums used by your code:
  static constexpr int ADD = 0;
  static constexpr int CYLINDER = 3;
};

} // namespace visualization_msgs
