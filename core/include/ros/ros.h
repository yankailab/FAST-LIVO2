#pragma once
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>

namespace ros {

struct Time {
  double sec = 0.0;
  Time() = default;
  explicit Time(double s) : sec(s) {}
  double toSec() const { return sec; }
  static Time now() { return Time(0.0); }
};

struct Duration {
  double sec = 0.0;
  Duration() = default;
  explicit Duration(double s) : sec(s) {}
  double toSec() const { return sec; }
};

class Publisher {
public:
  Publisher() = default;
  template <class MsgT>
  void publish(const MsgT&) const {}
};

class Rate {
public:
  explicit Rate(double /*hz*/) {}
  void sleep() const {}
};

class NodeHandle {
public:
  NodeHandle() = default;

  template <typename T>
  void param(const std::string& /*name*/, T& var, const T& default_val) const {
    var = default_val;
  }
};

inline void init(int /*argc*/, char** /*argv*/, const std::string& /*name*/) {}
inline bool ok() { return true; }
inline void spinOnce() {}

}  // namespace ros

// ---- ROS logging/assert macros used by FAST-LIVO2 ----
#define ROS_INFO(...)  do { std::fprintf(stderr,"[INFO] ");  std::fprintf(stderr,__VA_ARGS__); std::fprintf(stderr,"\n"); } while(0)
#define ROS_WARN(...)  do { std::fprintf(stderr,"[WARN] ");  std::fprintf(stderr,__VA_ARGS__); std::fprintf(stderr,"\n"); } while(0)
#define ROS_ERROR(...) do { std::fprintf(stderr,"[ERROR] "); std::fprintf(stderr,__VA_ARGS__); std::fprintf(stderr,"\n"); } while(0)

#define ROS_ASSERT(cond) do { \
  if(!(cond)) { \
    std::fprintf(stderr,"[ASSERT] %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    std::abort(); \
  } \
} while(0)
