#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fast_livo2 {
namespace detail {

// The visual frontend only needs rigid-transform composition and application.
// Keep that small interface native to Eigen instead of depending on legacy Sophus.
class SE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3() = default;
  SE3(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
      : rotation_(rotation), translation_(translation) {}

  const Eigen::Matrix3d& rotation_matrix() const { return rotation_; }
  const Eigen::Vector3d& translation() const { return translation_; }
  SE3 inverse() const {
    const Eigen::Matrix3d inverse_rotation = rotation_.transpose();
    return SE3(inverse_rotation, -(inverse_rotation * translation_));
  }
  SE3 operator*(const SE3& other) const {
    return SE3(rotation_ * other.rotation_, rotation_ * other.translation_ + translation_);
  }
  Eigen::Vector3d operator*(const Eigen::Vector3d& point) const {
    return rotation_ * point + translation_;
  }

 private:
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();
};

}  // namespace detail
}  // namespace fast_livo2
