#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <array>

namespace fast_livo2 {
namespace detail {

// Internal camera interface used by the unmodified visual frontend call sites.
// Coordinates follow OpenCV: x right, y down, z forward; pixel centers are integer.
class AbstractCamera {
 public:
  virtual ~AbstractCamera() = default;
  virtual int width() const = 0;
  virtual int height() const = 0;
  virtual double fx() const = 0;
  virtual double fy() const = 0;
  virtual double cx() const = 0;
  virtual double cy() const = 0;
  virtual double scale() const = 0;
  virtual Eigen::Vector3d cam2world(double u, double v) const = 0;
  Eigen::Vector3d cam2world(const Eigen::Vector2d& pixel) const {
    return cam2world(pixel.x(), pixel.y());
  }
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& point) const = 0;
  virtual Eigen::Matrix<double, 2, 3> projectionJacobian(const Eigen::Vector3d& point) const = 0;
  bool isInFrame(const Eigen::Vector2i& pixel, int border = 0) const {
    return pixel.x() >= border && pixel.y() >= border &&
           pixel.x() < width() - border && pixel.y() < height() - border;
  }
};

class PinholeCamera final : public AbstractCamera {
 public:
  // Intrinsics and dimensions describe the actual input image. Distortion order:
  // OpenCV k1, k2, p1, p2, k3. Zero distortion is a rectified pinhole camera.
  PinholeCamera(int width, int height, double fx, double fy, double cx, double cy,
                std::array<double, 5> distortion = {}, double scale = 1.0);
  int width() const override { return width_; }
  int height() const override { return height_; }
  double fx() const override { return fx_; }
  double fy() const override { return fy_; }
  double cx() const override { return cx_; }
  double cy() const override { return cy_; }
  double scale() const override { return scale_; }
  using AbstractCamera::cam2world;
  Eigen::Vector3d cam2world(double u, double v) const override;
  Eigen::Vector2d world2cam(const Eigen::Vector3d& point) const override;
  Eigen::Matrix<double, 2, 3> projectionJacobian(const Eigen::Vector3d& point) const override;
  void undistortImage(const cv::Mat& input, cv::Mat& output) const;

 private:
  int width_, height_;
  double fx_, fy_, cx_, cy_, scale_;
  std::array<double, 5> distortion_;
  bool distorted_;
  cv::Mat camera_matrix_, distortion_matrix_;
};

double interpolateMat_8u(const cv::Mat& image, double x, double y);
float shiTomasiScore(const cv::Mat& image, int x, int y);
void halfSample(const cv::Mat& input, cv::Mat& output);

}  // namespace detail
}  // namespace fast_livo2

// These names are private to the standalone build; vikit is not required or
// exposed by the installed public API. Aliases minimize upstream merge changes.
namespace vk {
using fast_livo2::detail::AbstractCamera;
using fast_livo2::detail::PinholeCamera;
using fast_livo2::detail::interpolateMat_8u;
using fast_livo2::detail::shiTomasiScore;
using fast_livo2::detail::halfSample;
}
