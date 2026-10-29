#include "standalone/vision.h"

#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace fast_livo2 {
namespace detail {

PinholeCamera::PinholeCamera(int width, int height, double fx, double fy,
                           double cx, double cy, std::array<double, 5> distortion,
                           double scale)
    : width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy),
      scale_(scale), distortion_(distortion), distorted_(false) {
  if (width <= 0 || height <= 0 || !std::isfinite(fx) || fx <= 0 ||
      !std::isfinite(fy) || fy <= 0 || !std::isfinite(cx) || !std::isfinite(cy) ||
      !std::isfinite(scale) || scale <= 0) {
    throw std::invalid_argument("Invalid pinhole camera dimensions or intrinsics");
  }
  for (double coefficient : distortion_) {
    if (!std::isfinite(coefficient)) throw std::invalid_argument("Invalid camera distortion");
    distorted_ = distorted_ || coefficient != 0.0;
  }
  camera_matrix_ = cv::Mat(cv::Matx33d(fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1));
  distortion_matrix_ = cv::Mat(1, 5, CV_64FC1, distortion_.data()).clone();
}

Eigen::Vector3d PinholeCamera::cam2world(double u, double v) const {
  Eigen::Vector3d bearing((u - cx_) / fx_, (v - cy_) / fy_, 1.0);
  if (distorted_) {
    std::vector<cv::Point2d> source{{u, v}}, normalized;
    cv::undistortPoints(source, normalized, camera_matrix_, distortion_matrix_);
    bearing.x() = normalized.front().x;
    bearing.y() = normalized.front().y;
  }
  return bearing.normalized();
}

Eigen::Vector2d PinholeCamera::world2cam(const Eigen::Vector3d& point) const {
  const double x = point.x() / point.z(), y = point.y() / point.z();
  const double radius2 = x * x + y * y;
  const auto& d = distortion_;
  const double radial = 1.0 + radius2 * (d[0] + radius2 * (d[1] + radius2 * d[4]));
  return {fx_ * (x * radial + 2.0 * d[2] * x * y + d[3] * (radius2 + 2.0 * x * x)) + cx_,
          fy_ * (y * radial + d[2] * (radius2 + 2.0 * y * y) + 2.0 * d[3] * x * y) + cy_};
}

Eigen::Matrix<double, 2, 3> PinholeCamera::projectionJacobian(const Eigen::Vector3d& point) const {
  const double inverse_z = 1.0 / point.z();
  const double x = point.x() * inverse_z, y = point.y() * inverse_z;
  const double radius2 = x * x + y * y;
  const auto& d = distortion_;
  const double radial = 1.0 + radius2 * (d[0] + radius2 * (d[1] + radius2 * d[4]));
  const double radial_derivative = 2.0 * (d[0] + radius2 * (2.0 * d[1] + 3.0 * radius2 * d[4]));
  Eigen::Matrix2d distortion_jacobian;
  distortion_jacobian <<
      fx_ * (radial + x * x * radial_derivative + 2.0 * d[2] * y + 6.0 * d[3] * x),
      fx_ * (x * y * radial_derivative + 2.0 * d[2] * x + 2.0 * d[3] * y),
      fy_ * (x * y * radial_derivative + 2.0 * d[2] * x + 2.0 * d[3] * y),
      fy_ * (radial + y * y * radial_derivative + 6.0 * d[2] * y + 2.0 * d[3] * x);
  Eigen::Matrix<double, 2, 3> perspective;
  perspective << inverse_z, 0, -x * inverse_z, 0, inverse_z, -y * inverse_z;
  return distortion_jacobian * perspective;
}

void PinholeCamera::undistortImage(const cv::Mat& input, cv::Mat& output) const {
  if (distorted_) cv::undistort(input, output, camera_matrix_, distortion_matrix_);
  else output = input.clone();
}

double interpolateMat_8u(const cv::Mat& image, double x, double y) {
  if (image.type() != CV_8UC1 || !std::isfinite(x) || !std::isfinite(y) ||
      x < 0 || y < 0 || x >= image.cols - 1 || y >= image.rows - 1) {
    throw std::out_of_range("Bilinear interpolation requires a valid grayscale image coordinate");
  }
  const int column = static_cast<int>(x), row = static_cast<int>(y);
  const double dx = x - column, dy = y - row;
  const auto* top = image.ptr<std::uint8_t>(row);
  const auto* bottom = image.ptr<std::uint8_t>(row + 1);
  return (1.0 - dy) * ((1.0 - dx) * top[column] + dx * top[column + 1]) +
         dy * ((1.0 - dx) * bottom[column] + dx * bottom[column + 1]);
}

float shiTomasiScore(const cv::Mat& image, int x, int y) {
  if (image.type() != CV_8UC1) throw std::invalid_argument("Corner scoring requires grayscale input");
  constexpr int half_width = 4;
  if (x - half_width < 1 || y - half_width < 1 ||
      x + half_width >= image.cols - 1 || y + half_width >= image.rows - 1) return 0.0f;
  Eigen::Matrix2f structure = Eigen::Matrix2f::Zero();
  for (int row = y - half_width; row < y + half_width; ++row) {
    const auto* current = image.ptr<std::uint8_t>(row);
    const auto* above = image.ptr<std::uint8_t>(row - 1);
    const auto* below = image.ptr<std::uint8_t>(row + 1);
    for (int column = x - half_width; column < x + half_width; ++column) {
      const Eigen::Vector2f gradient(float(current[column + 1]) - current[column - 1],
                                     float(below[column]) - above[column]);
      structure.noalias() += gradient * gradient.transpose();
    }
  }
  // Preserve the frontend's historical 8x8 central-difference score scale.
  structure /= 128.0f;
  const float difference = structure(0, 0) - structure(1, 1);
  return 0.5f * (structure.trace() - std::sqrt(difference * difference +
                                            4.0f * structure(0, 1) * structure(0, 1)));
}

void halfSample(const cv::Mat& input, cv::Mat& output) {
  if (input.type() != CV_8UC1) throw std::invalid_argument("Half-sampling requires grayscale input");
  output.create(input.rows / 2, input.cols / 2, CV_8UC1);
  for (int row = 0; row < output.rows; ++row) {
    const auto* top = input.ptr<std::uint8_t>(2 * row);
    const auto* bottom = input.ptr<std::uint8_t>(2 * row + 1);
    auto* target = output.ptr<std::uint8_t>(row);
    for (int column = 0; column < output.cols; ++column) {
      const int source = 2 * column;
      target[column] = (unsigned(top[source]) + top[source + 1] + bottom[source] + bottom[source + 1]) / 4;
    }
  }
}

}  // namespace detail
}  // namespace fast_livo2
