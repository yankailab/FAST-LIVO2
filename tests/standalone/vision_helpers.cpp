#include "standalone/geometry.h"
#include "standalone/vision.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}

int main() {
  using namespace fast_livo2::detail;
  try {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const SE3 pose(rotation, Eigen::Vector3d(1, 2, 3));
    const Eigen::Vector3d point(0.2, -0.1, 2.0);
    require((pose.inverse() * (pose * point) - point).norm() < 1e-12, "Rigid-transform inverse failed");
    require(((pose * pose.inverse()) * point - point).norm() < 1e-12, "Rigid-transform composition failed");

    for (const auto& distortion : {std::array<double, 5>{},
                                  std::array<double, 5>{-0.08, 0.01, 0.002, -0.003, 0.001}}) {
      PinholeCamera camera(640, 480, 400, 405, 320, 240, distortion);
      const Eigen::Vector2d pixel = camera.world2cam(point);
      require((camera.cam2world(pixel) - point.normalized()).norm() < 1e-8, "Camera projection round trip failed");
      const auto analytic = camera.projectionJacobian(point);
      Eigen::Matrix<double, 2, 3> numerical;
      for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d offset = Eigen::Vector3d::Zero();
        offset[axis] = 1e-6;
        numerical.col(axis) = (camera.world2cam(point + offset) - camera.world2cam(point - offset)) / 2e-6;
      }
      require((analytic - numerical).cwiseAbs().maxCoeff() < 1e-6, "Distorted projection Jacobian failed");
      require(camera.isInFrame(Eigen::Vector2i(5, 5), 5), "Camera border inclusion failed");
      require(!camera.isInFrame(Eigen::Vector2i(635, 5), 5), "Camera border exclusion failed");
    }

    bool rejected = false;
    try { PinholeCamera camera(640, 480, 0, 400, 320, 240); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Invalid calibration was accepted");

    cv::Mat storage(24, 26, CV_8UC1);
    for (int row = 0; row < storage.rows; ++row)
      for (int column = 0; column < storage.cols; ++column)
        storage.at<unsigned char>(row, column) = 2 * row + 3 * column;
    const cv::Mat roi = storage(cv::Rect(2, 3, 20, 18));
    require(!roi.isContinuous(), "Test must exercise strided images");
    require(std::abs(interpolateMat_8u(roi, 5.25, 6.5) - (12 + 3 * 5.25 + 2 * 6.5)) < 1e-12,
            "Strided bilinear sampling failed");
    cv::Mat downsampled;
    halfSample(roi, downsampled);
    require(downsampled.rows == 9 && downsampled.cols == 10, "Half-sampling dimensions failed");
    require(downsampled.at<unsigned char>(0, 0) == 14, "Half-sampling average failed");
    require(std::abs(shiTomasiScore(roi, 10, 9)) < 1e-5, "Linear image should have zero corner score");
    rejected = false;
    try { (void)interpolateMat_8u(roi, 19.0, 1.0); }
    catch (const std::out_of_range&) { rejected = true; }
    require(rejected, "Invalid interpolation coordinate was accepted");
    std::cout << "Standalone geometry and vision helper checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
