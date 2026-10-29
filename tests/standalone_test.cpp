#include <fast_livo2/slam.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace fast_livo2;
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template<class Exception = std::invalid_argument, class F>
void rejects(F&& operation, const char* message) {
  try { operation(); } catch (const Exception&) { return; }
  throw std::runtime_error(message);
}
LidarScan scene(double stamp, double duration = 0.08, double x = 0) {
  LidarScan scan;
  scan.timestamp = stamp;
  // Three perpendicular textured surfaces with many points per map voxel.
  for (int a = -15; a <= 15; ++a) {
    for (int b = -15; b <= 15; ++b) {
      const float u = a * 0.07f, v = b * 0.07f;
      scan.points.push_back({u - static_cast<float>(x), v, 3, float((a + b + 30) % 16), 0});
      scan.points.push_back({2.13f - static_cast<float>(x), u, v + 3, 20, 0});
      scan.points.push_back({u - static_cast<float>(x), -2, v + 3, 30, 0});
    }
  }
  for (std::size_t i = 0; i < scan.points.size(); ++i)
    scan.points[i].time_offset = duration * i / (scan.points.size() - 1);
  return scan;
}
void addImu(Slam& slam, double from, double to) {
  for (int i = 0; from + i * 0.005 <= to + 1e-9; ++i)
    slam.insertImu({from + i * 0.005, Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d::Zero()});
}
void contracts() {
  Config invalid;
  invalid.map_max_layer = 5;
  rejects([&] { Slam bad(invalid); }, "missing layer settings accepted");
  invalid = Config{}; invalid.T_imu_lidar.linear()(0, 0) = 2;
  rejects([&] { Slam bad(invalid); }, "nonrigid extrinsics accepted");
  Slam slam;
  check(slam.process().status == Status::WaitingForLidar, "idle status");
  rejects([&] { slam.insertLidar({0, {}}); }, "empty scan accepted");
  auto bad = scene(1); bad.points[0].time_offset = -1;
  rejects([&] { slam.insertLidar(bad); }, "negative point offset accepted");
  slam.insertLidar(scene(1));
  check(slam.process().status == Status::WaitingForImu, "missing IMU status");
  rejects([&] { slam.insertLidar(scene(1.01)); }, "overlapping scan accepted");
  rejects([&] { slam.insertImu({1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}); }, "zero-gravity initialization accepted");
  addImu(slam, 0.99, 1.1);
  rejects([&] { slam.insertImu({1.1, Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d::Zero()}); }, "duplicate IMU accepted");
  check(slam.process().status == Status::Initializing, "initialization status");
  slam.finish();
  rejects<std::logic_error>([&] { slam.insertLidar(scene(2)); }, "insertion after finish accepted");
  check(slam.process().status == Status::Finished, "finished status");
  slam.reset();
  check(!slam.latestEstimate() && slam.mapPoints().empty(), "reset retained state");
  check(slam.pendingLidarScans() == 0 && slam.process().status == Status::WaitingForLidar, "reset queues");
  Config small; small.max_lidar_queue = 1;
  Slam bounded(small); bounded.insertLidar(scene(0));
  rejects<std::length_error>([&] { bounded.insertLidar(scene(1)); }, "queue limit ignored");
}
void lidarImu() {
  Config config;
  config.imu_initialization_samples = 10;
  config.T_imu_lidar.translation() = Eigen::Vector3d(0.1, -0.02, 0.03);
  Slam slam(config);
  addImu(slam, 0, 0.5);
  slam.insertLidar(scene(0));
  check(slam.process().status == Status::Initializing, "first scan must initialize");
  for (int i = 1; i <= 4; ++i) {
    slam.insertLidar(scene(i * 0.1));
    auto result = slam.process();
    check(result.status == Status::Updated && !!result.estimate, "LIO did not update");
    const auto& e = *result.estimate;
    check(std::abs(e.timestamp - (i * 0.1 + 0.08)) < 1e-9, "wrong scan-end timestamp");
    check(e.T_world_imu.translation().norm() < 0.05, "stationary LIO drift");
    check(e.T_world_lidar.matrix().isApprox((e.T_world_imu * config.T_imu_lidar).matrix(), 1e-9), "wrong extrinsic pose composition");
    check(e.scan_lidar.size() == e.scan_world.size() && !e.scan_world.empty(), "missing output cloud");
    const auto& p = e.scan_lidar.front(); const auto& q = e.scan_world.front();
    check((e.T_world_lidar * Eigen::Vector3d(p.x, p.y, p.z) - Eigen::Vector3d(q.x, q.y, q.z)).norm() < 1e-5,
          "world cloud inconsistent with pose");
    if (i > 1) check(e.lidar_correspondences > 50, "synthetic planes failed to constrain registration");
  }
  check(!slam.mapPoints().empty(), "map empty");
  const auto retained = slam.latestEstimate();
  const auto path = std::filesystem::temp_directory_path() / "fast_livo2_standalone_test.pcd";
  slam.saveMap(path.string());
  std::ifstream file(path); std::string contents((std::istreambuf_iterator<char>(file)), {});
  check(contents.find("DATA ascii") != std::string::npos, "map export invalid");
  file.close(); std::filesystem::remove(path);
  slam.reset();
  check(retained && !retained->scan_world.empty() && !slam.latestEstimate(), "snapshot invalidated by reset");
}
void lidarOnly() {
  Config config; config.use_imu = false;
  Slam slam(config);
  slam.insertLidar(scene(0, 0)); // global-shutter depth
  auto first = slam.process();
  check(first.status == Status::Updated, "LiDAR-only first frame");
  slam.insertLidar(scene(0.1, 0, 0.02));
  auto second = slam.process();
  check(second.status == Status::Updated && second.estimate->T_world_lidar.matrix().allFinite(), "LiDAR-only tracking");
  check(second.estimate->T_world_lidar.translation().x() > 0.005, "LiDAR motion was not estimated");
  slam.insertLidar(scene(0.2, 0, 0.04)); slam.finish();
  check(slam.process().status == Status::Updated, "finish failed to drain complete scan");
  check(slam.process().status == Status::Finished, "finish did not terminate");
}
void interpolatedImu() {
  Config config; config.imu_initialization_samples = 2;
  Slam slam(config);
  for (double t : {0.0, 0.05, 0.1, 0.2})
    slam.insertImu({t, Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d(0, 0, t <= 0.1 ? 0 : 0.2)});
  const auto sparse = [](double start, double duration) {
    return LidarScan{start, {{1, 1, 3, 1, 0}, {2, 2, 4, 1, duration}}};
  };
  slam.insertLidar(sparse(0, 0.1));
  check(slam.process().status == Status::Initializing, "sparse initialization");
  slam.insertLidar(sparse(0.11, 0.04));
  check(slam.process().status == Status::Updated, "first interpolated endpoint");
  slam.insertLidar(sparse(0.16, 0.015));
  const auto result = slam.process();
  check(result.status == Status::Updated, "second interpolated endpoint");
  const auto& rotation = result.estimate->T_world_imu.linear();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  check(std::abs(yaw - 0.005625) < 1e-7, "integration reused stale IMU bracket after interpolated endpoint");
}
void degenerateMap() {
  Config config; config.use_imu = false;
  Slam slam(config);
  for (int i = 0; i < 12; ++i) {
    slam.insertLidar({i * 0.1, {{1, 1, 3, 1, 0}, {2, 2, 4, 1, 0}}});
    const auto result = slam.process();
    check(result.status == Status::Updated && result.estimate->T_world_lidar.matrix().allFinite(),
          "repeated points created an invalid plane");
    check(result.estimate->lidar_correspondences == 0, "repeated points should not constrain a plane");
  }
}
void lidarVisual(bool inverse = false, bool normal = true) {
  Config config; config.use_camera = true; config.imu_initialization_samples = 5;
  config.inverse_composition = inverse; config.use_normal = normal;
  config.camera.width = 320; config.camera.height = 240;
  config.camera.fx = 100; config.camera.fy = 100;
  config.camera.cx = 160; config.camera.cy = 120;
  config.grid_size = 24; // width not divisible by grid size
  config.camera.distortion = {0.01, -0.005, 0, 0, 0};
  Slam slam(config);
  addImu(slam, 0, 0.5);
  slam.insertLidar(scene(0, 0.18));
  slam.insertLidar(scene(0.2, 0.18));
  check(slam.process().status == Status::WaitingForImage, "LIVO missing image status");
  cv::Mat image(240, 320, CV_8UC3);
  for (int y = 0; y < image.rows; ++y)
    for (int x = 0; x < image.cols; ++x)
      image.at<cv::Vec3b>(y, x) = cv::Vec3b((x * 7 + y * 3) % 256, (x * 3 + y * 11) % 256, (x + y) % 256);
  rejects([&] { slam.insertImage({0.02, cv::Mat(20, 20, CV_8UC1)}); }, "bad image dimensions accepted");
  slam.insertImage({0.051, image}); // between IMU samples: endpoint interpolation
  check(slam.process().status == Status::Initializing, "visual initialization");
  check(slam.pendingLidarScans() == 2, "scan tail lost during visual slicing");
  bool had_visual_constraints = false;
  for (double t : {0.101, 0.151, 0.251, 0.351}) {
    slam.insertImage({t, image});
    const auto result = slam.process();
    check(result.status == Status::Updated && result.estimate->visual_update, "LIVO failed to process image");
    check(std::abs(result.estimate->timestamp - t) < 1e-9, "LIVO pose not at image time");
    check(result.estimate->T_world_imu.matrix().allFinite(), "LIVO nonfinite pose");
    had_visual_constraints = had_visual_constraints || result.estimate->visual_correspondences > 0;
  }
  check(had_visual_constraints, "visual pipeline never tracked a patch");
  slam.finish();
  check(slam.process().status == Status::Finished && slam.pendingLidarScans() == 1, "incomplete scan not retained on finish");
}
}
int main() {
  try {
    contracts(); lidarImu(); lidarOnly(); interpolatedImu(); degenerateMap();
    lidarVisual(); lidarVisual(true, false);
    std::cout << "Standalone API, LIO, LO, and LIVO checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test failure: " << e.what() << '\n'; return 1;
  }
}
