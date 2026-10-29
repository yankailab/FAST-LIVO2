#include <fast_livo2/slam.hpp>

#include <iostream>

// A stationary synthetic room demonstrates the streaming API without a driver,
// ROS, a dataset, or a viewer. Replace these samples with calibrated sensor data.
int main()
{
  fast_livo2::Config config;
  config.use_camera = false;
  fast_livo2::Slam slam(config);

  // Buffer enough IMU coverage to bracket every scan end. Acceleration is in
  // m/s^2 and includes gravity; a stationary, Z-up IMU reads approximately +g Z.
  for (int i = 0; i <= 340; ++i) {
    fast_livo2::ImuSample imu;
    imu.timestamp = 0.99 + i * 0.005;
    imu.acceleration = Eigen::Vector3d(0, 0, 9.81);
    slam.insertImu(imu);
  }

  std::size_t updates = 0;
  for (int frame = 0; frame < 16; ++frame) {
    fast_livo2::LidarScan scan;
    scan.timestamp = 1.0 + frame * 0.1;
    for (int i = -8; i <= 8; ++i) {
      for (int j = -8; j <= 8; ++j) {
        const float a = i * 0.2f, b = j * 0.2f;
        scan.points.push_back({4.0f, a, b, 1.0f, 0.0});
        scan.points.push_back({a, 4.0f, b, 1.0f, 0.0});
        scan.points.push_back({a, b, -2.0f, 1.0f, 0.0});
      }
    }
    for (std::size_t i = 0; i < scan.points.size(); ++i)
      scan.points[i].time_offset = 0.09 * i / (scan.points.size() - 1);
    slam.insertLidar(scan);
    const auto result = slam.process();
    if (result.status == fast_livo2::Status::Updated) {
      ++updates;
      std::cout << "t=" << result.estimate->timestamp
                << " position=" << result.estimate->T_world_imu.translation().transpose()
                << " points=" << result.estimate->scan_world.size() << '\n';
    }
  }

  slam.finish();
  while (slam.process().status != fast_livo2::Status::Finished) {}
  std::cout << updates << " updates, " << slam.mapPoints().size() << " map samples\n";
  return updates == 0 ? 1 : 0;
}
