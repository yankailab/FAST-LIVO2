#include "fastlivo2_core/FastLivo2Core.hpp"
#include "Engine.hpp"
#include <mutex>
#include <deque>

namespace fastlivo2_core
{

  struct FastLivo2Core::Impl
  {
    std::mutex m;
    std::deque<ImuSample> imu_q;
    std::deque<LidarScan> lidar_q;
    std::deque<ImageFrame> img_q;
    std::unique_ptr<fastlivo2_core::Engine> engine;

    Eigen::Isometry3d Twb = Eigen::Isometry3d::Identity();
    std::vector<ColoredPoint> colored_map;
    std::string yaml_path;

    bool initialized = false;
  };

  FastLivo2Core::FastLivo2Core() : impl_(std::make_unique<Impl>())
  {
    impl_->engine = std::make_unique<fastlivo2_core::Engine>();
  }
  FastLivo2Core::~FastLivo2Core() = default;

  bool FastLivo2Core::init(const std::string &yaml_path)
  {
    std::lock_guard<std::mutex> lk(impl_->m);
    impl_->yaml_path = yaml_path;
    //  impl_->initialized = true; // placeholder
    //  return true;
    return impl_->engine->init();
  }

  bool FastLivo2Core::setCameraPinhole(int width, int height, double fx, double fy, double cx, double cy)
  {
    return impl_->engine->setCameraPinhole(width, height, fx, fy, cx, cy);
  }

  bool FastLivo2Core::setCameraPinholeDistorted(int width, int height,
                                                double fx, double fy, double cx, double cy,
                                                const std::array<double, 5> &dist)
  {
    return impl_->engine->setCameraPinholeDistorted(width, height, fx, fy, cx, cy, dist);
  }

  bool FastLivo2Core::setLidarToCameraExtrinsic(const std::array<double, 9> &R_cl_rowmajor,
                                                const std::array<double, 3> &t_cl)
  {
    return impl_->engine->setLidarToCameraExtrinsic(R_cl_rowmajor, t_cl);
  }

  bool FastLivo2Core::setImuToLidarExtrinsic(const std::array<double, 9> &R_il_rowmajor,
                                             const std::array<double, 3> &t_il)
  {
    return impl_->engine->setImuToLidarExtrinsic(R_il_rowmajor, t_il);
  }

  void FastLivo2Core::pushImu(const ImuSample &s) { impl_->engine->pushImu(s); }
  void FastLivo2Core::pushLidar(const LidarScan &s) { impl_->engine->pushLidar(s); }
  void FastLivo2Core::pushImage(const ImageFrame &f) { impl_->engine->pushImage(f); }
  bool FastLivo2Core::spinOnce() { return impl_->engine->spinOnce(); }
  // bool FastLivo2Core::spinOnce(){
  //   LidarMeasureGroup meas;
  //   return impl_->engine->sync_packages(meas); // for now: “data plumbing OK”
  // }
  Eigen::Isometry3d FastLivo2Core::T_wb() const
  {
    std::lock_guard<std::mutex> lk(impl_->m);
    return impl_->Twb;
  }

  std::vector<ColoredPoint> FastLivo2Core::getColoredMap() const
  {
    std::lock_guard<std::mutex> lk(impl_->m);
    return impl_->colored_map;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr FastLivo2Core::getMap() const
  {
    return impl_->engine->getMap();
  }

  bool FastLivo2Core::configureColoredMapMaintenance(int downsample_every_n_frames,
                                                     size_t max_map_points,
                                                     double voxel_leaf_size_m)
  {
    return impl_->engine->configureColoredMapMaintenance(
        downsample_every_n_frames, max_map_points, voxel_leaf_size_m);
  }

  bool FastLivo2Core::saveColoredMapPLY(const std::string &path) const
  {
    return impl_->engine->saveColoredMapPLY(path);
  }

  bool FastLivo2Core::saveColoredMapPCD(const std::string &path) const
  {
    return impl_->engine->saveColoredMapPCD(path);
  }

  void FastLivo2Core::reset(bool clear_map)
  {
    impl_->engine->reset(clear_map);
  }

  bool FastLivo2Core::getPose(Pose &out) const
  {
    return impl_->engine->getPose(out);
  }

  bool FastLivo2Core::getVelocity(Eigen::Vector3d &out_vel) const
  {
    return impl_->engine->getVelocity(out_vel);
  }

  bool FastLivo2Core::getBiases(fastlivo2_core::Biases &out_biases) const
  {
    return impl_->engine->getBiases(out_biases);
  }

  bool FastLivo2Core::getStateDebug(fastlivo2_core::StateDebug &out_dbg) const
  {
    return impl_->engine->getStateDebug(out_dbg);
  }

  void FastLivo2Core::setImageEnabled(bool en)
  {
    impl_->engine->setImageEnabled(en);
  }

} // namespace fastlivo2_core
