#include <fast_livo2/slam.hpp>
#include "IMU_Processing.h"
#include "vio.h"
#include <opencv2/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace fast_livo2 {
namespace {
constexpr double unset = -std::numeric_limits<double>::infinity();
void require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}
bool positive(double x) { return std::isfinite(x) && x > 0; }
bool rigid(const Eigen::Isometry3d& t) {
  return t.matrix().allFinite() &&
      (t.linear().transpose() * t.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) &&
      std::abs(t.linear().determinant() - 1.0) < 1e-5 &&
      t.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-5);
}
void validate(const Config& c) {
  require(rigid(c.T_imu_lidar), "T_imu_lidar must be a finite rigid transform");
  require(std::isfinite(c.min_range) && c.min_range >= 0 && positive(c.max_range) && c.max_range > c.min_range,
          "invalid LiDAR range limits");
  require(c.point_stride > 0 && positive(c.scan_voxel_size), "invalid scan filter settings");
  require(c.imu_initialization_samples >= 2, "IMU initialization needs at least two samples");
  require(positive(c.accelerometer_covariance) && positive(c.gyroscope_covariance) &&
          positive(c.accelerometer_bias_covariance) && positive(c.gyroscope_bias_covariance), "invalid IMU noise covariance");
  require(positive(c.map_voxel_size) && c.map_max_layer >= 0 && c.map_max_layer <= 10 &&
          c.map_max_points >= 5 && c.lidar_iterations > 0, "invalid voxel map settings");
  require(c.layer_initialization_points.size() > static_cast<std::size_t>(c.map_max_layer), "missing voxel layer thresholds");
  for (int n : c.layer_initialization_points)
    require(n >= 3 && n <= c.map_max_points, "invalid voxel initialization threshold");
  require(positive(c.plane_eigenvalue_threshold) && positive(c.range_error) &&
          positive(c.beam_error_degrees) && positive(c.plane_sigma), "invalid plane/noise settings");
  require(c.half_map_size > 0 && positive(c.map_sliding_distance), "invalid sliding map settings");
  require(c.max_imu_queue > 1 && c.max_lidar_queue > 0 && c.max_image_queue > 0, "invalid queue capacity");
  if (c.use_camera) {
    require(c.use_imu, "camera fusion requires IMU");
    const auto& k = c.camera;
    require(k.width >= 32 && k.height >= 32 && positive(k.fx) && positive(k.fy) &&
            std::isfinite(k.cx) && std::isfinite(k.cy) && rigid(k.T_camera_lidar), "invalid camera calibration");
    for (double d : k.distortion) require(std::isfinite(d), "invalid distortion coefficient");
    require(c.grid_size > 10 && c.grid_size <= std::min(k.width, k.height) && c.patch_size >= 4 &&
            c.patch_size <= 32 && c.patch_size % 2 == 0 && c.pyramid_levels >= 1 && c.pyramid_levels <= 5,
            "invalid visual grid or patch settings");
    require((c.patch_size + 2) * (1 << (c.pyramid_levels - 1)) < std::min(k.width, k.height), "image too small for patch pyramid");
    require(c.visual_iterations > 0 && positive(c.image_point_covariance) && positive(c.image_outlier_threshold) &&
            positive(c.inverse_exposure_covariance), "invalid visual estimation settings");
  }
}
Point makePoint(const Eigen::Vector3d& p, float intensity = 0) {
  return {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), intensity, 0};
}
NativeImuConstPtr nativeImu(const ImuSample& s) {
  auto p = std::make_shared<NativeImu>();
  p->header.stamp.seconds = s.timestamp;
  p->linear_acceleration = {s.acceleration.x(), s.acceleration.y(), s.acceleration.z()};
  p->angular_velocity = {s.angular_velocity.x(), s.angular_velocity.y(), s.angular_velocity.z()};
  return p;
}
void collectMap(const VoxelOctoTree* tree, std::vector<Point>& points) {
  if (!tree) return;
  if (tree->plane_ptr_->is_plane_) {
    points.push_back(makePoint(tree->plane_ptr_->center_));
    return;
  }
  for (const auto& p : tree->temp_points_) points.push_back(makePoint(p.point_w));
  for (auto* child : tree->leaves_) collectMap(child, points);
}
} // namespace

class Slam::Impl {
public:
  explicit Impl(const Config& settings) : cfg(settings) {
    validate(cfg);
    imu.set_extrinsic(cfg.T_imu_lidar.translation(), cfg.T_imu_lidar.linear());
    imu.set_gyr_cov_scale(V3D::Constant(cfg.gyroscope_covariance));
    imu.set_acc_cov_scale(V3D::Constant(cfg.accelerometer_covariance));
    imu.set_gyr_bias_cov(V3D::Constant(cfg.gyroscope_bias_covariance));
    imu.set_acc_bias_cov(V3D::Constant(cfg.accelerometer_bias_covariance));
    imu.set_inv_expo_cov(cfg.inverse_exposure_covariance);
    imu.set_imu_init_frame_num(cfg.imu_initialization_samples);
    imu.lidar_type = AVIA;
    if (!cfg.use_imu) imu.disable_imu();
    if (!cfg.estimate_gravity) imu.disable_gravity_est();
    if (!cfg.estimate_bias) imu.disable_bias_est();
    if (!cfg.use_camera || !cfg.estimate_exposure) imu.disable_exposure_est();
    VoxelMapConfig vc{};
    vc.max_voxel_size_ = cfg.map_voxel_size;
    vc.max_layer_ = cfg.map_max_layer;
    vc.max_iterations_ = cfg.lidar_iterations;
    vc.layer_init_num_ = cfg.layer_initialization_points;
    vc.max_points_num_ = cfg.map_max_points;
    vc.planner_threshold_ = cfg.plane_eigenvalue_threshold;
    vc.beam_err_ = cfg.beam_error_degrees;
    vc.dept_err_ = cfg.range_error;
    vc.sigma_num_ = cfg.plane_sigma;
    vc.map_sliding_en = cfg.slide_local_map;
    vc.half_map_size = cfg.half_map_size;
    vc.sliding_thresh = cfg.map_sliding_distance;
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> empty;
    map = std::make_unique<VoxelMapManager>(vc, empty);
    map->extR_ = cfg.T_imu_lidar.linear();
    map->extT_ = cfg.T_imu_lidar.translation();
    filter.setLeafSize(cfg.scan_voxel_size, cfg.scan_voxel_size, cfg.scan_voxel_size);
    if (cfg.use_camera) {
      const auto& k = cfg.camera;
      camera = std::make_unique<vk::PinholeCamera>(k.width, k.height, k.fx, k.fy, k.cx, k.cy);
      visual = std::make_unique<VIOManager>();
      visual->cam = camera.get();
      visual->grid_size = cfg.grid_size;
      visual->grid_n_height = 17;
      visual->grid_n_width = 0;
      visual->patch_size = cfg.patch_size;
      visual->patch_pyrimid_level = cfg.pyramid_levels;
      visual->max_iterations = cfg.visual_iterations;
      visual->img_point_cov = cfg.image_point_covariance;
      visual->outlier_threshold = cfg.image_outlier_threshold;
      visual->normal_en = cfg.use_normal;
      visual->inverse_composition_en = cfg.inverse_composition;
      visual->raycast_en = cfg.raycast;
      visual->exposure_estimate_en = cfg.estimate_exposure;
      visual->plot_flag = false;
      visual->colmap_output_en = false;
      visual->state = &state;
      visual->state_propagat = &propagated;
      visual->setImuToLidarExtrinsic(cfg.T_imu_lidar.translation(), cfg.T_imu_lidar.linear());
      std::vector<double> r(9), t(3);
      for (int i = 0; i < 3; ++i) {
        t[i] = k.T_camera_lidar.translation()[i];
        for (int j = 0; j < 3; ++j) r[i * 3 + j] = k.T_camera_lidar.linear()(i, j);
      }
      visual->setLidarToCameraExtrinsic(r, t);
      visual->initializeVIO();
      if (std::any_of(k.distortion.begin(), k.distortion.end(), [](double x) { return x != 0; })) {
        cv::Mat K(cv::Matx33d(k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1));
        cv::Mat distortion(1, 5, CV_64F, const_cast<double*>(k.distortion.data()));
        cv::initUndistortRectifyMap(K, distortion, cv::Mat(), K, cv::Size(k.width, k.height), CV_32FC1, rect_x, rect_y);
      }
    }
  }
  void accepting() const {
    if (finished) throw std::logic_error("finish() called; reset before inserting data");
  }
  ProcessResult waiting(Status status) const { return {finished ? Status::Finished : status, {}}; }
  void transform(const PointCloudXYZI::Ptr& input, const PointCloudXYZI::Ptr& output) const {
    *output = *input;
    const Eigen::Isometry3d pose = poseImu() * cfg.T_imu_lidar;
    for (auto& p : *output) {
      const V3D v = pose * V3D(p.x, p.y, p.z);
      p.x = v.x(); p.y = v.y(); p.z = v.z();
    }
  }
  Eigen::Isometry3d poseImu() const {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = state.rot_end;
    result.translation() = state.pos_end;
    return result;
  }
  // Keep a real sample on either side of the update; interpolate an exact endpoint
  // so the core never extrapolates beyond available IMU observations.
  void gatherImu(double target, MeasureGroup& group) {
    while (imu_queue.size() > 1 && imu_queue[1].timestamp <= update_time) imu_queue.pop_front();
    for (const auto& s : imu_queue) {
      if (s.timestamp > target) break;
      if (imu.imu_need_init || s.timestamp > update_time) group.imu.push_back(nativeImu(s));
    }
    if (group.imu.empty() || group.imu.back()->header.stamp.toSec() < target) {
      auto upper = std::upper_bound(imu_queue.begin(), imu_queue.end(), target,
          [](double t, const ImuSample& s) { return t < s.timestamp; });
      const auto& b = *upper;
      const auto& a = *std::prev(upper);
      const double alpha = (target - a.timestamp) / (b.timestamp - a.timestamp);
      ImuSample end{target, a.acceleration + alpha * (b.acceleration - a.acceleration),
                    a.angular_velocity + alpha * (b.angular_velocity - a.angular_velocity)};
      group.imu.push_back(nativeImu(end));
    }
    while (imu_queue.size() > 1 && imu_queue[1].timestamp <= target) imu_queue.pop_front();
  }
  ProcessResult process() {
    if (lidar_queue.empty()) return waiting(Status::WaitingForLidar);
    if (cfg.use_camera && image_queue.empty()) return waiting(Status::WaitingForImage);
    if (!std::isfinite(update_time)) update_time = lidar_queue.front().timestamp;
    // Images predating the first LiDAR point cannot form a synchronized update.
    while (cfg.use_camera && !image_queue.empty() && image_queue.front().timestamp <= update_time)
      image_queue.pop_front();
    if (cfg.use_camera && image_queue.empty()) return waiting(Status::WaitingForImage);
    const double target = cfg.use_camera ? image_queue.front().timestamp : lidar_queue.front().end;
    if (target > last_lidar_end) return waiting(Status::WaitingForLidar);
    if (cfg.use_imu && (imu_queue.empty() || imu_queue.back().timestamp < target))
      return waiting(Status::WaitingForImu);
    if (cfg.use_imu && imu_queue.front().timestamp > update_time)
      throw std::invalid_argument("IMU must cover the start of the LiDAR interval; reset and supply earlier IMU data");

    LidarMeasureGroup measurements;
    measurements.lidar_frame_beg_time = update_time;
    measurements.lidar_frame_end_time = target;
    measurements.last_lio_update_time = update_time;
    measurements.lio_vio_flg = cfg.use_imu ? LIO : LO;
    MeasureGroup group;
    group.lio_time = target;
    if (cfg.use_imu) gatherImu(target, group);
    while (!lidar_queue.empty()) {
      auto& scan = lidar_queue.front();
      if (scan.timestamp > target) break;
      while (scan.cursor < scan.points.size()) {
        const auto& p = scan.points[scan.cursor];
        const double stamp = scan.timestamp + p.time_offset;
        if (stamp > target) break;
        PointType point;
        point.x = p.x; point.y = p.y; point.z = p.z; point.intensity = p.intensity;
        point.normal_x = point.normal_y = point.normal_z = 0;
        point.curvature = static_cast<float>((stamp - update_time) * 1000.0);
        measurements.pcl_proc_cur->push_back(point);
        ++scan.cursor;
      }
      if (scan.end <= target) lidar_queue.pop_front(); else break;
      if (!cfg.use_camera) break;
    }
    measurements.lidar = measurements.pcl_proc_cur;
    measurements.measures.push_back(group);
    cv::Mat img;
    if (cfg.use_camera) { img = image_queue.front().image; image_queue.pop_front(); }
    auto undistorted = PointCloudXYZI::Ptr(new PointCloudXYZI);
    const bool initializing = cfg.use_imu && imu.imu_need_init;
    if (cfg.use_imu || !measurements.lidar->empty()) imu.Process2(measurements, state, undistorted);
    update_time = target;
    if (initializing) {
      if (!imu.imu_need_init && cfg.align_gravity && !gravity_aligned) {
        M3D rotation = Eigen::Quaterniond::FromTwoVectors(state.gravity, V3D(0, 0, -1)).toRotationMatrix();
        state.rot_end = rotation * state.rot_end;
        state.pos_end = rotation * state.pos_end;
        state.vel_end = rotation * state.vel_end;
        state.gravity = rotation * state.gravity;
        gravity_aligned = true;
      }
      return {Status::Initializing, {}};
    }
    if (undistorted->empty()) return {Status::NoPoints, {}};
    propagated = state;
    auto down = PointCloudXYZI::Ptr(new PointCloudXYZI);
    filter.setInputCloud(undistorted);
    filter.filter(*down);
    if (down->empty()) return {Status::NoPoints, {}};
    auto world = PointCloudXYZI::Ptr(new PointCloudXYZI);
    transform(down, world);
    map->state_ = state;
    map->feats_undistort_ = undistorted;
    map->feats_down_body_ = down;
    map->feats_down_world_ = world;
    map->feats_down_size_ = static_cast<int>(down->size());
    if (!map_initialized) { map->BuildVoxelMap(); map_initialized = true; }
    map->StateEstimation(propagated);
    state = map->state_;
    transform(down, world);
    for (std::size_t i = 0; i < world->size(); ++i) {
      auto& pv = map->pv_list_[i];
      pv.point_w = V3D((*world)[i].x, (*world)[i].y, (*world)[i].z);
      const M3D& cross = map->cross_mat_list_[i];
      const M3D rotation = state.rot_end * cfg.T_imu_lidar.linear();
      pv.var = rotation * map->body_cov_list_[i] * rotation.transpose() +
          (-cross) * state.cov.block<3, 3>(0, 0) * (-cross).transpose() + state.cov.block<3, 3>(3, 3);
    }
    map->UpdateVoxelMap(map->pv_list_);
    if (cfg.slide_local_map) map->mapSliding();
    if (visual) {
      propagated = state; // VIO prior is the LIO posterior at this same timestamp.
      visual->processFrame(img, map->pv_list_, map->voxel_map_, target);
    }
    if (!state.rot_end.allFinite() || !state.pos_end.allFinite() || !state.cov.allFinite())
      throw std::runtime_error("FAST-LIVO2 produced a non-finite estimate; reset and check calibration/data");
    auto out = std::make_shared<Estimate>();
    out->timestamp = target;
    out->T_world_imu = poseImu();
    out->T_world_lidar = out->T_world_imu * cfg.T_imu_lidar;
    out->velocity_world = state.vel_end;
    out->accelerometer_bias = state.bias_a;
    out->gyroscope_bias = state.bias_g;
    out->gravity_world = state.gravity;
    out->pose_covariance = state.cov.block<6, 6>(0, 0);
    out->lidar_correspondences = map->effct_feat_num_;
    out->visual_correspondences = visual ? static_cast<std::size_t>(visual->total_points) : 0;
    out->visual_update = !!visual;
    out->scan_lidar.reserve(down->size()); out->scan_world.reserve(down->size());
    for (const auto& p : *down) {
      const V3D xyz(p.x, p.y, p.z);
      out->scan_lidar.push_back(makePoint(xyz, p.intensity));
      out->scan_world.push_back(makePoint(out->T_world_lidar * xyz, p.intensity));
    }
    latest = out;
    return {Status::Updated, out};
  }
  struct Scan { double timestamp, end; std::vector<Point> points; std::size_t cursor = 0; };
  Config cfg;
  ImuProcess imu;
  StatesGroup state, propagated;
  std::unique_ptr<VoxelMapManager> map;
  std::unique_ptr<vk::PinholeCamera> camera;
  std::unique_ptr<VIOManager> visual;
  pcl::VoxelGrid<PointType> filter;
  cv::Mat rect_x, rect_y;
  std::deque<ImuSample> imu_queue;
  std::deque<Scan> lidar_queue;
  std::deque<ImageSample> image_queue;
  double last_imu = unset, last_lidar = unset, last_lidar_end = unset, last_image = unset, update_time = unset;
  bool finished = false, map_initialized = false, gravity_aligned = false;
  std::shared_ptr<const Estimate> latest;
};

Slam::Slam(const Config& config) : impl_(std::make_unique<Impl>(config)) {}
Slam::~Slam() = default;
Slam::Slam(Slam&&) noexcept = default;
Slam& Slam::operator=(Slam&&) noexcept = default;
void Slam::insertImu(const ImuSample& sample) {
  auto& p = *impl_; p.accepting();
  require(p.cfg.use_imu, "IMU is disabled");
  require(std::isfinite(sample.timestamp) && sample.timestamp >= 0 && sample.timestamp > p.last_imu &&
          sample.acceleration.allFinite() && sample.angular_velocity.allFinite(), "invalid or non-increasing IMU sample");
  require(!p.imu.imu_need_init || sample.acceleration.norm() > 1e-6, "initialization acceleration must include gravity");
  if (p.imu_queue.size() >= p.cfg.max_imu_queue) throw std::length_error("IMU queue full");
  p.imu_queue.push_back(sample); p.last_imu = sample.timestamp;
}
void Slam::insertLidar(const LidarScan& scan) {
  auto& p = *impl_; p.accepting();
  require(std::isfinite(scan.timestamp) && scan.timestamp >= 0 && scan.timestamp > p.last_lidar &&
          scan.timestamp >= p.last_lidar_end, "invalid, overlapping, or non-increasing LiDAR scan");
  require(!scan.points.empty(), "LiDAR scan is empty");
  if (p.lidar_queue.size() >= p.cfg.max_lidar_queue) throw std::length_error("LiDAR queue full");
  Impl::Scan copy{scan.timestamp, scan.timestamp, {}};
  copy.points.reserve(scan.points.size() / p.cfg.point_stride + 1);
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const auto& point = scan.points[i];
    require(std::isfinite(point.time_offset) && point.time_offset >= 0 && point.time_offset <= 60,
            "point time_offset must be seconds in [0, 60]");
    copy.end = std::max(copy.end, scan.timestamp + point.time_offset);
    const double range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z));
    if (i % p.cfg.point_stride == 0 && std::isfinite(range) && std::isfinite(point.intensity) &&
        range >= p.cfg.min_range && range <= p.cfg.max_range) copy.points.push_back(point);
  }
  require(!copy.points.empty(), "LiDAR scan has no points after range/finite/stride filtering");
  require(std::isfinite(copy.end), "LiDAR end timestamp is not finite");
  std::stable_sort(copy.points.begin(), copy.points.end(), [](const Point& a, const Point& b) { return a.time_offset < b.time_offset; });
  const double end = copy.end;
  p.lidar_queue.push_back(std::move(copy));
  p.last_lidar = scan.timestamp; p.last_lidar_end = end;
}
void Slam::insertImage(const ImageSample& sample) {
  auto& p = *impl_; p.accepting();
  require(p.cfg.use_camera, "camera is disabled");
  require(std::isfinite(sample.timestamp) && sample.timestamp >= 0 && sample.timestamp > p.last_image &&
          sample.timestamp > p.update_time, "invalid, late, or non-increasing image timestamp");
  require(!sample.image.empty() && (sample.image.type() == CV_8UC1 || sample.image.type() == CV_8UC3) &&
          sample.image.cols == p.cfg.camera.width && sample.image.rows == p.cfg.camera.height,
          "image must be calibrated size, grayscale 8-bit or BGR 8-bit");
  if (p.image_queue.size() >= p.cfg.max_image_queue) throw std::length_error("image queue full");
  cv::Mat copy;
  if (!p.rect_x.empty()) cv::remap(sample.image, copy, p.rect_x, p.rect_y, cv::INTER_LINEAR);
  else copy = sample.image.clone();
  p.image_queue.push_back({sample.timestamp, std::move(copy)}); p.last_image = sample.timestamp;
}
ProcessResult Slam::process() { return impl_->process(); }
std::shared_ptr<const Estimate> Slam::latestEstimate() const { return impl_->latest; }
std::vector<Point> Slam::mapPoints() const {
  std::vector<Point> result;
  for (const auto& entry : impl_->map->voxel_map_) collectMap(entry.second, result);
  return result;
}
void Slam::saveMap(const std::string& path) const {
  const auto points = mapPoints();
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot open map file: " + path);
  file << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH "
       << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA ascii\n";
  file << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (const auto& p : points) file << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << '\n';
  file.close();
  if (!file) throw std::runtime_error("cannot write map file: " + path);
}
void Slam::reset() { auto replacement = std::make_unique<Impl>(impl_->cfg); impl_ = std::move(replacement); }
void Slam::finish() { impl_->finished = true; }
std::size_t Slam::pendingLidarScans() const { return impl_->lidar_queue.size(); }
std::size_t Slam::pendingImages() const { return impl_->image_queue.size(); }
const Config& Slam::config() const { return impl_->cfg; }
} // namespace fast_livo2
