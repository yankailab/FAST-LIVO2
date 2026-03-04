#include "Engine.hpp"
#include <opencv2/imgproc.hpp>
#include <omp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

namespace fastlivo2_core
{

  Engine::Engine()
  {
    feats_undistort.reset(new PointCloudXYZI());
    feats_down_body.reset(new PointCloudXYZI());
    feats_down_world.reset(new PointCloudXYZI());
    pcl_w_wait_pub.reset(new PointCloudXYZI());

    // init LidarMeasures internal pointers safely
    LidarMeasures.lidar.reset(new PointCloudXYZI());
    LidarMeasures.pcl_proc_cur.reset(new PointCloudXYZI());
    LidarMeasures.pcl_proc_next.reset(new PointCloudXYZI());

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    imu_proc_ = std::make_unique<ImuProcess>();
  }

  bool Engine::init()
  {
    // Build voxelmap + vio manager like in constructor/initializeComponents
    ros::NodeHandle nh; // your stub NodeHandle -> default params
    loadVoxelConfig(nh, voxel_config_);

    voxelmap_manager.reset(new VoxelMapManager(voxel_config_, voxel_map_));
    vio_manager.reset(new VIOManager());

    // set extrinsics into voxelmap_manager and vio_manager like initializeComponents()
    voxelmap_manager->extT_ = extT;
    voxelmap_manager->extR_ = extR;

    vio_manager->grid_size = grid_size;
    vio_manager->patch_size = patch_size;
    vio_manager->outlier_threshold = outlier_threshold;
    vio_manager->setImuToLidarExtrinsic(extT, extR);
    vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
    vio_manager->state = &_state;
    vio_manager->state_propagat = &state_propagat;
    vio_manager->max_iterations = max_iterations;
    vio_manager->img_point_cov = IMG_POINT_COV;
    vio_manager->normal_en = normal_en;
    vio_manager->inverse_composition_en = inverse_composition_en;
    vio_manager->raycast_en = raycast_en;
    vio_manager->grid_n_width = grid_n_width;
    vio_manager->grid_n_height = grid_n_height;
    vio_manager->patch_pyrimid_level = patch_pyrimid_level;
    vio_manager->exposure_estimate_en = exposure_estimate_en;

    if (cam_owner_)
    {
      vio_manager->pinhole_cam = cam_owner_.get();
      vio_manager->cam = cam_owner_.get();
      if (!vio_initialized_)
      {
        vio_manager->initializeVIO();
        vio_initialized_ = true;
      }
    }
    else
    {
      vio_manager->cam = nullptr;
      vio_manager->pinhole_cam = nullptr;
    }

    // IMU proc settings (defaults; tune later)
    if (!imu_proc_)
      imu_proc_ = std::make_unique<ImuProcess>();
    imu_proc_->set_extrinsic(extT, extR);
    imu_proc_->set_inv_expo_cov(inv_expo_cov);

    return true;
  }

  bool Engine::setCameraPinhole(int width, int height, double fx, double fy, double cx, double cy)
  {
    return setCameraPinholeDistorted(width, height, fx, fy, cx, cy, {0.0, 0.0, 0.0, 0.0, 0.0});
  }

  bool Engine::setCameraPinholeDistorted(int width, int height,
                                         double fx, double fy, double cx, double cy,
                                         const std::array<double, 5> &d)
  {
    if (width <= 0 || height <= 0)
      return false;
    if (fx <= 0 || fy <= 0)
      return false;

    // vikit_common expects:
    // (width, height, scale, fx, fy, cx, cy, d0, d1, d2, d3, d4)
    // where d0..d4 typically correspond to k1,k2,p1,p2,k3.
    cam_owner_ = std::make_unique<vk::PinholeCamera>(
        double(width), double(height),
        1.0, // scale
        fx, fy, cx, cy,
        d[0], d[1], d[2], d[3], d[4]);

    if (!vio_manager)
      return true;

    vio_manager->pinhole_cam = cam_owner_.get();
    vio_manager->cam = cam_owner_.get();

    // Initialize VIO once (avoid repeated re-init surprises)
    if (!vio_initialized_)
    {
      vio_manager->initializeVIO();
      vio_initialized_ = true;
    }
    return true;
  }

  bool Engine::setLidarToCameraExtrinsic(const std::array<double, 9> &R_cl_rowmajor,
                                         const std::array<double, 3> &t_cl)
  {
    // Store in the same format upstream uses: vector<double>(9/3)
    cameraextrinR.assign(R_cl_rowmajor.begin(), R_cl_rowmajor.end());
    cameraextrinT.assign(t_cl.begin(), t_cl.end());

    // If VIO manager already exists, apply immediately
    if (vio_manager)
    {
      vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);

      // If VIO has already been initialized, this *might* be enough.
      // If you observe that VIO still uses old extrinsics at runtime,
      // we can re-run initializeVIO() after updating extrinsics.
      // vio_manager->initializeVIO();
    }
    return true;
  }

  bool Engine::setImuToLidarExtrinsic(const std::array<double, 9> &R_il_rowmajor,
                                      const std::array<double, 3> &t_il)
  {
    // Update internal extR/extT used by transformLidar/RGBpointBodyToWorld
    extT = V3D(t_il[0], t_il[1], t_il[2]);
    extR << R_il_rowmajor[0], R_il_rowmajor[1], R_il_rowmajor[2],
        R_il_rowmajor[3], R_il_rowmajor[4], R_il_rowmajor[5],
        R_il_rowmajor[6], R_il_rowmajor[7], R_il_rowmajor[8];

    // Apply to IMU processor immediately
    imu_proc_->set_extrinsic(extT, extR);

    // Apply to voxelmap manager if exists
    if (voxelmap_manager)
    {
      voxelmap_manager->extT_ = extT;
      voxelmap_manager->extR_ = extR;
    }

    // Apply to VIO if exists
    if (vio_manager)
    {
      vio_manager->setImuToLidarExtrinsic(extT, extR);
    }

    return true;
  }

  sensor_msgs::Imu::ConstPtr Engine::makeImuMsg(const ImuSample &s)
  {
    auto imu = std::make_shared<sensor_msgs::Imu>();
    imu->header.stamp = ros::Time(s.t);
    imu->linear_acceleration.x = s.acc.x();
    imu->linear_acceleration.y = s.acc.y();
    imu->linear_acceleration.z = s.acc.z();
    imu->angular_velocity.x = s.gyro.x();
    imu->angular_velocity.y = s.gyro.y();
    imu->angular_velocity.z = s.gyro.z();
    return imu;
  }

  PointCloudXYZI::Ptr Engine::makeCloud(const LidarScan &scan)
  {
    auto cloud = std::make_shared<PointCloudXYZI>();
    cloud->reserve(scan.pts.size());
    for (const auto &p : scan.pts)
    {
      PointType pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pt.intensity = p.intensity;
      // IMPORTANT: FAST-LIO/LIVO uses curvature as "time offset in ms"
      // in several places. Give it something meaningful:
      pt.curvature = float((p.t - scan.t0) * 1000.0); // ms
      cloud->push_back(pt);
    }
    return cloud;
  }

  cv::Mat Engine::makeBgrCopy(const ImageFrame &img)
  {
    if (img.width <= 0 || img.height <= 0)
      return {};
    if ((int)img.bgr.size() != img.width * img.height * 3)
      return {};
    cv::Mat m(img.height, img.width, CV_8UC3, (void *)img.bgr.data());
    return m.clone();
  }

  void Engine::pushImu(const ImuSample &s)
  {
    auto msg = makeImuMsg(s);
    std::lock_guard<std::mutex> lk(mtx_);
    imu_buf_.push_back(msg);
  }

  void Engine::pushLidar(const LidarScan &s)
  {
    LidarItem it;
    it.t0 = s.t0 + lidar_time_offset;
    it.t1 = s.t1 + lidar_time_offset;
    it.cloud = makeCloud(s);

    std::lock_guard<std::mutex> lk(mtx_);
    lidar_buf_.push_back(std::move(it));
  }

  void Engine::pushImage(const ImageFrame &img)
  {
    ImgItem it;
    it.t = img.t + img_time_offset;
    it.bgr = makeBgrCopy(img);

    std::lock_guard<std::mutex> lk(mtx_);
    img_buf_.push_back(std::move(it));
  }

  // ---- This is a simplified, library-friendly LIVO sync ----
  // It alternates LIO then VIO (like upstream LIVO branch), but without multi-frame cutting.
  // If you want the exact upstream “pcl_proc_cur/next” cutting later, we can add it.
  bool Engine::sync_packages(LidarMeasureGroup &meas)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (imu_buf_.empty())
      return false;

    // Initialize last update time
    if (meas.last_lio_update_time < 0.0)
    {
      if (!lidar_buf_.empty())
        meas.last_lio_update_time = lidar_buf_.front().t0;
      else
        return false;
    }

    // If no images are used, always produce LIO measurements
    if (!img_enabled_)
    {
      // behave like ONLY_LIO: consume one lidar scan + imu up to scan end
      if (lidar_buf_.empty())
        return false;

      const auto lidar = lidar_buf_.front();
      const double t0 = lidar.t0;
      const double t1 = lidar.t1;
      if (imu_buf_.empty())
        return false;
      if (imu_buf_.back()->header.stamp.toSec() < t1)
        return false;

      meas.lidar = lidar.cloud;
      meas.lidar_frame_beg_time = t0;
      meas.lidar_frame_end_time = t1;
      meas.pcl_proc_cur = meas.lidar;

      MeasureGroup m;
      m.imu.clear();
      m.lio_time = t1;

      while (!imu_buf_.empty())
      {
        if (imu_buf_.front()->header.stamp.toSec() > t1)
          break;
        if (imu_buf_.front()->header.stamp.toSec() > meas.last_lio_update_time)
          m.imu.push_back(imu_buf_.front());
        imu_buf_.pop_front();
      }

      meas.measures.clear();
      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      meas.last_lio_update_time = t1;

      lidar_buf_.pop_front();
      return true;
    }

    // Alternate: WAIT/VIO -> produce LIO ; LIO -> produce VIO
    const EKF_STATE last_flg = meas.lio_vio_flg;

    if (last_flg == WAIT || last_flg == VIO)
    {
      if (lidar_buf_.empty())
        return false;

      const auto lidar = lidar_buf_.front();
      const double t0 = lidar.t0;
      const double t1 = lidar.t1;

      // Need IMU coverage
      if (imu_buf_.back()->header.stamp.toSec() < t1)
        return false;

      meas.lidar = lidar.cloud;
      meas.lidar_frame_beg_time = t0;
      meas.lidar_frame_end_time = t1;
      meas.pcl_proc_cur = meas.lidar; // consistent with ONLY_LIO branch upstream

      // Build MeasureGroup m (IMU slice up to t1)
      MeasureGroup m;
      m.imu.clear();
      m.lio_time = t1;

      while (!imu_buf_.empty())
      {
        if (imu_buf_.front()->header.stamp.toSec() > t1)
          break;
        if (imu_buf_.front()->header.stamp.toSec() > meas.last_lio_update_time)
          m.imu.push_back(imu_buf_.front());
        imu_buf_.pop_front();
      }

      meas.measures.clear();
      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      meas.last_lio_update_time = t1;

      lidar_buf_.pop_front();
      return true;
    }

    if (last_flg == LIO)
    {
      if (img_buf_.empty())
        return false;

      const double img_capture_time = img_buf_.front().t + exposure_time_init;

      // Require IMU coverage to image time (Process2 will propagate there)
      if (imu_buf_.back()->header.stamp.toSec() < img_capture_time)
        return false;

      MeasureGroup m;
      m.imu.clear();
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;

      m.img = img_buf_.front().bgr;

      // Optional: attach IMU between last LIO and this VIO time
      while (!imu_buf_.empty())
      {
        if (imu_buf_.front()->header.stamp.toSec() > img_capture_time)
          break;
        if (imu_buf_.front()->header.stamp.toSec() > meas.last_lio_update_time)
          m.imu.push_back(imu_buf_.front());
        imu_buf_.pop_front();
      }

      meas.measures.clear();
      meas.measures.push_back(m);
      meas.lio_vio_flg = VIO;

      img_buf_.pop_front();
      return true;
    }

    return false;
  }

  void Engine::handleFirstFrame()
  {
    if (!is_first_frame)
    {
      _first_lidar_time = LidarMeasures.last_lio_update_time;
      imu_proc_->first_lidar_time = _first_lidar_time; // same behavior as upstream
      is_first_frame = true;
      ROS_INFO("FIRST LIDAR FRAME!");
    }
  }

  void Engine::gravityAlignment()
  {
    if (!imu_proc_->imu_need_init && !gravity_align_finished)
    {
      V3D ez(0, 0, -1), gz(_state.gravity);
      Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
      M3D G_R_I0 = G_q_I0.toRotationMatrix();

      _state.pos_end = G_R_I0 * _state.pos_end;
      _state.rot_end = G_R_I0 * _state.rot_end;
      _state.vel_end = G_R_I0 * _state.vel_end;
      _state.gravity = G_R_I0 * _state.gravity;

      gravity_align_finished = true;
      ROS_INFO("Gravity Alignment Finished");
    }
  }

  void Engine::processImu()
  {
    imu_proc_->Process2(LidarMeasures, _state, feats_undistort);

    if (gravity_align_en)
      gravityAlignment();

    state_propagat = _state;
    voxelmap_manager->state_ = _state;
    voxelmap_manager->feats_undistort_ = feats_undistort;
  }

  void Engine::stateEstimationAndMapping()
  {
    switch (LidarMeasures.lio_vio_flg)
    {
    case VIO:
      if (vio_initialized_ && vio_manager && vio_manager->cam)
        handleVIO();
      else
        handleLIO(); // fallback
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
    default:
      break;
    }
  }

  bool Engine::getPose(fastlivo2_core::Pose &out) const
  {
    std::lock_guard<std::mutex> lk(mtx_);

    if (!is_first_frame)
      return false;

    out.t = LidarMeasures.last_lio_update_time;
    out.p = _state.pos_end;
    out.q = Eigen::Quaterniond(_state.rot_end);
    out.q.normalize();
    return true;
  }

  void Engine::reset(bool clear_map)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    // 1) clear incoming buffers
    imu_buf_.clear();
    lidar_buf_.clear();
    img_buf_.clear();

    // 2) clear processing clouds
    if (feats_undistort)
      feats_undistort->clear();
    if (feats_down_body)
      feats_down_body->clear();
    if (feats_down_world)
      feats_down_world->clear();
    if (pcl_w_wait_pub)
      pcl_w_wait_pub->clear();

    // 3) reset state and flags
    _state = StatesGroup(); // default state
    state_propagat = _state;

    is_first_frame = false;
    _first_lidar_time = -1.0;
    gravity_align_finished = false;
    lidar_map_inited = false;

    // 4) reset LidarMeasures
    LidarMeasures = LidarMeasureGroup();
    // Ensure required pointers are allocated (constructor usually does some of this)
    if (!LidarMeasures.lidar)
      LidarMeasures.lidar.reset(new PointCloudXYZI());
    if (!LidarMeasures.pcl_proc_cur)
      LidarMeasures.pcl_proc_cur.reset(new PointCloudXYZI());
    if (!LidarMeasures.pcl_proc_next)
      LidarMeasures.pcl_proc_next.reset(new PointCloudXYZI());
    LidarMeasures.measures.clear();
    LidarMeasures.last_lio_update_time = -1.0;
    LidarMeasures.lio_vio_flg = WAIT;

    // 5) reset / clear maps
    if (clear_map)
    {
      voxel_map_.clear();

      if (colored_map)
        colored_map->clear();
      if (colored_frame)
        colored_frame->clear();
      if (pcl_wait_pub)
        pcl_wait_pub->clear();

      pub_num = 0;
      color_frame_count_ = 0;
    }

    // 6) re-create managers to ensure internal references match cleared map/state
    if (voxelmap_manager)
    {
      voxelmap_manager.reset(new VoxelMapManager(voxel_config_, voxel_map_));
      voxelmap_manager->extT_ = extT;
      voxelmap_manager->extR_ = extR;
    }
    if (vio_manager)
    {
      vio_manager.reset(new VIOManager());

      // re-apply VIO configuration
      vio_manager->grid_size = grid_size;
      vio_manager->patch_size = patch_size;
      vio_manager->outlier_threshold = outlier_threshold;
      vio_manager->setImuToLidarExtrinsic(extT, extR);
      vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
      vio_manager->state = &_state;
      vio_manager->state_propagat = &state_propagat;
      vio_manager->max_iterations = max_iterations;
      vio_manager->img_point_cov = IMG_POINT_COV;
      vio_manager->normal_en = normal_en;
      vio_manager->inverse_composition_en = inverse_composition_en;
      vio_manager->raycast_en = raycast_en;
      vio_manager->grid_n_width = grid_n_width;
      vio_manager->grid_n_height = grid_n_height;
      vio_manager->patch_pyrimid_level = patch_pyrimid_level;
      vio_manager->exposure_estimate_en = exposure_estimate_en;

      // re-bind camera if available
      if (cam_owner_)
      {
        vio_manager->pinhole_cam = cam_owner_.get();
        vio_manager->cam = cam_owner_.get();
        // re-init VIO
        vio_initialized_ = false;
        vio_manager->initializeVIO();
        vio_initialized_ = true;
      }
      else
      {
        vio_manager->cam = nullptr;
        vio_manager->pinhole_cam = nullptr;
        vio_initialized_ = false;
      }
    }

    // 7) reset IMU processor (clean internal state), then re-apply settings
    imu_proc_ = std::make_unique<ImuProcess>();
    imu_proc_->set_extrinsic(extT, extR);
    imu_proc_->set_inv_expo_cov(inv_expo_cov);
  }

  bool Engine::getVelocity(Eigen::Vector3d &out_vel) const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!is_first_frame)
      return false;
    out_vel = _state.vel_end;
    return true;
  }

  bool Engine::getBiases(fastlivo2_core::Biases &out_biases) const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!is_first_frame)
      return false;
    out_biases.gyro = _state.bias_g;
    out_biases.accel = _state.bias_a;
    return true;
  }

  bool Engine::getStateDebug(fastlivo2_core::StateDebug &out_dbg) const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!is_first_frame)
      return false;

    out_dbg.t = LidarMeasures.last_lio_update_time;

    out_dbg.p = _state.pos_end;
    out_dbg.q = Eigen::Quaterniond(_state.rot_end);
    out_dbg.q.normalize();
    out_dbg.v = _state.vel_end;

    out_dbg.biases.gyro = _state.bias_g;
    out_dbg.biases.accel = _state.bias_a;

    out_dbg.gravity = _state.gravity;
    out_dbg.inv_expo_time = _state.inv_expo_time;

    out_dbg.is_first_frame = is_first_frame;
    out_dbg.lio_vio_flg = int(LidarMeasures.lio_vio_flg);

    // Covariance diagonal (StatesGroup::cov must exist for this; it does in FAST-LIVO2)
    out_dbg.cov_diag = _state.cov.diagonal();

    return true;
  }

  void Engine::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                              const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
  {
    PointCloudXYZI().swap(*trans_cloud);
    trans_cloud->reserve(input_cloud->size());
    for (size_t i = 0; i < input_cloud->size(); i++)
    {
      pcl::PointXYZINormal p_c = input_cloud->points[i];
      Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
      p = (rot * (extR * p + extT) + t);
      PointType pi;
      pi.x = p(0);
      pi.y = p(1);
      pi.z = p(2);
      pi.intensity = p_c.intensity;
      trans_cloud->points.push_back(pi);
    }
  }

  void Engine::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
  {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
  }

  void Engine::handleLIO()
  {
    if (feats_undistort->empty() || (feats_undistort == nullptr))
    {
      ROS_WARN("[ LIO ]: No point!!!");
      return;
    }

    // downsample
    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);

    int feats_down_size = (int)feats_down_body->points.size();
    voxelmap_manager->feats_down_body_ = feats_down_body;

    transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
    voxelmap_manager->feats_down_world_ = feats_down_world;
    voxelmap_manager->feats_down_size_ = feats_down_size;

    if (!lidar_map_inited)
    {
      lidar_map_inited = true;
      voxelmap_manager->BuildVoxelMap();
    }

    // core LIO solve
    voxelmap_manager->StateEstimation(state_propagat);
    _state = voxelmap_manager->state_;

    // update per-point variance and update voxel map (same as upstream)
    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
    transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);

    for (size_t i = 0; i < world_lidar->points.size(); i++)
    {
      voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
      M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
      M3D var = voxelmap_manager->body_cov_list_[i];

      var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
            (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() +
            _state.cov.block<3, 3>(3, 3);

      voxelmap_manager->pv_list_[i].var = var;
    }

    voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);

    if (voxelmap_manager->config_setting_.map_sliding_en)
    {
      voxelmap_manager->mapSliding();
    }

    // Build pcl_w_wait_pub like upstream (used by VIO for colorization)
    PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    int size = (int)laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
      RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }
    *pcl_w_wait_pub = *laserCloudWorld;
  }

  void Engine::handleVIO()
  {
    if (!pcl_w_wait_pub || pcl_w_wait_pub->empty())
    {
      ROS_WARN("[ VIO ] No point!!!");
      return;
    }
    if (!vio_manager || !vio_manager->new_frame_ || !vio_manager->new_frame_->cam_)
      return;
    if (LidarMeasures.measures.empty())
      return;

    const double t_rel = LidarMeasures.last_lio_update_time - _first_lidar_time;

    // VIO update
    vio_manager->processFrame(
        LidarMeasures.measures.back().img, // real BGR (CV_8UC3)
        voxelmap_manager->pv_list_,
        voxelmap_manager->voxel_map_,
        t_rel);

    // Choose image for sampling: prefer vio_manager->img_rgb if it’s produced
    cv::Mat img_bgr = vio_manager->img_rgb.empty()
                          ? LidarMeasures.measures.back().img
                          : vio_manager->img_rgb;
    if (img_bgr.empty() || img_bgr.type() != CV_8UC3)
      return;

    // Accumulate scans like upstream
    pub_num++;
    *pcl_wait_pub += *pcl_w_wait_pub;
    if (pub_num < pub_scan_num)
      return;
    pub_num = 0;

    colored_frame->clear();
    colored_frame->reserve(pcl_wait_pub->size());

    for (size_t i = 0; i < pcl_wait_pub->size(); i++)
    {
      const auto &p = pcl_wait_pub->points[i];
      V3D p_w(p.x, p.y, p.z);

      V3D pf(vio_manager->new_frame_->w2f(p_w));
      if (pf[2] < 0)
        continue;

      V2D pc(vio_manager->new_frame_->w2c(p_w));
      if (!vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3))
        continue;

      V3F pix = vio_manager->getInterpolatedPixel(img_bgr, pc);

      PointTypeRGB pr;
      pr.x = p.x;
      pr.y = p.y;
      pr.z = p.z;
      pr.r = (uint8_t)pix[2];
      pr.g = (uint8_t)pix[1];
      pr.b = (uint8_t)pix[0];

      if (pf.norm() > blind_rgb_points)
      {
        colored_frame->push_back(pr);
        colored_map->push_back(pr);
      }
    }

    // We just produced a colorized frame (passed pub_scan_num gating)
    color_frame_count_++;
    maintainColoredMap_();

    // Clear like upstream does after publish
    PointCloudXYZI().swap(*pcl_wait_pub);
    PointCloudXYZI().swap(*pcl_w_wait_pub);
  }

  bool Engine::spinOnce()
  {
    if (!sync_packages(LidarMeasures))
      return false;

    handleFirstFrame();
    processImu();
    stateEstimationAndMapping();
    return true;
  }

  bool Engine::configureColoredMapMaintenance(int downsample_every_n_frames,
                                              size_t max_map_points,
                                              double voxel_leaf_size_m)
  {
    // Allow disabling with 0 / <=0
    color_ds_every_n_ = downsample_every_n_frames;
    color_max_points_ = max_map_points;
    color_voxel_leaf_ = voxel_leaf_size_m;
    return true;
  }

  bool Engine::saveColoredMapPLY(const std::string &path) const
  {
    if (!colored_map || colored_map->empty())
      return false;
    return pcl::io::savePLYFileBinary(path, *colored_map) == 0;
  }
  bool Engine::saveColoredMapPCD(const std::string &path) const
  {
    if (!colored_map || colored_map->empty())
      return false;
    pcl::PCDWriter w;
    w.writeBinary(path, *colored_map);
    return true;
  }

  void Engine::maintainColoredMap_()
  {
    if (!colored_map || colored_map->empty())
      return;

    // 1) Hard cap: keep last color_max_points_ points (chronological tail)
    if (color_max_points_ > 0 && colored_map->size() > color_max_points_)
    {
      const size_t keep = color_max_points_;
      PointCloudXYZRGB::Ptr trimmed(new PointCloudXYZRGB());
      trimmed->reserve(keep);

      // Copy tail
      auto &v = colored_map->points;
      const size_t start = v.size() - keep;
      trimmed->points.insert(trimmed->points.end(), v.begin() + start, v.end());
      trimmed->width = (uint32_t)trimmed->points.size();
      trimmed->height = 1;
      trimmed->is_dense = false;

      colored_map.swap(trimmed);
    }

    // 2) Periodic voxel downsample
    if (color_ds_every_n_ > 0 && color_voxel_leaf_ > 0.0)
    {
      if (color_frame_count_ % (uint64_t)color_ds_every_n_ == 0)
      {
        pcl::VoxelGrid<PointTypeRGB> vg;
        vg.setLeafSize((float)color_voxel_leaf_, (float)color_voxel_leaf_, (float)color_voxel_leaf_);
        vg.setInputCloud(colored_map);

        PointCloudXYZRGB::Ptr filtered(new PointCloudXYZRGB());
        vg.filter(*filtered);
        filtered->is_dense = false;

        colored_map.swap(filtered);

        // Re-apply hard cap after downsample (usually not needed but safe)
        if (color_max_points_ > 0 && colored_map->size() > color_max_points_)
        {
          const size_t keep = color_max_points_;
          PointCloudXYZRGB::Ptr trimmed(new PointCloudXYZRGB());
          trimmed->reserve(keep);
          auto &v = colored_map->points;
          const size_t start = v.size() - keep;
          trimmed->points.insert(trimmed->points.end(), v.begin() + start, v.end());
          trimmed->width = (uint32_t)trimmed->points.size();
          trimmed->height = 1;
          trimmed->is_dense = false;
          colored_map.swap(trimmed);
        }
      }
    }
  }

} // namespace fastlivo2_core
