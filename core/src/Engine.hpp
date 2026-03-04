#pragma once
#include "fastlivo2_core/Types.hpp"

#include <array>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <vikit/pinhole_camera.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "common_lib.h" // MeasureGroup / LidarMeasureGroup / PointCloudXYZI / PointType
#include "IMU_Processing.h"
#include "voxel_map.h"
#include "vio.h"

namespace fastlivo2_core
{

  class Engine
  {
  public:
    Engine();

    bool init();

    void setImageEnabled(bool en)
    {
      img_enabled_ = en;
    };
    bool setCameraPinhole(int width, int height, double fx, double fy, double cx, double cy);
    bool setCameraPinholeDistorted(int width, int height,
                                   double fx, double fy, double cx, double cy,
                                   const std::array<double, 5> &dist_k1k2p1p2k3);

    bool setLidarToCameraExtrinsic(const std::array<double, 9> &R_cl_rowmajor,
                                   const std::array<double, 3> &t_cl);
    bool setImuToLidarExtrinsic(const std::array<double, 9> &R_il_rowmajor,
                                const std::array<double, 3> &t_il);

    void pushImu(const ImuSample &s);
    void pushLidar(const LidarScan &s);
    void pushImage(const ImageFrame &img);

    // Build one synced LidarMeasureGroup (like LIVMapper::sync_packages).
    bool spinOnce();

    bool configureColoredMapMaintenance(int downsample_every_n_frames,
                                        size_t max_map_points,
                                        double voxel_leaf_size_m);

    PointCloudXYZRGB::Ptr getColoredMap() const { return colored_map; }
    PointCloudXYZRGB::Ptr getColoredFrame() const { return colored_frame; }
    bool saveColoredMapPLY(const std::string &path) const;
    bool saveColoredMapPCD(const std::string &path) const;

    PointCloudXYZI::Ptr getUndistortedCloud() const { return feats_undistort; }
    const StatesGroup &getState() const { return _state; }

    void reset(bool clear_map = true);
    bool getPose(fastlivo2_core::Pose &out) const;

    bool getVelocity(Eigen::Vector3d &out_vel) const;
    bool getBiases(fastlivo2_core::Biases &out_biases) const;
    bool getStateDebug(fastlivo2_core::StateDebug &out_dbg) const;

  private:
    // ======== data buffering ========
    struct LidarItem
    {
      double t0 = 0, t1 = 0;
      PointCloudXYZI::Ptr cloud;
    };
    struct ImgItem
    {
      double t = 0.0;
      cv::Mat bgr; // CV_8UC3
    };

    mutable std::mutex mtx_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buf_;
    std::deque<LidarItem> lidar_buf_;
    std::deque<ImgItem> img_buf_;

    // ======== LIVMapper-like state ========
    LidarMeasureGroup LidarMeasures; // same name as upstream, easier port
    StatesGroup _state;              // upstream state
    StatesGroup state_propagat;      // upstream
    bool is_first_frame = false;
    double _first_lidar_time = -1.0;

    // managers
    std::shared_ptr<VoxelMapManager> voxelmap_manager;
    std::shared_ptr<VIOManager> vio_manager;

    // point clouds (same as upstream names)
    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr feats_down_body;
    PointCloudXYZI::Ptr feats_down_world;
    PointCloudXYZI::Ptr pcl_w_wait_pub;

    // filters
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    double filter_size_surf_min = 0.5; // default from readParameters
    bool dense_map_en = false;

    // VIO/LIO tuning (defaults)
    int grid_size = 5;
    int grid_n_width = 0; // set later if you need
    int grid_n_height = 17;
    int patch_pyrimid_level = 3;
    int patch_size = 8;
    int max_iterations = 5;
    double IMG_POINT_COV = 100.0;
    double outlier_threshold = 1000.0;

    bool normal_en = true;
    bool inverse_composition_en = false;
    bool raycast_en = false;
    bool exposure_estimate_en = true;
    double inv_expo_cov = 0.2;

    // time offsets (defaults)
    double exposure_time_init = 0.0;
    double img_time_offset = 0.0;
    double imu_time_offset = 0.0;
    double lidar_time_offset = 0.0;

    // IMU
    std::unique_ptr<ImuProcess> imu_proc_;
    bool gravity_align_en = false;
    bool gravity_align_finished = false;

    // extrinsics (defaults identity)
    V3D extT = V3D(0, 0, 0);
    M3D extR = M3D::Identity();
    std::vector<double> cameraextrinT{0, 0, 0};
    std::vector<double> cameraextrinR{1, 0, 0, 0, 1, 0, 0, 0, 1};

    // map init flag
    bool lidar_map_inited = false;

    // ======== core functions (ported) ========
    bool sync_packages(LidarMeasureGroup &meas);
    void handleFirstFrame();
    void gravityAlignment();
    void processImu();
    void stateEstimationAndMapping();
    void handleLIO();
    void handleVIO();

    // helpers (ported)
    void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                        const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
    void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);

    // conversions from your public API
    static sensor_msgs::Imu::ConstPtr makeImuMsg(const ImuSample &s);
    static PointCloudXYZI::Ptr makeCloud(const LidarScan &scan);
    static cv::Mat makeBgrCopy(const ImageFrame &img);

    // Keep a copy of voxel config so we can reconstruct voxelmap_manager on reset
    VoxelMapConfig voxel_config_;

    // Backing voxel map storage (passed by ref into VoxelMapManager ctor)
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;

    std::unique_ptr<vk::PinholeCamera> cam_owner_;
    bool vio_initialized_ = false;

    // Colorization settings (same semantics as publish_frame_world)
    int pub_scan_num_ = 1; // accumulate N scans before sampling
    int pub_num_ = 0;
    double blind_rgb_points_ = 0.01; // drop too-close points

    PointCloudXYZI::Ptr pcl_wait_pub{new PointCloudXYZI()};      // accumulates world XYZ(I)
    PointCloudXYZRGB::Ptr colored_map{new PointCloudXYZRGB()};   // accumulated colored map
    PointCloudXYZRGB::Ptr colored_frame{new PointCloudXYZRGB()}; // last colored frame (optional)

    int pub_scan_num = 1;
    int pub_num = 0;
    double blind_rgb_points = 0.01;

    // ---- colored map maintenance ----
    int color_ds_every_n_ = 30;           // downsample every N *colorized* frames (0 disables)
    size_t color_max_points_ = 2'000'000; // 0 disables cap
    double color_voxel_leaf_ = 0.10;      // meters (<=0 disables voxel filter)
    uint64_t color_frame_count_ = 0;      // increments when a colored frame is produced

    void maintainColoredMap_();

    bool img_enabled_ = false;
  };

} // namespace fastlivo2_core
