#pragma once
#include "fastlivo2_core/Export.hpp"
#include "fastlivo2_core/Types.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>
#include <string>
#include <array>

namespace fastlivo2_core
{

  class FASTLIVO2_CORE_API FastLivo2Core
  {
  public:
    FastLivo2Core();
    ~FastLivo2Core();

    FastLivo2Core(const FastLivo2Core &) = delete;
    FastLivo2Core &operator=(const FastLivo2Core &) = delete;

    // Init from a YAML path (we'll wire it up in phase 2)
    bool init(const std::string &yaml_path);

    void setImageEnabled(bool en);

    bool setCameraPinhole(int width, int height, double fx, double fy, double cx, double cy);
    bool setCameraPinholeDistorted(int width, int height,
                                   double fx, double fy, double cx, double cy,
                                   const std::array<double, 5> &dist_k1k2p1p2k3);

    bool setLidarToCameraExtrinsic(const std::array<double, 9> &R_cl_rowmajor,
                                   const std::array<double, 3> &t_cl);
    bool setImuToLidarExtrinsic(const std::array<double, 9> &R_il_rowmajor,
                                const std::array<double, 3> &t_il);
    // Feed sensors
    void pushImu(const ImuSample &s);
    void pushLidar(const LidarScan &s);
    void pushImage(const ImageFrame &f);

    // Process buffered data (returns true if something was processed)
    bool spinOnce();

    bool configureColoredMapMaintenance(int downsample_every_n_frames,
                                        size_t max_map_points,
                                        double voxel_leaf_size_m);
    // Outputs
    Eigen::Isometry3d T_wb() const; // pose world<-body
    std::vector<ColoredPoint> getColoredMap() const;

    // Return a snapshot of the LiDAR map (works even without images)
    pcl::PointCloud<pcl::PointXYZI>::Ptr getMap() const;

    bool saveColoredMapPLY(const std::string &path) const;
    bool saveColoredMapPCD(const std::string &path) const;

    // Reset: clear buffers and restart. If clear_map=true, also clears voxel + colored map.
    void reset(bool clear_map = true);
    // Get current pose (returns false if not initialized / no frame processed yet)
    bool getPose(Pose &out) const;

    bool getVelocity(Eigen::Vector3d &out_vel) const;
    bool getBiases(fastlivo2_core::Biases &out_biases) const;
    bool getStateDebug(fastlivo2_core::StateDebug &out_dbg) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

} // namespace fastlivo2_core
