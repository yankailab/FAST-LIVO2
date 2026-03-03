#pragma once
#include "fastlivo2_core/Export.hpp"
#include "fastlivo2_core/Types.hpp"
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

    bool saveColoredMapPLY(const std::string &path) const;
    bool saveColoredMapPCD(const std::string &path) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

} // namespace fastlivo2_core
