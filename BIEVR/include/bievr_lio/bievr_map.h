#ifndef BIEVR_LIO_BIEVR_MAP_H_
#define BIEVR_LIO_BIEVR_MAP_H_

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include <list>

#include "bievr_lio/common.h"
#include "unordered_dense/unordered_dense.h"

namespace bievr {

struct Voxel {
  bool observed_{false};  // We consider a voxel observed if we have seen enough points inside it
  Transform T_C_W_ = Transform::Identity();
  Transform T_O_W_ = Transform::Identity();
  Eigen::MatrixXf bump_img_;
  Eigen::MatrixXf bump_smoothed_;
  Eigen::MatrixXf bump_weights_;
  // Timestamp (s) of the most recent hit each pixel received. Same shape as bump_img_, kept
  // in lockstep with it (resized/carried over together in reprojectImage) so a pixel that
  // stops getting confirmed can be told apart from one that never existed.
  Eigen::MatrixXd last_seen_s_;
  M3 outer_sum_ = Eigen::Matrix3d::Zero();
  V3 sum_ = Eigen::Vector3d::Zero();
  size_t num_points_{0};
  double mean_img_dist_{0.0};
  // Raw points accumulated while the voxel is not yet observed (no normal yet, so they cannot be
  // projected into a bump image). Cleared once the voxel becomes observed.
  std::vector<Eigen::Vector4d> pending_points_;
};

class BIEVRMap {
 public:
  struct Config {
    size_t max_size{10000000};  // maximum number of voxels to store
    double voxel_size{0.5};     // voxel side length
    double px_size{0.05};       // size of bump image pixels
    bool weighted = false;      // use range weighted update for bump image
    bool smooth = false;        // apply gaussian smoothing to bump image
    double norm_tol_deg{3.0};   // if normal changes more than this, reproject bump image
    // Seconds a pixel can go without a fresh hit before it is *eligible* to be cleared the
    // next time its voxel is touched. Lets terrain that has genuinely changed (e.g. fine
    // sand reshaped by wheels) overwrite the old height instead of being averaged against
    // it forever. <= 0 disables decay entirely.
    double stale_timeout_s{0.0};
    // A pixel that is stale by the rule above is only actually cleared if this scan's point
    // count in the voxel is at least this fraction of the voxel's currently-observed pixel
    // count. LiDAR point density is not uniform across the map -- the same physical spot
    // gets far fewer points from long range or a grazing angle than it did when the surface
    // was first built -- so a sparse look proves nothing about whether an unconfirmed pixel
    // is actually gone, only that this particular scan did not resample it. Only a look at
    // least as dense as whatever built the existing surface is trusted as evidence of that.
    double stale_min_relative_density{0.5};
  };

  explicit BIEVRMap(Config config);

  bool integratePoints(const Pointcloud& input_cloud, const std::vector<double>* ranges = nullptr,
                       double time_s = 0.0);

  inline size_t hashIndex(const Point& point) const {
    Eigen::Vector3i voxel_idx = (point * inv_voxel_size_).array().floor().cast<int>();

    return hashIndexVoxel(voxel_idx);
  }
  size_t size() const { return map_.size(); }

  /*** Every observed voxel's surface as world points.
   *
   * The map stores a height image per voxel rather than points, so this is the exact
   * inverse of how points went in. integratePoints writes bump_img_(y, x) at
   * x = round(p_O.x / px_size), y = round(p_O.y / px_size) with p_O = T_C_W_ * p_W, so a
   * pixel comes back out as
   *
   *   p_W = T_C_W_^-1 * (x * px_size, y * px_size, bump_smoothed_(y, x))
   *
   * bump_smoothed_ and bump_weights_ > 0 rather than the raw image, because that pair is
   * precisely what registration samples (getSubPixelValue) -- so what this returns is the
   * surface the filter actually believes in, not an approximation of it.
   *
   * stride > 1 keeps every nth pixel along both axes, which is the cheap way to trade map
   * resolution for message size and extraction time.
   ***/
  void extractSurface(Pointcloud& out, int stride = 1) const;
  Eigen::Vector3i getVoxelIdx(const Point& point) const;
  const Voxel* getVoxel(const size_t hash_idx) const;
  bool nearestVoxel(const Point& point, size_t& result) const;

  const double& voxel_size = config_.voxel_size;
  const double& pixel_size = config_.px_size;
  const double& inv_px_size = inv_px_size_;

 private:
  struct ImageBounds {
    int width;
    int height;
    double u_min;
    double v_min;
  };

  // Voxel update / bump-image pipeline, in the order integratePoints invokes them.
  bool updateNormal(Voxel& voxel);

  bool updateBumpImage(const std::vector<Eigen::Vector4d>& points, Voxel& voxel,
                       bool normal_change, double time_s);

  ImageBounds computeImageSize(const Voxel& voxel, const Point& reference_point) const;

  void reprojectImage(Voxel& voxel, const ImageBounds& bounds, Eigen::MatrixXi& changed);

  void integratePoints(const std::vector<Eigen::Vector4d>& points, Voxel& voxel,
                       Eigen::MatrixXi& changed, double time_s);

  // Clears (zeroes the weight of) any pixel in the voxel's bump image that has not been hit
  // within config_.stale_timeout_s of time_s, but only if points_this_scan shows this
  // update looked at the voxel densely enough (config_.stale_min_relative_density) to trust
  // an absence as real rather than as an artefact of range/angle-dependent point spacing.
  // Runs before this scan's points are integrated, so a pixel hit again this frame starts a
  // fresh average instead of blending into a stale one. No-op when stale_timeout_s <= 0.
  void decayStalePixels(Voxel& voxel, double time_s, size_t points_this_scan);

  void dilateMask(const Eigen::MatrixXi& changed, const Eigen::MatrixXf& weights,
                  Eigen::MatrixXi& changed_dilated);

  void maskedGaussianSmooth(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                            const Eigen::MatrixXi& changed, Eigen::MatrixXf& image_smooth);

  void computeScore(Voxel& voxel);

  // Geometry helper.
  Eigen::Vector3d getVoxelOrigin(const Point& point) const;

  Eigen::MatrixXf gauss_kernel_;
  Config config_;
  double inv_voxel_size_{1.0};
  double inv_px_size_{0.05};            // inv size of bump image
  double norm_tol_rad_{3.0};            // if normal changes more than this, update bump image
  static constexpr int kNMinValid = 4;  // minimum points for a valid voxel

  struct VoxelEntry {
    Voxel voxel;
    std::list<size_t>::iterator lru_it;  // iterator into voxels_cache_
  };
  ankerl::unordered_dense::map<size_t, VoxelEntry> map_;
  std::list<size_t> voxels_cache_;

  Eigen::MatrixXd corner_offsets_;
  std::vector<Point> neighbor_offsets_;
};

using VoxelPtr = std::shared_ptr<Voxel>;
using ConstVoxelPtr = std::shared_ptr<const Voxel>;

}  // namespace bievr
#endif  // BIEVR_LIO_BIEVR_MAP_H_