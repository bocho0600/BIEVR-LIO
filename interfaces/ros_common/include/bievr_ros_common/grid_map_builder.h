#ifndef BIEVR_ROS_COMMON_GRID_MAP_BUILDER_H_
#define BIEVR_ROS_COMMON_GRID_MAP_BUILDER_H_

// Builds a 2.5D elevation grid map from the near-field bump-image voxels of the
// map. This lives in bievr_ros_common rather than the core library so that
// `bievr_lio` stays free of any grid_map dependency, and once here rather than
// in each wrapper because grid_map_core is itself ROS-version-agnostic.
//
// Only compiled when the consumer defines BIEVR_WITH_GRID_MAP, which the ROS1
// and ROS2 CMakeLists set when the optional grid_map packages are found.

#ifdef BIEVR_WITH_GRID_MAP

#include <grid_map_core/grid_map_core.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "bievr_lio/bievr_map.h"

namespace bievr {

namespace grid_internal {

inline std::vector<grid_map::Index> makeCircleMask(double radius, double resolution) {
  int r = static_cast<int>(std::ceil(radius / resolution));
  std::vector<grid_map::Index> mask;
  mask.reserve((2 * r + 1) * (2 * r + 1));
  for (int dx = -r; dx <= r; ++dx) {
    for (int dy = -r; dy <= r; ++dy) {
      if (std::sqrt(dx * dx + dy * dy) * resolution <= radius) {
        mask.emplace_back(dx, dy);
      }
    }
  }
  return mask;
}

}  // namespace grid_internal

/// Build a 2.5D elevation grid map from the near-field bump-image voxels of the
/// map around the given position. The cells hold world-frame heights; the caller
/// sets the frame id and timestamp before publishing.
inline void createGridMap(const BIEVRMap& map, const V3& position, const double length,
                          const double grid_res, grid_map::GridMap& grid_map) {
  std::string layer = "elevation";
  grid_map.setGeometry(grid_map::Length(length, length), grid_res);
  grid_map.add(layer);
  grid_map.setPosition(position.head<2>());

  double map_res = map.voxel_size;

  int voxel_steps = static_cast<int>(std::ceil((length / 2) / map_res));
  int z_steps_up = 2;
  int z_steps_down = 3;
  Eigen::Vector3i center_idx = map.getVoxelIdx(position);

  std::vector<const Voxel*> voxels;
  voxels.reserve((2 * voxel_steps + 1) * (2 * voxel_steps + 1) * (z_steps_down + z_steps_up + 1));
  for (int x_idx = center_idx.x() - voxel_steps; x_idx <= center_idx.x() + voxel_steps; ++x_idx) {
    for (int y_idx = center_idx.y() - voxel_steps; y_idx <= center_idx.y() + voxel_steps; ++y_idx) {
      for (int z_idx = center_idx.z() - z_steps_down; z_idx <= center_idx.z() + z_steps_up;
           ++z_idx) {
        const Voxel* voxel_ptr = map.getVoxel(hashIndexVoxel(Eigen::Vector3i(x_idx, y_idx, z_idx)));
        if (voxel_ptr) {
          voxels.push_back(voxel_ptr);
        }
      }
    }
  }

  const double res = map.pixel_size;
  const double res_half = res / 2.0;
  // GridMap::at() hashes the layer name on every call, so resolve the layer once
  // rather than once per emitted point.
  grid_map::Matrix& data = grid_map[layer];

  for (const Voxel* voxel_ptr : voxels) {
    const Transform T_W_C = voxel_ptr->T_C_W_.inverse();
    const int rows = voxel_ptr->bump_img_.rows();
    const int cols = voxel_ptr->bump_img_.cols();
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        if (voxel_ptr->bump_weights_(i, j) <= 0) continue;  // Skip invalid pixels
        const double depth = voxel_ptr->bump_smoothed_(i, j);

        for (int k = -1; k < 2; ++k) {
          Point p_pix = Point(j * res + k * res_half, i * res + k * res_half, depth);
          Point p_world = T_W_C * p_pix;  // Transform to world coordinates

          grid_map::Index idx;
          if (grid_map.getIndex(Eigen::Vector2d(p_world.x(), p_world.y()), idx)) {
            float& cell_value = data(idx(0), idx(1));

            if (!std::isfinite(cell_value)) {
              // Cell is empty / uninitialized
              cell_value = p_world.z();
            } else {
              // Cell already has a value
              cell_value = std::max(cell_value, static_cast<float>(p_world.z()));
            }
          }
        }
      }
    }
  }

  double radius = 0.05;
  auto circleMask = grid_internal::makeCircleMask(radius, grid_res);  // compute once

  const int rows = data.rows();
  const int cols = data.cols();

  // Applied only after the sweep: filling in place would let a just-filled cell
  // act as a valid neighbour, flooding holes in sweep order instead of filling
  // only their rim.
  std::vector<std::pair<int, float>> infill;  // column-major linear index -> value

  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      if (std::isfinite(data(i, j))) continue;
      double sum = 0.0;
      int count = 0;

      // Apply precomputed circle mask
      for (const auto& offset : circleMask) {
        const int ni = i + offset[1];  // row
        const int nj = j + offset[0];  // col

        if (ni < 0 || ni >= rows || nj < 0 || nj >= cols) continue;
        const float neighbour = data(ni, nj);
        if (!std::isfinite(neighbour)) continue;

        sum += neighbour;
        count++;
      }

      if (count > 0) {
        infill.emplace_back(j * rows + i, static_cast<float>(sum / count));
      }
    }
  }

  for (const auto& [index, value] : infill) {
    data.data()[index] = value;
  }
}

}  // namespace bievr

#endif  // BIEVR_WITH_GRID_MAP
#endif  // BIEVR_ROS_COMMON_GRID_MAP_BUILDER_H_
