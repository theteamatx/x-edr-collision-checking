#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VOXEL_INDEXER_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VOXEL_INDEXER_H_

#include <limits>

#include "collision_checking/geometry.h"
#include "collision_checking/logging.h"
#include "collision_checking/eigenmath.h"
#include "third_party/eigen3/Eigen/Core"

namespace collision_checking {

// A class for discretizing floating point coordinates.
template <typename FloatType, typename IntType>
class VoxelIndexer {
 public:
  VoxelIndexer() = default;
  VoxelIndexer(const VoxelIndexer<FloatType, IntType>&) = default;

  // Construct a VoxelIndexer for coordinates within `box` with a given
  // cell_width.
  explicit VoxelIndexer(const AlignedBox<FloatType>& box, FloatType cell_width);

  // Returns the smallest valid cell width.
  static FloatType MinCellWidth(const Vector3<FloatType>& box_width);

  // Returns the cell index for `point`.
  Vector3<IntType> ToCell(
      const Vector3<FloatType>& point) const;

  // Returns the point for `index` with the lowest coordinate values in all
  // dimensions, where 'lowest' means closest to
  // std::numeric_limits<FloatType>::lowest().
  Vector3<FloatType> ToPoint(
      const Vector3<IntType>& cell) const;

 private:
  Eigen::Array3<FloatType> ToCellFloat(
      const Vector3<FloatType>& point) const;
  AlignedBox<FloatType> box_;
  FloatType cell_width_;
  FloatType cell_width_inverse_;
  Eigen::Array3<FloatType> max_cell_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
namespace voxel_indexer_details {
template <typename FloatType>
FloatType CheckedInverse(const FloatType value) {
  CC_CHECK_GT(value, FloatType{0}, "cell_width must be positive.");
  return FloatType{1} / value;
}
}  // namespace voxel_indexer_details

template <typename FloatType, typename IntType>
VoxelIndexer<FloatType, IntType>::VoxelIndexer(const AlignedBox<FloatType>& box,
                                               const FloatType cell_width)
    : box_(box),
      cell_width_(cell_width),
      cell_width_inverse_(voxel_indexer_details::CheckedInverse(cell_width)) {
  static_assert(std::is_floating_point_v<FloatType>,
                "FloatType must be a floating point type.");

  max_cell_ = (ToCellFloat(box_.high) - FloatType{0.5}).floor();
  const FloatType kMaxIndexAsFloat{max_cell_.maxCoeff()};
  CC_CHECK_LE(kMaxIndexAsFloat, std::numeric_limits<IntType>::max(),
             "Too many cells for IntType (kMaxIndexAsFloat: %.18e)",
             kMaxIndexAsFloat);
}

template <typename FloatType, typename IntType>
FloatType VoxelIndexer<FloatType, IntType>::MinCellWidth(
    const Vector3<FloatType>& box_width) {
  return (box_width / (FloatType{std::numeric_limits<IntType>::max()}))
      .maxCoeff();
}

template <typename FloatType, typename IntType>
Eigen::Array3<FloatType> VoxelIndexer<FloatType, IntType>::ToCellFloat(
    const Vector3<FloatType>& point) const {
  return (point.array() - box_.low.array()) * cell_width_inverse_ +
         FloatType{0.5};
}

template <typename FloatType, typename IntType>
Vector3<IntType> VoxelIndexer<FloatType, IntType>::ToCell(
    const Vector3<FloatType>& point) const {
  return (ToCellFloat(point).min(max_cell_).max(FloatType{0}))
      .template cast<IntType>();
}

template <typename FloatType, typename IntType>
Vector3<FloatType> VoxelIndexer<FloatType, IntType>::ToPoint(
    const Vector3<IntType>& cell) const {
  return box_.low + cell.template cast<FloatType>() * cell_width_;
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VOXEL_INDEXER_H_
