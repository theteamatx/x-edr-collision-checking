// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VOXEL_MAP_OBJECT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VOXEL_MAP_OBJECT_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "absl/algorithm/container.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/object_id.h"
#include "collision_checking/voxel_code.h"
#include "collision_checking/voxel_indexer.h"

namespace collision_checking {

// One node in the VoxelMapObject (see below).
// Holds the geometry (currently always one sphere), along with the hash code
// for linearizing the three dimensional geometry.
template <typename Scalar>
struct VoxelMapNode {
  uint32_t code = 0;
  Sphere<Scalar> sphere;
  int index = 0;
  // The id of the object/semantic type this voxel is part of.
  ObjectId object_id = kDefaultVoxelMapId;
};

template <typename Scalar>
bool operator<(const VoxelMapNode<Scalar>& left,
               const VoxelMapNode<Scalar>& right) {
  return left.code < right.code;
}

template <typename Scalar>
bool operator<(const VoxelMapNode<Scalar>& left, const uint32_t right) {
  return left.code < right;
}

template <typename Scalar>
bool operator<(const uint32_t left, const VoxelMapNode<Scalar>& right) {
  return left < right.code;
}

// A class that provides an interface for iterating over the spheres contained
// in a range of VoxelMapNode objects.
// See VoxelMapObject::GetVoxelRange() for a usage example.
template <typename Scalar>
class VoxelRange {
 public:
  // Iterator class over an array of VoxelMapNode objects that gives access
  // to the nested `sphere` object.
  class Iterator;

  using const_iterator = Iterator;

  // Construct a `VoxelRange` [`begin`, `end`).
  VoxelRange(const VoxelMapNode<Scalar>* begin,
             const VoxelMapNode<Scalar>* end);
  // Returns the `begin` iterator.
  const_iterator begin() const;
  // Returns the `end` iterator.
  const_iterator end() const;

 private:
  const_iterator begin_;
  const_iterator end_;
};

// A class that provides an interface for iterating over the spheres contained
// in a range of VoxelMapNode objects that overlap or are contained within a
// given axis aligned bounding box. See VoxelMapObject::GetInBoxVoxelRange() for
// a usage example.
// TODO: Further optimize bounding box range query.
template <typename Scalar>
class InBoxVoxelRange {
 public:
  // Iterator class over an array of VoxelMapNode objects that gives access
  // to the nested `sphere` object and skips spheres that don't overlap the
  // bounding box.
  class Iterator;

  using const_iterator = Iterator;

  InBoxVoxelRange(const AlignedBox<Scalar>& box,
                  const VoxelMapNode<Scalar>* begin,
                  const VoxelMapNode<Scalar>* end);

  const_iterator begin() const;

  const_iterator end() const;

 private:
  static Iterator GetBegin(const AlignedBox<Scalar>& box,
                           const VoxelMapNode<Scalar>* begin,
                           const VoxelMapNode<Scalar>* end);
  const AlignedBox<Scalar> box_;
  const_iterator begin_;
  const_iterator end_;
};

// A collision object that is usually generated from a VoxelMap.
// Effectively this is a special kind of CompositeObject that is treated
// differently in collision checks to improve efficiency, and provides some
// additional methods.
// It currently only supports sphere geometries.
// TODO: Consider extending this to support all static geometry
// efficiently.
template <typename Scalar>
class VoxelMapObject {
 public:
  // Options for the VoxelMapObject.
  struct Options {
    // Width of cells used for discretization of coordinates before
    // linearization. A larger cell width will be used if necessary.
    Scalar cell_width = Scalar{0.02};
    // If true, sort cells according to their voxel code and use that
    // to accelerate the InBoxVoxelRange iterator.
    // The current hash code is computed from the Morton (or z-order) curve.
    // If the call to UpdateQueryStructures() is in the critical path,
    // this might not be worthwhile, but should help if it is not.
    bool use_voxel_code = true;
    // Upper bound for voxel radii.
    Scalar max_sphere_radius = Scalar{0.2};
  };

  // Non real-time safe functions.

  // Allocates memory for `size` spheres and resets the valid range of nodes.
  void ResizeBuffers(std::size_t size);

  // Copies `other` to `this`.
  void CopyFrom(const VoxelMapObject<Scalar>& other);

  // Updates data for accelerated range queries via GetInBoxVoxelRange accoring
  // to `options`.
  // If `use_voxel_code` = true and `max_sphere_radius` is smaller than the
  // largest sphere radius that was added, collisions might be missed.
  void UpdateQueryStructures(Options options = {});

  // Real-time safe functions.

  // Returns the number of spheres.
  std::size_t size() const;

  // Returns the sphere capacity.
  std::size_t capacity() const;

  // Adds `sphere`. Will assert if the capacity has been exceeded.
  void AddSphere(const Sphere<Scalar>& sphere, int index);

  // Adds a sphere with `center` and `radius`.
  // Will assert if the capacity has been exceeded.
  void AddSphere(const Vector3<Scalar>& center, Scalar radius, int index,
                 ObjectId id = kDefaultVoxelMapId);

  // Adds a sphere with `center` and `radius`.
  // Will assert if the capacity has been exceeded.
  void AddSphere(absl::Span<const Scalar> center, Scalar radius, int index,
                 ObjectId id = kDefaultVoxelMapId);

  // Transforms all spheres accoring to:
  // new_center = translation + rotation * old_center.
  // This is only an optimization to account for the existing APIs in
  // 'WorldState' and 'CollisionWorld' and will be removed in the future.
  void TransformSpheres(const Vector3<Scalar>& translation,
                        const Matrix3<Scalar>& rotation);

  // Returns a reference to the ObjectFlags for *this.
  ObjectFlags& flags() { return flags_; }

  // Returns a const reference to the ObjectFlags for *this.
  const ObjectFlags& flags() const { return flags_; }

  // Returns `InBoxVoxelRange` for iterating over spheres that overlap `box`.
  // Example usage:
  // for (const auto& s: voxel_map_object.GetInBoxVoxelRange(some_box)) {
  //   // .. do something with `s`.
  // }
  InBoxVoxelRange<Scalar> GetInBoxVoxelRange(
      const AlignedBox<Scalar>& box) const;

  // Returns `VoxelRange` for iterating over all spheres/voxels.
  // Example usage:
  // for (const auto& s: voxel_map_object.GetVoxelRange()) {
  //   // .. do something with `s`.
  // }
  VoxelRange<Scalar> GetVoxelRange() const;

 private:
  void Reset();
  VoxelCodeTraits::CodeType Encode(const Vector3<Scalar>& point) const;
  Vector3<Scalar> Decode(VoxelCodeTraits::CodeType code) const;

  bool query_structures_are_valid_ = true;
  Options options_;
  ObjectFlags flags_ = {.id_set = kVoxelMapIdSet,
                        .inclusion_set = ~kVoxelMapIdSet};
  std::size_t used_nodes_ = 0;
  std::vector<VoxelMapNode<Scalar>> nodes_;
  // Set size to zero, so range queries don't produce overflow/underflow.
  AlignedBox<Scalar> bounding_box_{.low = Vector3<Scalar>::Zero(),
                                   .high = Vector3<Scalar>::Zero()};
  VoxelIndexer<Scalar, VoxelCodeTraits::IntCoordType> voxel_indexer_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

// VoxelRange implementation.
template <typename Scalar>
class VoxelRange<Scalar>::Iterator {
 public:
  // Creates an iterator from a given `ptr`.
  explicit Iterator(const VoxelMapNode<Scalar>* ptr) : ptr_(ptr) {}
  // Returns the sphere at the iterator position.
  const VoxelMapNode<Scalar>& operator*() const { return *ptr_; }
  Iterator& operator++() {
    ++ptr_;
    return *this;
  }
  Iterator& operator++(int) { return ++(*this); }
  friend bool operator==(const Iterator& a, const Iterator& b) {
    return a.ptr_ == b.ptr_;
  }
  friend bool operator!=(const Iterator& a, const Iterator& b) {
    return !(a.ptr_ == b.ptr_);
  }

 private:
  const VoxelMapNode<Scalar>* ptr_;
};

template <typename Scalar>
VoxelRange<Scalar>::VoxelRange(const VoxelMapNode<Scalar>* begin,
                               const VoxelMapNode<Scalar>* end)
    : begin_(begin), end_(end) {}

template <typename Scalar>
typename VoxelRange<Scalar>::const_iterator VoxelRange<Scalar>::begin() const {
  return const_iterator(begin_);
}
template <typename Scalar>
typename VoxelRange<Scalar>::const_iterator VoxelRange<Scalar>::end() const {
  return const_iterator(end_);
}

// InBoxVoxelRange implementation.
template <typename Scalar>
class InBoxVoxelRange<Scalar>::Iterator {
 public:
  // Creates an iterator from a given `box`, initial element `ptr` and `end`
  // pointer. `ptr` and `end` must point to a valid range of
  // VoxelMapNode<Scalar> objects.
  Iterator(const AlignedBox<Scalar>& box, const VoxelMapNode<Scalar>* ptr,
           const VoxelMapNode<Scalar>* end)
      : box_(box), ptr_(ptr), end_(end) {}

  // Dereference operator that returns the sphere at the iterator position.
  const VoxelMapNode<Scalar>& operator*() const { return *ptr_; }
  // Advances the iterator until an element overlaps with `box`, or the end of
  // the range is reached.
  Iterator& operator++() {
    while (true) {
      ++ptr_;
      if (ptr_ >= end_ || IsInside(box_, ptr_->sphere.center)) {
        break;
      }
    }
    return *this;
  }
  Iterator& operator++(int) { return ++(*this); }
  friend bool operator==(const Iterator& a, const Iterator& b) {
    return a.ptr_ == b.ptr_;
  }
  friend bool operator!=(const Iterator& a, const Iterator& b) {
    return !(a.ptr_ == b.ptr_);
  }

 private:
  const AlignedBox<Scalar>& box_;
  const VoxelMapNode<Scalar>* ptr_;
  const VoxelMapNode<Scalar>* end_;
};

template <typename Scalar>
typename InBoxVoxelRange<Scalar>::Iterator InBoxVoxelRange<Scalar>::GetBegin(
    const AlignedBox<Scalar>& box, const VoxelMapNode<Scalar>* begin,
    const VoxelMapNode<Scalar>* end) {
  // Advance `begin_` until it is within the specified box, or `end` is reached.
  while (begin != end && !IsInside(box, begin->sphere.center)) {
    ++begin;
  }
  return Iterator(box, begin, end);
}

template <typename Scalar>
InBoxVoxelRange<Scalar>::InBoxVoxelRange(const AlignedBox<Scalar>& box,
                                         const VoxelMapNode<Scalar>* begin,
                                         const VoxelMapNode<Scalar>* end)
    : box_(box), begin_(GetBegin(box_, begin, end)), end_(box_, end, end) {}

template <typename Scalar>
typename InBoxVoxelRange<Scalar>::const_iterator
InBoxVoxelRange<Scalar>::begin() const {
  return begin_;
}

template <typename Scalar>
typename InBoxVoxelRange<Scalar>::const_iterator InBoxVoxelRange<Scalar>::end()
    const {
  return end_;
}

// VoxelMapObject implementation.
template <typename Scalar>
void VoxelMapObject<Scalar>::ResizeBuffers(std::size_t size) {
  nodes_.resize(size);
  used_nodes_ = 0;
}

template <typename Scalar>
void VoxelMapObject<Scalar>::CopyFrom(const VoxelMapObject<Scalar>& other) {
  ResizeBuffers(other.size());
  nodes_ = other.nodes_;
  used_nodes_ = other.used_nodes_;
}

template <typename Scalar>
void VoxelMapObject<Scalar>::UpdateQueryStructures(Options options) {
  query_structures_are_valid_ = true;
  Reset();
  options_ = options;
  if (nodes_.empty()) {
    return;
  }

  if (options.use_voxel_code) {
    if (nodes_.empty()) {
      bounding_box_.high.setZero();
      bounding_box_.low.setZero();
    } else {
      SetEmptySize(&bounding_box_);
      for (const auto& n : nodes_) {
        GrowAlignedBoxAround(n.sphere, &bounding_box_);
      }
    }
    const Scalar min_voxel_size =
        voxel_indexer_.MinCellWidth(bounding_box_.high - bounding_box_.low);

    const Scalar cell_width = std::max(min_voxel_size, options_.cell_width);
    voxel_indexer_ = VoxelIndexer<Scalar, VoxelCodeTraits::IntCoordType>(
        bounding_box_, cell_width);

    // Update z-curve values & sort spheres.
    for (auto& n : nodes_) {
      // TODO: Account for geometry extent, update bvh.
      n.code = Encode(n.sphere.center);
    }
    absl::c_stable_sort(nodes_);
  }
}

template <typename Scalar>
std::size_t VoxelMapObject<Scalar>::size() const {
  return used_nodes_;
}

template <typename Scalar>
std::size_t VoxelMapObject<Scalar>::capacity() const {
  return nodes_.size();
}

template <typename Scalar>
void VoxelMapObject<Scalar>::AddSphere(const Sphere<Scalar>& sphere,
                                       int index) {
  CC_CHECK_LT(used_nodes_, nodes_.size());
  nodes_[used_nodes_].sphere = sphere;
  nodes_[used_nodes_].index = index;
  ++used_nodes_;
  query_structures_are_valid_ = false;
}

template <typename Scalar>
void VoxelMapObject<Scalar>::AddSphere(const Vector3<Scalar>& center,
                                       const Scalar radius, int index,
                                       ObjectId id) {
  CC_CHECK_LT(used_nodes_, nodes_.size(), "used_nodes_= %d, nodes_.size()= %zu",
              used_nodes_, nodes_.size());
  nodes_[used_nodes_].sphere.center = center;
  nodes_[used_nodes_].sphere.radius = radius;
  nodes_[used_nodes_].index = index;
  nodes_[used_nodes_].object_id = id;
  ++used_nodes_;
  query_structures_are_valid_ = false;
}

template <typename Scalar>
void VoxelMapObject<Scalar>::AddSphere(absl::Span<const Scalar> center,
                                       const Scalar radius, int index,
                                       ObjectId id) {
  CC_CHECK_LT(used_nodes_, nodes_.size());
  nodes_[used_nodes_].sphere.center[0] = center[0];
  nodes_[used_nodes_].sphere.center[1] = center[1];
  nodes_[used_nodes_].sphere.center[2] = center[2];
  nodes_[used_nodes_].sphere.radius = radius;
  nodes_[used_nodes_].index = index;
  nodes_[used_nodes_].object_id = id;
  ++used_nodes_;
  query_structures_are_valid_ = false;
}

template <typename Scalar>
void VoxelMapObject<Scalar>::TransformSpheres(
    const Vector3<Scalar>& translation, const Matrix3<Scalar>& rotation) {
  for (auto& node : nodes_) {
    node.sphere.center = translation + rotation * node.sphere.center;
  }
  query_structures_are_valid_ = false;
}

template <typename Scalar>
InBoxVoxelRange<Scalar> VoxelMapObject<Scalar>::GetInBoxVoxelRange(
    const AlignedBox<Scalar>& box) const {
  if (nodes_.empty()) {
    return InBoxVoxelRange<Scalar>(box, nodes_.data(),
                                   nodes_.data() + nodes_.size());
  }

  if (options_.use_voxel_code == false ||
      query_structures_are_valid_ == false) {
    // Enlarge box with max radius.
    const AlignedBox<Scalar> enlarged_box{
        .low = box.low.array() - options_.max_sphere_radius,
        .high = box.high.array() + options_.max_sphere_radius};
    return InBoxVoxelRange<Scalar>(enlarged_box, nodes_.data(),
                                   nodes_.data() + nodes_.size());
  }

  // Enlarge box with max radius, then saturate to voxel map size
  const AlignedBox<Scalar> enlarged_box{
      .low = (box.low.array() - options_.max_sphere_radius)
                 .cwiseMax(bounding_box_.low.array()),
      .high = (box.high.array() + options_.max_sphere_radius)
                  .cwiseMin(bounding_box_.high.array())};

  auto low_cell = voxel_indexer_.ToCell(enlarged_box.low);
  auto high_cell = voxel_indexer_.ToCell(enlarged_box.high);
  // Narrow the range passed to InBoxVoxelRange to the node range from
  // the lower and upper corners of the bounding box.
  // Nodes with codes higher than code_high or lower than code_low are all
  // outside the bounding box.
  // TODO: In general, most cells with codes in [code_low,
  // code_high] are also outside the bounding box. Consider finding the valid
  // subranges in the space of voxel codes instead of performing a bounding box
  // check for all nodes in [code_low, code_high].
  const auto code_low = VoxelIndexToCode(low_cell);
  const auto code_high = VoxelIndexToCode(high_cell);

  // Get index before or at code_low.
  auto low_it = absl::c_lower_bound(nodes_, code_low);
  const std::ptrdiff_t index_low =
      std::max<std::ptrdiff_t>(low_it - nodes_.begin(), 0);
  // Get index after code_high.
  auto end_it = std::upper_bound(low_it, nodes_.end(), code_high);
  const std::ptrdiff_t index_end =
      std::min<std::ptrdiff_t>(end_it - nodes_.begin(), nodes_.size());

  return InBoxVoxelRange<Scalar>(enlarged_box, nodes_.data() + index_low,
                                 nodes_.data() + index_end);
}

template <typename Scalar>
VoxelRange<Scalar> VoxelMapObject<Scalar>::GetVoxelRange() const {
  return VoxelRange<Scalar>(nodes_.data(), nodes_.data() + nodes_.size());
}

template <typename Scalar>
void VoxelMapObject<Scalar>::Reset() {
  bounding_box_.high.setZero();
  bounding_box_.low.setZero();
}

template <typename Scalar>
VoxelCodeTraits::CodeType VoxelMapObject<Scalar>::Encode(
    const Vector3<Scalar>& point) const {
  return VoxelIndexToCode(voxel_indexer_.ToCell(point));
}

template <typename Scalar>
Vector3<Scalar> VoxelMapObject<Scalar>::Decode(
    VoxelCodeTraits::CodeType code) const {
  voxel_indexer_.ToPoint(CodeToVoxelIndex(code));
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VOXEL_MAP_OBJECT_H_
