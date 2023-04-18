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

// This file collects functions to convert voxel/cell indices into hash codes
// and vice versa.
// Currently this always uses a Z-Curve encoding (bit-wise interleaving) for
// linearization with 8-bits per dimension.
// This is sufficient for currently used voxel map dimensions and cell widths
// (typically we have 3m bounding box edges and 2cm cell widths, so 150 cells).
// If this changes, implement EncodeVoxel/DecodeVoxel functions for wider data
// types.

#ifndef COLLISION_CHECKING_VOXEL_CODE_H_
#define COLLISION_CHECKING_VOXEL_CODE_H_

#include <cstdint>
#include <limits>

#include "collision_checking/geometry.h"

namespace collision_checking {

// This struct collects definitions of data types used for voxel
// encoding/decoding.
struct VoxelCodeTraits {
 public:
  using IntCoordType = uint8_t;
  using CodeType = uint32_t;

  static constexpr CodeType kMinCode = 0x0;
  static constexpr CodeType kMaxCode = 0xFFFFFF;
  static constexpr IntCoordType kMinCoord = IntCoordType{0};
  static constexpr IntCoordType kMaxCoord =
      std::numeric_limits<IntCoordType>::max();
};

// Returns the voxel code for `coordinates` (hash value of the index).
inline VoxelCodeTraits::CodeType VoxelIndexToCode(
    const Vector3<VoxelCodeTraits::IntCoordType>& coords) {
  VoxelCodeTraits::CodeType result = 0;
  // Trivial but slow implementation. At least 3x speedup should be doable.
  // Hook in an existing morton code library, or implement multiplication
  // trick here if you care about the speed of this.
  for (int i = 0; i < CHAR_BIT * sizeof(VoxelCodeTraits::IntCoordType); ++i) {
    for (int j = 0; j < 3; ++j) {
      result |= static_cast<VoxelCodeTraits::CodeType>((coords[j] >> i) & 1)
                << (i * 3 + j);
    }
  }
  return result;
}

// Returns the coordinates (3d point) for `code` (hash value).
inline Vector3<VoxelCodeTraits::IntCoordType> CodeToVoxelIndex(
    const VoxelCodeTraits::CodeType code) {
  Vector3<VoxelCodeTraits::IntCoordType> coords{{0, 0, 0}};
  // Trivial but slow implementation. At least 3x speedup should be doable.
  // Hook in an existing morton code library, or implement multiplication
  // trick here if you care about the speed of this.
  for (int i = 0; i < CHAR_BIT * sizeof(VoxelCodeTraits::IntCoordType); ++i) {
    for (int j = 0; j < 3; ++j) {
      coords[j] |=
          static_cast<VoxelCodeTraits::IntCoordType>((code >> (i * 3 + j)) & 1)
          << i;
    }
  }
  return coords;
}

}  // namespace collision_checking

#endif  // COLLISION_CHECKING_VOXEL_CODE_H_
