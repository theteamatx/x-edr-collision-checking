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

#ifndef COLLISION_CHECKING_MODEL_OPTIONS_H_
#define COLLISION_CHECKING_MODEL_OPTIONS_H_

#include "collision_checking/inlining.h"

namespace collision_checking {

// Policies for disabling certain collision pairs.
enum class CollisionMaskingPolicy {
  // Invalid default value.
  kUnknown,
  // No automatic masking.
  kNothing,
  // Mask links that are connected according to AssemblyConnectivity.
  kConnectedLinks
};

// Collision Checking options for assemblies.
template <typename Scalar>
class ModelOptions {
 public:
  ModelOptions() = default;
  // Returns the axis aligned bounding box padding.
  CC_INLINE Scalar GetAABBPadding() const { return aabb_padding_; }
  // Sets the axis aligned bounding box padding.
  // Only distances between objects with overlapping bounding boxes are
  // computed. Set the padding to a value that is larger then the largest
  // positive distance that should be computed.
  // Smaller values for the padding generally enable faster collision
  // checking.
  ModelOptions& SetAABBPadding(Scalar padding) {
    aabb_padding_ = padding;
    return *this;
  }
  // Returns the assembly padding.
  CC_INLINE Scalar GetAssemblyPadding() const { return assembly_padding_; }
  // Sets the padding [m] to apply to all collision gemetries in the assembly.
  ModelOptions& SetAssemblyPadding(Scalar padding) {
    assembly_padding_ = padding;
    return *this;
  }
  // Returns policy for filtering collision pairs.
  CollisionMaskingPolicy GetMaskingPolicy() const { return masking_policy_; }
  // Sets the policy for filtering collision pairs.
  ModelOptions& SetMaskingPolicy(CollisionMaskingPolicy policy) {
    masking_policy_ = policy;
    return *this;
  }

 private:
  CollisionMaskingPolicy masking_policy_ = CollisionMaskingPolicy::kNothing;
  // Padding for axis aligned bounding boxes.
  Scalar aabb_padding_ = 0.1;
  // Padding for assembly geometries.
  Scalar assembly_padding_ = 0.0;
};

}  // namespace collision_checking
#endif  // COLLISION_CHECKING_MODEL_OPTIONS_H_
