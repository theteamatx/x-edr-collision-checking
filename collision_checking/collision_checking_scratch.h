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

#ifndef COLLISION_CHECKING_COLLISION_CHECKING_SCRATCH_H_
#define COLLISION_CHECKING_COLLISION_CHECKING_SCRATCH_H_

#include "collision_checking/assembly_kinematics.h"
#include "collision_checking/composite_object.h"
#include "collision_checking/inlining.h"
#include "collision_checking/vector.h"

namespace collision_checking {

// A scratch object for mutable data used during collision queries.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
struct Scratch : public AssemblyPoses<Scalar, AllocatorTraits> {
  Scratch() = default;
  // Constructs a Scratch object with `object_count` poses and
  // objects.
  explicit Scratch(int object_count) { Resize(object_count); }
  void Resize(int object_count) {
    AssemblyPoses<Scalar, AllocatorTraits>::Resize(object_count);
    transformed_objects.objects.resize(object_count);
  }

  // Returns the number of objects.
  CC_INLINE int GetObjectCount() const {
    return transformed_objects.objects.size();
  }

  // Collision objects in a transformed configuration.
  CollisionObjects<Scalar, AllocatorTraits> transformed_objects;
};
}  // namespace collision_checking
#endif  // COLLISION_CHECKING_COLLISION_CHECKING_SCRATCH_H_
