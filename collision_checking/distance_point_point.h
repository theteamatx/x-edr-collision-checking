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

#ifndef COLLISION_CHECKING_DISTANCE_POINT_POINT_H_
#define COLLISION_CHECKING_DISTANCE_POINT_POINT_H_

// Functions for computing the minimum squared distances between geometric
// primitives.

#include <type_traits>

#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/inlining.h"

namespace collision_checking {

// Result struct for point/point distance squared.
// This is used to enable templatized code for different *DistanceSquared
// functions.
template <typename Scalar>
struct PointPointResult {
  Scalar distance_squared;
};

// Returns the squared distance between two points.
// S must be a floating point type.
template <typename Scalar>
CC_INLINE PointPointResult<Scalar> DistanceSquared(
    const Point<Scalar>& point_a, const Point<Scalar>& point_b);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
template <typename Scalar>
CC_INLINE PointPointResult<Scalar> DistanceSquared(
    const Point<Scalar>& point_a, const Point<Scalar>& point_b) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating-point type.");

  return {.distance_squared = (point_a.center - point_b.center).squaredNorm()};
}
}  // namespace collision_checking

#endif  // COLLISION_CHECKING_DISTANCE_POINT_POINT_H_
