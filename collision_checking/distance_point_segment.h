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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_SEGMENT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_SEGMENT_H_

// Functions for computing the minimum squared distances between geometric
// primitives.

#include <limits>
#include <type_traits>

#include "collision_checking/debug_options.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/inlining.h"
#include "collision_checking/logging.h"
#include "eigenmath/scalar_utils.h"

namespace collision_checking {

template <typename Scalar>
struct PointSegmentResult {
  Scalar distance_squared;
  Scalar segment_parameter;
};

// Returns the squared distance between a point and a line segment
// and the parameter specifying the location of the closest point on the
// segment.
// If kDebugOptions & DebugOptions::kPerformExpensiveInputChecks != 0, input
// parameter preconditions are asserted, otherwise they are ignored. Scalar must
// be a floating point type.
template <typename Scalar, unsigned kDebugOptions = kDebugOptionsNone>
CC_INLINE PointSegmentResult<Scalar> DistanceSquared(
    const Point<Scalar>& point, const Segment<Scalar>& segment);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

template <typename Scalar, unsigned kDebugOptions>
CC_INLINE PointSegmentResult<Scalar> DistanceSquared(
    const Point<Scalar>& point, const Segment<Scalar>& segment) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating-point type.");
  if constexpr (kDebugOptions &
                DebugOptions::kDebugOptionsPerformExpensiveInputChecks) {
    CC_CHECK_LT(std::abs(segment.direction.squaredNorm() - Scalar{1}),
                std::numeric_limits<Scalar>::epsilon() * 10);
    CC_CHECK_GE(segment.half_length, Scalar{0});
  }

  // This solves:
  // min (segment.center+segment.direction*u - point)^2 -> min,
  // s.t. |u| <= segment.half_length.
  // segment: s.p0+s.dir*u, u in [-half_length,half_length].
  const Vector3<Scalar> mid_distance = point.center - segment.center;
  const Scalar u = std::clamp(mid_distance.dot(segment.direction),
                              -segment.half_length, segment.half_length);
  return {
      .distance_squared = (mid_distance - segment.direction * u).squaredNorm(),
      .segment_parameter = u};
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_SEGMENT_H_
