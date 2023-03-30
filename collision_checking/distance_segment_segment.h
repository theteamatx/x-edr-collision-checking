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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_SEGMENT_SEGMENT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_SEGMENT_SEGMENT_H_

// Functions for computing the minimum squared distances between geometric
// primitives.

#include <limits>
#include <string>
#include <type_traits>

#include "absl/strings/str_format.h"
#include "collision_checking/debug_options.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/inlining.h"
#include "collision_checking/logging.h"

namespace collision_checking {

template <typename Scalar>
struct SegmentSegmentResult {
  Scalar distance_squared;
  Scalar segment_a_parameter;
  Scalar segment_b_parameter;
};

// Returns the squared distance between two line segments, as well
// as the parameters specifying the location of the closest points on the
// segments.
// If kDebugOptions & DebugOptions::kPerformExpensiveInputChecks != 0, input
// parameter preconditions are asserted, otherwise they are ignored.
// Scalar must be a floating point type.
template <typename Scalar,
          unsigned kDebugOptions = DebugOptions::kDebugOptionsNone>
CC_INLINE SegmentSegmentResult<Scalar> SegmentSegmentDistanceSquared(
    const Segment<Scalar>& segment_a, const Segment<Scalar>& segment_b);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

template <typename Scalar, unsigned kDebugOptions>
CC_INLINE SegmentSegmentResult<Scalar> SegmentSegmentDistanceSquared(
    const Segment<Scalar>& segment_a, const Segment<Scalar>& segment_b) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating point type.");

  if constexpr (kDebugOptions &
                DebugOptions::kDebugOptionsPerformExpensiveInputChecks) {
    CC_CHECK_LT(std::abs(segment_a.direction.squaredNorm() - Scalar{1}),
                std::numeric_limits<Scalar>::epsilon() * 10);
    CC_CHECK_GE(segment_a.half_length, Scalar{0});
    CC_CHECK_LT(std::abs(segment_b.direction.squaredNorm() - Scalar{1}),
                std::numeric_limits<Scalar>::epsilon() * 10);
    CC_CHECK_GE(segment_b.half_length, Scalar{0});
  }

  // This solves:
  // min ((segment_a.center+segment_a.direction*u -
  //      (segment_b.center+segment_b.direction*v))^2)
  // s.t., |u| <= segment_a.half_length,
  //       |v| <= segment_b.half_length.
  // The function and the constraint region are convex, so the minimum is the
  // unconstrained minimum projected onto the valid region.
  // Regions for the projection are enumerated below, where u is the horizontal
  // axis and v the vertical axis.
  // If the unconstrained minimum is in a corner (8, 5, 9, 6), the projection
  // might be an any of the closest edges. The code below simply tries one,
  // then the other if that choice is invalid.
  // (This logic could be avoided by projecting in the direction computed from
  //  the gradient, but likely at higher computational cost).
  // 8|2|5
  // -----
  // 7|1|4
  // -----
  // 9|3|6
  const Vector3<Scalar> diff = segment_a.center - segment_b.center;
  const Scalar dira_dot_diff = segment_a.direction.dot(diff);
  const Scalar dirb_dot_diff = segment_b.direction.dot(diff);
  const Scalar dira_dot_dirb = segment_a.direction.dot(segment_b.direction);
  const Scalar det = Scalar(1.0) - dira_dot_dirb * dira_dot_dirb;
  const Scalar half_length_det1 = segment_a.half_length * det;
  const Scalar half_length_det2 = segment_b.half_length * det;

  // Check for special case of parallel lines.
  // This is chosen to pass all tests at float and double precision, but could
  // use some more rigorous analysis to verify / improve the choice.
  // The critical case is loss of precision in computing det at single
  // precision.
  constexpr Scalar kParallelLinesEpsilon =
      std::numeric_limits<Scalar>::epsilon() * Scalar(1e1);
  if (det <= kParallelLinesEpsilon) {
    // det ~ 0 means parallel lines.
    // Solve
    // min ((diff+w*dir)^2,
    // s.t. |w| <= wmax.
    // Here w= (u-v) and dir is an averaged direction.
    // Notes:
    // - The parameters u, v might not be uniquely defined.
    // - Just choosing one direction instead of averaging yields only slightly
    //   larger errors (depending on the choice of kParallelLinesEpsilon).
    //   So omitting averaging might be acceptable as a microoptimization.
    // NOLINTNEXTLINE(google3-custom-no-eigen-default-ctor): Initialized below.
    Vector3<Scalar> dir;
    if (dira_dot_dirb > Scalar{0.0}) {
      // parallel case.
      dir = Scalar{0.5} * (segment_a.direction + segment_b.direction);
    } else {
      // anti-parallel case.
      dir = Scalar{0.5} * (segment_a.direction - segment_b.direction);
    }
    const Scalar w_max = segment_a.half_length + segment_b.half_length;
    const Scalar w = Saturate(-dir.dot(diff), w_max);
    const Scalar distance_squared = (diff + w * dir).squaredNorm();
    // Minimum norm solution, projected onto valid v-set.
    const Scalar v_minnorm = Saturate(Scalar{0.5} * w, segment_b.half_length);
    // U might be violated for this solution, so project onto valid set and
    // recompute v, which should yield a valid solution.
    const Scalar u = Saturate(w + v_minnorm, segment_a.half_length);
    const Scalar v = u - w;
    return {.distance_squared = distance_squared,
            .segment_a_parameter = u,
            .segment_b_parameter = v};
  }
  // det > 0 means lines are not parallel.
  const Scalar u_det = -dira_dot_diff + dira_dot_dirb * dirb_dot_diff;
  const Scalar v_det = -dira_dot_dirb * dira_dot_diff + dirb_dot_diff;
  if (u_det >= -half_length_det1) {
    if (u_det <= half_length_det1) {
      if (v_det >= -half_length_det2) {
        if (v_det <= half_length_det2) {
          // Region 1: u/v are within limits: use line/line minimum.
          const Scalar u = u_det / det;
          const Scalar v = v_det / det;
          const Scalar distance_squared =
              (diff + segment_a.direction * u - segment_b.direction * v)
                  .squaredNorm();
          return {.distance_squared = distance_squared,
                  .segment_a_parameter = u,
                  .segment_b_parameter = v};
        }
        // u_det >= -half_length_det1
        // AND u_def <= half_length_det1
        // AND v_det > half_length_det2
        // Region 2: v is at the upper limit, u within limits.
        // Compute distance with v set to upper limit.
        const Scalar v = segment_b.half_length;
        const Scalar u =
            Saturate(-dira_dot_diff + dira_dot_dirb * v, segment_a.half_length);
        const Scalar distance_squared =
            (diff + segment_a.direction * u - segment_b.direction * v)
                .squaredNorm();
        return {.distance_squared = distance_squared,
                .segment_a_parameter = u,
                .segment_b_parameter = v};
      }
      // Region 3: v is below lower limit, u is within limits.
      // v is at the lower limit.
      // Compute distance with v set to lower limit.
      const Scalar v = -segment_b.half_length;
      const Scalar u =
          Saturate(-dira_dot_diff + dira_dot_dirb * v, segment_a.half_length);
      const Scalar distance_squared =
          (diff + segment_a.direction * u - segment_b.direction * v)
              .squaredNorm();
      return {.distance_squared = distance_squared,
              .segment_a_parameter = u,
              .segment_b_parameter = v};
    }
    // u_det > half_length_det1
    if (v_det >= -half_length_det2) {
      if (v_det <= half_length_det2) {
        // Region 4: u above limit, v within limits.
        // Compute distance with u set to upper limit.
        const Scalar u = segment_a.half_length;
        const Scalar v =
            Saturate(dirb_dot_diff + dira_dot_dirb * u, segment_b.half_length);
        const Scalar distance_squared =
            (diff + segment_a.direction * u - segment_b.direction * v)
                .squaredNorm();
        return {.distance_squared = distance_squared,
                .segment_a_parameter = u,
                .segment_b_parameter = v};
      }
      // Region 5: u above limit, v above limit.
      // Try solution at umax.
      Scalar u = segment_a.half_length;
      Scalar v =
          std::max(dirb_dot_diff + dira_dot_dirb * u, -segment_b.half_length);
      // Use solution at vmax instead.
      if (v > segment_b.half_length) {
        v = segment_b.half_length;
        u = Saturate(-dira_dot_diff + dira_dot_dirb * v, segment_a.half_length);
      }
      const Scalar distance_squared =
          (diff + segment_a.direction * u - segment_b.direction * v)
              .squaredNorm();
      return {.distance_squared = distance_squared,
              .segment_a_parameter = u,
              .segment_b_parameter = v};
    }
    // Region 6: u above limit, v below limit.
    // Try solution at umax.
    Scalar u = segment_a.half_length;
    Scalar v =
        std::min(dirb_dot_diff + dira_dot_dirb * u, segment_b.half_length);
    // Use solution at vmin instead.
    if (v < -segment_b.half_length) {
      v = -segment_b.half_length;
      u = Saturate(-dira_dot_diff + dira_dot_dirb * v, segment_a.half_length);
    }
    const Scalar distance_squared =
        (diff + segment_a.direction * u - segment_b.direction * v)
            .squaredNorm();
    return {.distance_squared = distance_squared,
            .segment_a_parameter = u,
            .segment_b_parameter = v};
  }
  // u_det < -half_length_det1
  if (v_det >= -half_length_det2) {
    if (v_det <= half_length_det2) {
      // Region 7: u below limit, v within limits.
      // u is at the lower limit.
      // Compute distance with u set to lower limit.
      const Scalar u = -segment_a.half_length;
      const Scalar v =
          Saturate(dirb_dot_diff + dira_dot_dirb * u, segment_b.half_length);
      const Scalar distance_squared =
          (diff + segment_a.direction * u - segment_b.direction * v)
              .squaredNorm();
      return {.distance_squared = distance_squared,
              .segment_a_parameter = u,
              .segment_b_parameter = v};
    }
    // Region 9: u below limit, v above limit.
    // Try solution at umin.
    Scalar u = -segment_a.half_length;
    Scalar v =
        std::max(dirb_dot_diff + dira_dot_dirb * u, -segment_b.half_length);
    // Use solution at vmax instead.
    if (v > segment_b.half_length) {
      v = segment_b.half_length;
      u = Saturate(-dira_dot_diff + dira_dot_dirb * v, segment_a.half_length);
    }
    const Scalar distance_squared =
        (diff + segment_a.direction * u - segment_b.direction * v)
            .squaredNorm();
    return {.distance_squared = distance_squared,
            .segment_a_parameter = u,
            .segment_b_parameter = v};
  }
  // Region 9: u below limit, v below limit.
  // Try solution at umin.
  Scalar u = -segment_a.half_length;
  Scalar v = std::min(dirb_dot_diff + dira_dot_dirb * u, segment_b.half_length);
  // Use solution at vmin instead.
  if (v < -segment_b.half_length) {
    v = -segment_b.half_length;
    u = Saturate(-dira_dot_diff + dira_dot_dirb * v, segment_a.half_length);
  }
  const Scalar distance_squared =
      (diff + segment_a.direction * u - segment_b.direction * v).squaredNorm();
  return {.distance_squared = distance_squared,
          .segment_a_parameter = u,
          .segment_b_parameter = v};
}

// Functions in this namespace are intended for internal or debugging use only.
namespace internal {
// Returns a string representation of Mathematica code to solve the
// segment/segment distance problem.
template <typename Scalar>
std::string SegmentSegmentDistanceSquaredMathematicaCode(
    const Segment<Scalar>& segment_a, const Segment<Scalar>& segment_b) {
  const Vector3<Scalar> diff = segment_a.center - segment_b.center;
  return absl::StrFormat(
      "Minimize[{(%.10f+(%.10f)*u-(%.10f)*v)^2+"
      "(%.10f+(%.10f)*u-(%.10f)*v)^2+(%.10f+(%.10f)*u-(%.10f)*v)^2,"
      "u >= %.10f, u <= %.10f, v >= %.10f, "
      "v<=%.10f},{u,v}]// FullForm",
      diff[0], segment_a.direction[0], segment_b.direction[0], diff[1],
      segment_a.direction[1], segment_b.direction[1], diff[2],
      segment_a.direction[2], segment_b.direction[2], -segment_a.half_length,
      segment_a.half_length, -segment_b.half_length, segment_b.half_length);
}

}  // namespace internal
}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_SEGMENT_SEGMENT_H_
