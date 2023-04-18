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

#ifndef COLLISION_CHECKING_DISTANCE_BOX_BOX_H_
#define COLLISION_CHECKING_DISTANCE_BOX_BOX_H_

// Functions for computing the minimum squared distances between geometric
// primitives.

#include <limits>
#include <type_traits>

#include "collision_checking/debug_options.h"
#include "collision_checking/distance_point_box.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/inlining.h"
#include "eigenmath/line_search.h"

namespace collision_checking {
template <typename Scalar>
struct ObjectObjectResult {
  Scalar distance_squared;
  Vector3<Scalar> point_on_object_a;
  Vector3<Scalar> point_on_object_b;
};

// as the closest points on box_a and box_b.
template <typename Scalar,
          unsigned kDebugOptions = DebugOptions::kDebugOptionsNone>
CC_INLINE ObjectObjectResult<Scalar> DistanceSquared(const Box<Scalar>& box_a,
                                                     const Box<Scalar>& box_b);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
namespace internal {
// Minimum distance squared and closest points for two box faces.
template <typename Scalar>
CC_INLINE ObjectObjectResult<Scalar> BoxFaceBoxFaceDistanceSquared(
    const Box<Scalar>& box_a, const Box<Scalar>& box_b, int face_a,
    Scalar offset_a, int face_b, Scalar offset_b) {
  // TODO: Implement analytical solution.
  using Vector3 = Vector3<Scalar>;
  using Matrix3 = Matrix3<Scalar>;

  // std::tie cannot be used in CUDA device code, because it isn't marked with
  // __device__ and isn't constexpr.
  // This struct can be used instead.
  struct CudaSafeTie {
    CC_INLINE CudaSafeTie(Scalar& f, Scalar& s) : first(f), second(s) {}
    CC_INLINE CudaSafeTie& operator=(const std::pair<Scalar, Scalar>& rhs) {
      first = rhs.first;
      second = rhs.second;
      return *this;
    }
    Scalar& first;
    Scalar& second;
  };

  Scalar min_distance_squared = std::numeric_limits<Scalar>::max();
  Vector3 min_point_a = Vector3::Zero(), min_point_b = Vector3::Zero();
  // u,v are axes in the face plane, w is the face normal.
  const int face_a_u = BoxFace<Scalar>::kUIndices[face_a];
  const int face_a_v = BoxFace<Scalar>::kVIndices[face_a];
  const int face_a_w = BoxFace<Scalar>::kWIndices[face_a];
  const int face_b_u = BoxFace<Scalar>::kUIndices[face_b];
  const int face_b_v = BoxFace<Scalar>::kVIndices[face_b];
  const int face_b_w = BoxFace<Scalar>::kWIndices[face_b];

  // Represent a vector given in box b frame in box a frame.
  const Matrix3 a_rotation_b =
      box_a.box_rotation_world * box_b.box_rotation_world.transpose();
  const Vector3 a_translation_b =
      box_a.box_rotation_world * (box_b.center - box_a.center);
  const Vector3 b_translation_a =
      box_b.box_rotation_world * (box_a.center - box_b.center);

  constexpr Scalar kTolerance = std::numeric_limits<Scalar>::epsilon() * 100;

  //-------------------------------
  // Edge face B in Box A frame.
  Scalar segment_parameter, distance_squared;
  CudaSafeTie tied_results(segment_parameter, distance_squared);
  Vector3 point_box;
  // u+ offset, v-direction
  point_box[face_b_u] = box_b.half_lengths[face_b_u];
  point_box[face_b_v] = 0;
  point_box[face_b_w] = offset_b;
  Vector3 seg_center = a_translation_b + a_rotation_b * point_box;
  Vector3 seg_dir = box_a.box_rotation_world *
                    box_b.box_rotation_world.row(face_b_v).transpose();
  Scalar seg_half_length = box_b.half_lengths[face_b_v];
  Vector3 box_half_lengths = box_a.half_lengths;

  auto distance_squared_func = [&](const Scalar segment_parameter) -> Scalar {
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    return internal::PointBoxDistanceSquared(segment_point, box_half_lengths)
        .distance_squared;
  };

  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_a.half_lengths)
            .point_on_box;
    min_point_a =
        box_a.center + box_a.box_rotation_world.transpose() * point_on_box;
    min_point_b =
        box_a.center + box_a.box_rotation_world.transpose() * segment_point;
  }

  // u- offset, v-direction
  point_box[face_b_u] = -box_b.half_lengths[face_b_u];
  point_box[face_b_v] = 0;
  point_box[face_b_w] = offset_b;
  seg_center = a_translation_b + a_rotation_b * point_box;
  seg_dir = a_rotation_b.col(face_b_v);
  seg_half_length = box_b.half_lengths[face_b_v];

  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_a.half_lengths)
            .point_on_box;
    min_point_a =
        box_a.center + box_a.box_rotation_world.transpose() * point_on_box;
    min_point_b =
        box_a.center + box_a.box_rotation_world.transpose() * segment_point;
  }

  // v+ offset, u-direction
  point_box[face_b_u] = 0;
  point_box[face_b_v] = box_b.half_lengths[face_b_v];
  point_box[face_b_w] = offset_b;
  seg_center = a_translation_b + a_rotation_b * point_box;
  seg_dir = a_rotation_b.col(face_b_u);
  seg_half_length = box_b.half_lengths[face_b_u];
  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_a.half_lengths)
            .point_on_box;
    min_point_a =
        box_a.center + box_a.box_rotation_world.transpose() * point_on_box;
    min_point_b =
        box_a.center + box_a.box_rotation_world.transpose() * segment_point;
  }

  // v- offset, u-direction
  point_box[face_b_u] = 0;
  point_box[face_b_v] = -box_b.half_lengths[face_b_v];
  point_box[face_b_w] = offset_b;
  seg_center = a_translation_b + a_rotation_b * point_box;
  seg_dir = a_rotation_b.col(face_b_u);
  seg_half_length = box_b.half_lengths[face_b_u];
  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_a.half_lengths)
            .point_on_box;
    min_point_a =
        box_a.center + box_a.box_rotation_world.transpose() * point_on_box;
    min_point_b =
        box_a.center + box_a.box_rotation_world.transpose() * segment_point;
  }

  //-------------------------------
  // Edge face A in Box B frame.
  box_half_lengths = box_b.half_lengths;
  // u+ offset, v-direction
  point_box[face_a_u] = box_a.half_lengths[face_a_u];
  point_box[face_a_v] = 0;
  point_box[face_a_w] = offset_a;
  seg_center = b_translation_a + a_rotation_b.transpose() * point_box;
  seg_dir = a_rotation_b.row(face_a_v).transpose();
  seg_half_length = box_a.half_lengths[face_a_v];
  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_b.half_lengths)
            .point_on_box;
    min_point_a =
        box_b.center + box_b.box_rotation_world.transpose() * segment_point;
    min_point_b =
        box_b.center + box_b.box_rotation_world.transpose() * point_on_box;
  }

  // u- offset, v-direction
  point_box[face_a_u] = -box_a.half_lengths[face_a_u];
  point_box[face_a_v] = 0;
  point_box[face_a_w] = offset_a;
  seg_center = b_translation_a + a_rotation_b.transpose() * point_box;
  seg_dir = a_rotation_b.row(face_a_v).transpose();
  seg_half_length = box_a.half_lengths[face_a_v];
  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_b.half_lengths)
            .point_on_box;
    min_point_a =
        box_b.center + box_b.box_rotation_world.transpose() * segment_point;
    min_point_b =
        box_b.center + box_b.box_rotation_world.transpose() * point_on_box;
  }

  // v+ offset, u-direction
  point_box[face_a_u] = 0;
  point_box[face_a_v] = box_a.half_lengths[face_a_v];
  point_box[face_a_w] = offset_a;
  seg_center = b_translation_a + a_rotation_b.transpose() * point_box;
  seg_dir = a_rotation_b.row(face_a_u).transpose();
  seg_half_length = box_a.half_lengths[face_a_u];
  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_b.half_lengths)
            .point_on_box;
    min_point_a =
        box_b.center + box_b.box_rotation_world.transpose() * segment_point;
    min_point_b =
        box_b.center + box_b.box_rotation_world.transpose() * point_on_box;
  }

  // v- offset, u-direction
  point_box[face_a_u] = 0;
  point_box[face_a_v] = -box_a.half_lengths[face_a_v];
  point_box[face_a_w] = offset_a;
  seg_center = b_translation_a + a_rotation_b.transpose() * point_box;
  seg_dir = a_rotation_b.row(face_a_u).transpose();
  seg_half_length = box_a.half_lengths[face_a_u];
  // TODO: Use successive quadratic interpolation here, as the
  // function is piece-wise quadratic.
  tied_results = GoldenSectionSearchMinimize(-seg_half_length, seg_half_length,
                                             distance_squared_func, kTolerance);

  if (distance_squared < min_distance_squared) {
    min_distance_squared = distance_squared;
    // Call point/box one more time to recover point on box.
    const Vector3 segment_point = seg_center + seg_dir * segment_parameter;
    const Vector3 point_on_box =
        internal::PointBoxDistanceSquared(segment_point, box_b.half_lengths)
            .point_on_box;
    min_point_a =
        box_b.center + box_b.box_rotation_world.transpose() * segment_point;
    min_point_b =
        box_b.center + box_b.box_rotation_world.transpose() * point_on_box;
  }
  return {.distance_squared = min_distance_squared,
          .point_on_object_a = min_point_a,
          .point_on_object_b = min_point_b};
}

}  // namespace internal

template <typename Scalar, unsigned kDebugOptions>
CC_INLINE ObjectObjectResult<Scalar> DistanceSquared(const Box<Scalar>& box_a,
                                                     const Box<Scalar>& box_b) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating point type.");
  if constexpr (kDebugOptions &
                DebugOptions::kDebugOptionsPerformExpensiveInputChecks) {
    BLUE_CHECK((box_a.half_lengths.array() >= Scalar{0}).all(),
               "box_a.half_lengths must be positive.");
    BLUE_CHECK(
        (box_a.box_rotation_world * box_a.box_rotation_world.transpose() -
         Matrix3<Scalar>::Identity())
                .norm() < std::numeric_limits<Scalar>::epsilon() * 10,
        "box_a.box_rotation_world must be a valid rotation matrix.");
    BLUE_CHECK((box_b.half_lengths.array() >= Scalar{0}).all(),
               "box_b.half_lengths must be positive.");
    BLUE_CHECK(
        (box_b.box_rotation_world * box_b.box_rotation_world.transpose() -
         Matrix3<Scalar>::Identity())
                .norm() < std::numeric_limits<Scalar>::epsilon() * 10,
        "box_b.box_rotation_world must be a valid rotation matrix.");
  }
  // This solves the minimum distance problem:
  // min
  // (box_a.center+box_a.box_rotation_world^T*u-box_b.center-box_b.box_rotation_world'*v)^2
  // -> min,
  //  s.t. |u_i| <= box_a.half_lengths[i],
  //       |v_i| <= box_b.half_lengths[i],
  // This is an expensive semi-numerical implementation based on the point-box
  // distance.
  // TODO: Implement analytical solution.
  using Vector3 = Vector3<Scalar>;

  // Check for overlap.
  Scalar min_distance_squared = std::numeric_limits<Scalar>::max();
  Vector3 min_point_a = Vector3::Zero(), min_point_b = Vector3::Zero();

  // These are all box faces:
  // const Face box_a_faces[] = {
  //     {0, box_a.half_lengths[0]},  {1, box_a.half_lengths[1]},
  //     {2, box_a.half_lengths[2]},  {0, -box_a.half_lengths[0]},
  //     {1, -box_a.half_lengths[1]}, {2, -box_a.half_lengths[2]},
  // };
  // const Face box_b_faces[] = {
  //     {0, box_b.half_lengths[0]},  {1, box_b.half_lengths[1]},
  //     {2, box_b.half_lengths[2]},  {0, -box_b.half_lengths[0]},
  //     {1, -box_b.half_lengths[1]}, {2, -box_b.half_lengths[2]},
  // };
  // It isn't necessary to check all, instead only check those with the
  // right sign.
  const Vector3 a_translation_b =
      box_a.box_rotation_world * (box_b.center - box_a.center);
  const Vector3 b_translation_a =
      box_b.box_rotation_world * (box_a.center - box_b.center);

  const BoxFace<Scalar> box_a_faces[] = {
      {0, std::copysign(box_a.half_lengths[0], a_translation_b[0])},
      {1, std::copysign(box_a.half_lengths[1], a_translation_b[1])},
      {2, std::copysign(box_a.half_lengths[2], a_translation_b[2])}};
  const BoxFace<Scalar> box_b_faces[] = {
      {0, std::copysign(box_b.half_lengths[0], b_translation_a[0])},
      {1, std::copysign(box_b.half_lengths[1], b_translation_a[1])},
      {2, std::copysign(box_b.half_lengths[2], b_translation_a[2])},
  };

  for (const auto& face_a : box_a_faces) {
    for (const auto& face_b : box_b_faces) {
      auto [distance_squared, point_a, point_b] =
          internal::BoxFaceBoxFaceDistanceSquared(box_a, box_b, face_a.index,
                                                  face_a.offset, face_b.index,
                                                  face_b.offset);
      // Set distance == 0 if it is very close to ensure tests for collision
      // using distance <= 0 are robust to floating point error.
      if (distance_squared < Eigen::NumTraits<Scalar>::dummy_precision()) {
        distance_squared = Scalar{0};
      }
      if (distance_squared < min_distance_squared) {
        min_distance_squared = distance_squared;
        min_point_a = point_a;
        min_point_b = point_b;
      }
    }
  }

  return {.distance_squared = min_distance_squared,
          .point_on_object_a = min_point_a,
          .point_on_object_b = min_point_b};
}

}  // namespace collision_checking
#endif  // COLLISION_CHECKING_DISTANCE_BOX_BOX_H_
