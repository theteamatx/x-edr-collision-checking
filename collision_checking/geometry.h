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

#ifndef COLLISION_CHECKING_GEOMETRY_H_
#define COLLISION_CHECKING_GEOMETRY_H_

// Structs and simple utility functions for geometric objects.
// All geometry must be given in a common reference frame, referred to as
// "world" in the code. The choice is largely arbitrary. All vectors are assumed
// to be given in this frame if not explicitly mentioned otherwise.

#include <limits>
#include <string>

#include "absl/strings/str_format.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/inlining.h"
#include "collision_checking/logging.h"

namespace collision_checking {
// A point struct that allows distance computation functions for different
// geometric objects to have the same signature.
template <typename Scalar>
struct Point {
  Vector3<Scalar> center = Vector3<Scalar>::Zero();
};

// A sphere, a.k.a. sphere swept point.
template <typename Scalar>
struct Sphere : public Point<Scalar> {
  Scalar radius = Scalar(0);
};

// A segment defined by its center, a direction and half length.
// Points on the segment are given by center+direction*u, with |u| <=
// half_length.
template <typename Scalar>
struct Segment {
  Vector3<Scalar> center = Vector3<Scalar>::Zero();
  // Direction of the segment. Must be normalized.
  Vector3<Scalar> direction = Vector3<Scalar>::Zero();
  // Half-length of the segment.
  Scalar half_length = Scalar(0);
};

// A capsule, a.k.a.  sphere swept segment.
template <typename Scalar>
struct Capsule : public Segment<Scalar> {
  Scalar radius = Scalar(0);
};

// An axis aligned bounding box.
template <typename Scalar>
struct AlignedBox {
  // Corner with minimum coordinate values.
  Vector3<Scalar> low =
      Vector3<Scalar>::Constant(std::numeric_limits<Scalar>::max());
  // Corner with maximum coordinate values.
  Vector3<Scalar> high =
      Vector3<Scalar>::Constant(std::numeric_limits<Scalar>::lowest());
};

// An oriented box.
template <typename Scalar>
struct Box {
  // Center of the box.
  Vector3<Scalar> center = Vector3<Scalar>::Zero();
  // Orientation of the box. Column vectors are the unit vectors of "world"
  // frame given in the "box" frame.
  Matrix3<Scalar> box_rotation_world = Matrix3<Scalar>::Identity();
  // Half-lengths of the box, in the frame defined by orientation.
  Vector3<Scalar> half_lengths = Vector3<Scalar>::Zero();
};

// The face of a Box.
template <typename Scalar>
struct BoxFace {
  // Lookup tables for axis indices based on face index.
  // Helpful for treating all faces uniformly as if they were parallel to the
  // (x, y) plane.
  inline static constexpr int kUIndices[3] = {1, 2, 0};
  inline static constexpr int kVIndices[3] = {2, 0, 1};
  inline static constexpr int kWIndices[3] = {0, 1, 2};

  int index;
  Scalar offset;
};

// Returns true if the bounding boxes a and b have overlap, false otherwise.
template <typename Scalar>
CC_INLINE bool DoOverlap(const AlignedBox<Scalar>& a,
                         const AlignedBox<Scalar>& b) {
  if (a.high[0] < b.low[0] || a.low[0] > b.high[0] || a.high[1] < b.low[1] ||
      a.low[1] > b.high[1] || a.high[2] < b.low[2] || a.low[2] > b.high[2]) {
    return false;
  }
  return true;
}

// Returns true if aligned_box and sphere overlap, false otherwise.
template <typename Scalar>
CC_INLINE bool DoOverlap(const AlignedBox<Scalar>& aligned_box,
                         const Sphere<Scalar>& sphere) {
  if (aligned_box.high[0] < sphere.center[0] - sphere.radius ||
      aligned_box.low[0] > sphere.center[0] + sphere.radius ||
      aligned_box.high[1] < sphere.center[1] - sphere.radius ||
      aligned_box.low[1] > sphere.center[1] + sphere.radius ||
      aligned_box.high[2] < sphere.center[2] - sphere.radius ||
      aligned_box.low[2] > sphere.center[2] + sphere.radius) {
    return false;
  }
  return true;
}

// Returns true if point is inside (or on the boundary of) aligned_box.
// Vector3 is a type with operator[] that returns a Scalar.
template <typename Scalar, typename Vector3>
CC_INLINE bool IsInside(const AlignedBox<Scalar>& aligned_box,
                        const Vector3& point) {
  if (aligned_box.high[0] < point[0] || aligned_box.low[0] > point[0] ||
      aligned_box.high[1] < point[1] || aligned_box.low[1] > point[1] ||
      aligned_box.high[2] < point[2] || aligned_box.low[2] > point[2]) {
    return false;
  }
  return true;
}

// Sets the bounding box parameters such that IsInside(aligned_box,point) and
// DoOverlap(aligned_box, geometry) return false for any input.
template <typename Scalar>
CC_INLINE void SetEmptySize(AlignedBox<Scalar>* aligned_box) {
  CC_CHECK_NE(aligned_box, nullptr);
  aligned_box->low.setConstant(std::numeric_limits<Scalar>::max());
  aligned_box->high.setConstant(std::numeric_limits<Scalar>::lowest());
}

// Updates the bounding box to incorporate another bounding box.
template <typename Scalar>
CC_INLINE void GrowAlignedBoxAround(const AlignedBox<Scalar> other,
                                    AlignedBox<Scalar>* aligned_box) {
  aligned_box->low = aligned_box->low.cwiseMin(other.low);
  aligned_box->high = aligned_box->high.cwiseMax(other.high);
}

// Updates the bounding aligned_box to incorporate sphere.
template <typename Scalar>
CC_INLINE void GrowAlignedBoxAround(const Sphere<Scalar>& sphere,
                                    AlignedBox<Scalar>* aligned_box) {
  const Vector3<Scalar> offset(Vector3<Scalar>::Constant(sphere.radius));
  aligned_box->low = aligned_box->low.cwiseMin(sphere.center - offset);
  aligned_box->high = aligned_box->high.cwiseMax(sphere.center + offset);
}

// Updates the bounding aligned_box to incorporate capsule.
template <typename Scalar>
CC_INLINE void GrowAlignedBoxAround(const Capsule<Scalar>& capsule,
                                    AlignedBox<Scalar>* aligned_box) {
  const Vector3<Scalar> extent =
      (capsule.direction.cwiseAbs() * capsule.half_length).array() +
      capsule.radius;
  aligned_box->low = aligned_box->low.cwiseMin(capsule.center - extent);
  aligned_box->high = aligned_box->high.cwiseMax(capsule.center + extent);
}

// Updates the bounding aligned_box to incorporate box.
template <typename Scalar>
CC_INLINE void GrowAlignedBoxAround(const Box<Scalar>& box,
                                    AlignedBox<Scalar>* aligned_box) {
  const Vector3<Scalar> extent =
      box.box_rotation_world.transpose().cwiseAbs() * box.half_lengths;
  aligned_box->low = aligned_box->low.cwiseMin(box.center - extent);
  aligned_box->high = aligned_box->high.cwiseMax(box.center + extent);
}

// Returns a string representation of `sphere`.
template <typename Scalar>
std::string ToString(const Sphere<Scalar>& sphere) {
  return absl::StrFormat("{\n center= [%f %f %f]\n radius= %f\n}\n",
                         sphere.center[0], sphere.center[1], sphere.center[2],
                         sphere.radius);
}
template <typename Scalar>
std::string ToString(const Capsule<Scalar>& capsule) {
  return absl::StrFormat(
      "{\n center= [%f %f %f]\n"
      " direction= [%f %f %f]\n"
      " half_length= %f\n"
      " radius= %f\n}\n",
      capsule.center[0], capsule.center[1], capsule.center[2],
      capsule.direction[0], capsule.direction[1], capsule.direction[2],
      capsule.half_length, capsule.radius);
}
template <typename Scalar>
std::string ToString(const Box<Scalar>& box) {
  return absl::StrFormat(
      "{\n center= [%f %f %f]\n"
      " box_rotation_world= [%f %f %f; %f %f %f; %f %f %f]\n"
      " half_lengths= [%f %f %f]\n",
      box.center[0], box.center[1], box.center[2], box.box_rotation_world(0, 0),
      box.box_rotation_world(0, 1), box.box_rotation_world(0, 2),
      box.box_rotation_world(1, 0), box.box_rotation_world(1, 1),
      box.box_rotation_world(1, 2), box.box_rotation_world(2, 0),
      box.box_rotation_world(2, 1), box.box_rotation_world(2, 2),
      box.half_lengths[0], box.half_lengths[1], box.half_lengths[2]);
}

}  // namespace collision_checking

#endif  // COLLISION_CHECKING_GEOMETRY_H_
