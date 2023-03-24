#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_BOX_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_BOX_H_

// Functions for computing the minimum squared distances between geometric
// primitives.

#include <limits>
#include <type_traits>

#include "experimental/users/buschmann/collision_checking/debug_options.h"
#include "experimental/users/buschmann/collision_checking/geometry.h"
#include "experimental/users/buschmann/collision_checking/inlining.h"
#include "googlex/proxy/eigenmath/scalar_utils.h"
#include "experimental/users/buschmann/collision_checking/eigenmath.h"

namespace collision_checking {

template <typename Scalar>
struct PointBoxResult {
  Scalar distance_squared;
  Vector3<Scalar> point_on_box;
};

// Returns the squared distance between `point` and `box`, as well
// as the parameters for the closest point on the box.
template <typename Scalar, unsigned kDebugOptions = kDebugOptionsNone>
CC_INLINE PointBoxResult<Scalar> DistanceSquared(
    const Point<Scalar>& point, const Box<Scalar>& box);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

namespace internal {
// Minimum distance squared and closest point to box in box coordinate frame.
template <typename Scalar>
CC_INLINE PointBoxResult<Scalar> PointBoxDistanceSquared(
    const Vector3<Scalar>& point_box,
    const Vector3<Scalar>& box_half_lengths) {
  // Projected solution (equivalent to closest point on box in box-local frame).
  // Note: could avoid a few ops by unrolling these operations explicitly.
  const Vector3<Scalar> point_box_proj = Vector3<Scalar>(
      std::clamp(point_box[0], -box_half_lengths[0], box_half_lengths[0]),
      std::clamp(point_box[1], -box_half_lengths[1], box_half_lengths[1]),
      std::clamp(point_box[2], -box_half_lengths[2], box_half_lengths[2]));
  const Scalar distance_squared = (point_box - point_box_proj).squaredNorm();
  return {.distance_squared = distance_squared, .point_on_box = point_box_proj};
}
}  // namespace internal

template <typename Scalar, unsigned kDebugOptions>
CC_INLINE PointBoxResult<Scalar> DistanceSquared(
    const Point<Scalar>& point, const Box<Scalar>& box) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating point type.");

  if constexpr (kDebugOptions &
                DebugOptions::kDebugOptionsPerformExpensiveInputChecks) {
    BLUE_CHECK((box.half_lengths.array() >= Scalar{0}).all(),
               "box.half_lengths must be positive.");
    BLUE_CHECK((box.box_rotation_world * box.box_rotation_world.transpose() -
                Matrix3<Scalar>::Identity())
                       .norm() < std::numeric_limits<Scalar>::epsilon() * 10,
               "box.box_rotation_world must be a valid rotation matrix.");
  }
  // This solves the minimum distance problem:
  // min (box.center+box.box_rotation_world^T*[u;v;w]-point)^2 -> min,
  //  s.t. |[u;v;w]|_i <= box.half_lengths[i]
  // The Hessian is the identity matrix, so the solution is directly obtained
  // from projecting the unconstrained solution onto the box constraints.

  // Unconstrained solution (equivalent to point coordinates in box-local
  // frame).
  const Vector3<Scalar> point_box =
      box.box_rotation_world * (point.center - box.center);
  return internal::PointBoxDistanceSquared(point_box, box.half_lengths);
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_BOX_H_
