#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_POINT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_POINT_H_

// Functions for computing the minimum squared distances between geometric
// primitives.

#include <type_traits>

#include "experimental/users/buschmann/collision_checking/geometry.h"
#include "experimental/users/buschmann/collision_checking/inlining.h"
#include "experimental/users/buschmann/collision_checking/eigenmath.h"

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

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_POINT_POINT_H_
