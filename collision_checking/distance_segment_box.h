#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_SEGMENT_BOX_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_SEGMENT_BOX_H_

// Functions for computing the minimum squared distances between a segment
// and a box.

#include <limits>
#include <type_traits>

#include "collision_checking/debug_options.h"
#include "collision_checking/distance_point_box.h"
#include "collision_checking/geometry.h"
#include "collision_checking/inlining.h"
#include "collision_checking/logging.h"
#include "googlex/proxy/eigenmath/scalar_utils.h"
#include "collision_checking/eigenmath.h"

namespace collision_checking {

template <typename Scalar>
struct SegmentBoxResult {
  Scalar distance_squared;
  Scalar segment_parameter;
  Vector3<Scalar> point_on_box;
};

// Returns the squared distance between segment and box, as well
// as the segment parameter for the closest point on the segment, and the box
// parameters for the closest point on the box.
template <typename Scalar,
          unsigned kDebugOptions = DebugOptions::kDebugOptionsNone>
CC_INLINE SegmentBoxResult<Scalar> DistanceSquared(
    const Segment<Scalar>& segment, const Box<Scalar>& box);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
namespace internal {

// Returns the distance result from a line that is parallel to a box edge, with
// the direction equal to the (positive) unit vector for `axis`.
template <typename Scalar>
CC_INLINE SegmentBoxResult<Scalar> AxisParallelLineBoxDistanceSquared(
    int axis, const Vector3<Scalar>& segment_center,
    const Vector3<Scalar>& box_half_lengths) {
  using Vector3 = Vector3<Scalar>;
  const Scalar segment_parameter =
      (box_half_lengths[axis] - segment_center[axis]);
  Vector3 projected_segment_center = segment_center;
  projected_segment_center[axis] = segment_center[axis] + segment_parameter;

  const Vector3 closest_on_box = {
      Saturate(projected_segment_center[0], box_half_lengths[0]),
      Saturate(projected_segment_center[1], box_half_lengths[1]),
      Saturate(projected_segment_center[2], box_half_lengths[2])};
  const Scalar distance_squared =
      (projected_segment_center - closest_on_box).squaredNorm();
  return {distance_squared, segment_parameter, closest_on_box};
}

// Returns the distance result from a face-parallel line to a box.
// All direction components must be positive (strictly > 0).
template <typename Scalar>
CC_INLINE SegmentBoxResult<Scalar> FaceParallelLineBoxDistanceSquared(
    int plane_index, const Vector3<Scalar>& direction,
    Vector3<Scalar> segment_center,
    const Vector3<Scalar>& box_half_lengths) {
  using Vector3 = Vector3<Scalar>;
  CC_CHECK_GE(plane_index, 0);
  CC_CHECK_LE(plane_index, 2);
  // (u,v,w) indices such that w is the plane and (u,v,w) are a right-handed
  // coordinate frame.
  const int uindex = BoxFace<Scalar>::kUIndices[plane_index];
  const int vindex = BoxFace<Scalar>::kVIndices[plane_index];
  const int windex = BoxFace<Scalar>::kWIndices[plane_index];

  // Because direction[uindex] > 0, direction[vindex] > 0, the line can either
  // intersect the box or be closest to two of the four corners.
  // Which corner is closest depends on the angle between
  // direction and the line to the segment center.
  // Components of cross-product w-component, whose sign determines the sign
  // of the angle:
  const Scalar cross_plus = direction[uindex] * segment_center[vindex];
  const Scalar cross_minus = direction[vindex] * segment_center[uindex];

  Scalar box_corner_u, box_corner_v;
  if (cross_plus > cross_minus) {  // cross = cross_plus-cross_minus  > 0.
    box_corner_u = -box_half_lengths[uindex];
    box_corner_v = box_half_lengths[vindex];
  } else {
    box_corner_u = box_half_lengths[uindex];
    box_corner_v = -box_half_lengths[vindex];
  }

  // Compute the closest point on the line to the box corner (2d).
  Scalar line_parameter =
      direction[uindex] * (box_corner_u - segment_center[uindex]) +
      direction[vindex] * (box_corner_v - segment_center[vindex]);
  // NOLINTNEXTLINE(google3-custom-no-eigen-default-ctor): Initialized below.
  Vector3 line_point;
  line_point[uindex] =
      segment_center[uindex] + line_parameter * direction[uindex];
  line_point[vindex] =
      segment_center[vindex] + line_parameter * direction[vindex];
  line_point[windex] = segment_center[windex];

  // Check for box intersection: if line_point is not beyond the
  // box_corner, there is an intersection.
  if (((box_corner_u > 0) && (line_point[uindex] < box_corner_u)) ||
      ((box_corner_u < 0) && (line_point[uindex] > box_corner_u))) {
    line_point[uindex] = box_corner_u;
    // Division by direction[uindex] is safe, as it is > kParallelEpsilon.
    line_parameter =
        (line_point[uindex] - segment_center[uindex]) / direction[uindex];
    line_point[vindex] =
        segment_center[vindex] + direction[vindex] * line_parameter;

    if (line_point[vindex] > box_half_lengths[vindex]) {
      line_point[vindex] = box_half_lengths[vindex];
      line_parameter =
          (line_point[vindex] - segment_center[vindex]) / direction[vindex];
      line_point[uindex] =
          segment_center[uindex] + direction[uindex] * line_parameter;
    } else if (line_point[vindex] < -box_half_lengths[vindex]) {
      line_point[vindex] = -box_half_lengths[vindex];
      // Division by direction[vindex] is safe, as it is > kParallelEpsilon.
      line_parameter =
          (line_point[vindex] - segment_center[vindex]) / direction[vindex];
      line_point[uindex] =
          segment_center[uindex] + direction[uindex] * line_parameter;
    }
  }

  const Vector3 closest_on_box = {Saturate(line_point[0], box_half_lengths[0]),
                                  Saturate(line_point[1], box_half_lengths[1]),
                                  Saturate(line_point[2], box_half_lengths[2])};
  const Scalar distance_squared = (line_point - closest_on_box).squaredNorm();
  return {distance_squared, line_parameter, closest_on_box};
}

// Returns the distance result between a face and a line.
// All components of direction must be positive and non-zero.
template <typename Scalar>
CC_INLINE SegmentBoxResult<Scalar> LineFaceDistanceSquared(
    int plane_index, const Vector3<Scalar>& direction,
    Vector3<Scalar> segment_center,
    const Vector3<Scalar>& box_half_lengths) {
  using Vector3 = Vector3<Scalar>;
  CC_CHECK_GE(plane_index, 0);
  CC_CHECK_LE(plane_index, 2);
  const int uindex = BoxFace<Scalar>::kUIndices[plane_index];
  const int vindex = BoxFace<Scalar>::kVIndices[plane_index];
  const int windex = BoxFace<Scalar>::kWIndices[plane_index];

  // Compute the line / face-plane intersection: if it is within the face,
  // the distance is zero.
  const Scalar sintersect =
      (box_half_lengths[windex] - segment_center[windex]) / direction[windex];
  const Scalar uintersect =
      segment_center[uindex] + sintersect * direction[uindex];
  const Scalar vintersect =
      segment_center[vindex] + sintersect * direction[vindex];
  if ((std::abs(uintersect) < box_half_lengths[uindex]) &&
      (std::abs(vintersect) < box_half_lengths[vindex])) {
    // NOLINTNEXTLINE(google3-custom-no-eigen-default-ctor): Initialized below.
    Vector3 intersect_point;
    intersect_point[windex] = box_half_lengths[windex];
    intersect_point[uindex] = uintersect;
    intersect_point[vindex] = vintersect;
    return {Scalar{0}, sintersect, intersect_point};
  }

  // If the line doesn't intersect the face, one of the edges is closest to the
  // line. Of the four edges, only two can be minimal, because `direction` has
  // only positive components:
  //  u=-box_half_lengths[uindex] and v=-box_half_lengths[vindex].
  // The following code just computes both, then selects the smaller.
  const Vector3 direction_squared = direction.array() * direction.array();
  // Segment center w-coordinate from positive w face.
  const Scalar w_pos_offset = box_half_lengths[windex] - segment_center[windex];
  // Segment center u offset from negative u face.
  const Scalar u_neg_offset =
      -box_half_lengths[uindex] - segment_center[uindex];
  // Segment center v offset from negative v face.
  const Scalar v_neg_offset =
      -box_half_lengths[vindex] - segment_center[vindex];

  // Distance to u=-box_half_lengths[uindex]:
  // NOLINTNEXTLINE(google3-custom-no-eigen-default-ctor): Initialized below.
  Vector3 face_point_u;
  face_point_u[windex] = box_half_lengths[windex];
  face_point_u[uindex] = -box_half_lengths[uindex];
  Scalar smin_u =
      Scalar{1} / (direction_squared[uindex] + direction_squared[windex]) *
      (direction[uindex] * u_neg_offset + direction[windex] * w_pos_offset);
  face_point_u[vindex] =
      Saturate(segment_center[vindex] + direction[vindex] * smin_u,
               box_half_lengths[vindex]);
  smin_u = direction.dot(face_point_u - segment_center);
  const Scalar distance_squared_u =
      (segment_center + smin_u * direction - face_point_u).squaredNorm();

  // Distance to v=-box_half_lengths[vindex]:
  // NOLINTNEXTLINE(google3-custom-no-eigen-default-ctor): Initialized below.
  Vector3 face_point_v;
  face_point_v[windex] = box_half_lengths[windex];
  face_point_v[vindex] = -box_half_lengths[vindex];
  Scalar smin_v =
      Scalar{1} / (direction_squared[vindex] + direction_squared[windex]) *
      (direction[vindex] * v_neg_offset + direction[windex] * w_pos_offset);
  face_point_v[uindex] =
      Saturate(segment_center[uindex] + direction[uindex] * smin_v,
               box_half_lengths[uindex]);
  smin_v = direction.dot(face_point_v - segment_center);
  const Scalar distance_squared_v =
      (segment_center + smin_v * direction - face_point_v).squaredNorm();

  if (distance_squared_u < distance_squared_v) {
    return {distance_squared_u, smin_u, face_point_u};
  } else {
    return {distance_squared_v, smin_v, face_point_v};
  }
}

// Returns the distance result between a line and a box (ie, not taking the
// segment extent into account).
// This function assumes that all components in direction are positive.
template <typename Scalar>
CC_INLINE SegmentBoxResult<Scalar> LineBoxDistanceSquared(
    const Vector3<Scalar>& direction,
    const Vector3<Scalar>& segment_center,
    const Vector3<Scalar>& box_half_lengths) {
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kParallelEpsilon =
      std::numeric_limits<Scalar>::epsilon() * 100;

  const Vector3 abs_direction = direction.cwiseAbs();
  if (abs_direction[0] < kParallelEpsilon) {
    if (abs_direction[1] < kParallelEpsilon) {
      // Parallel to z-axis.
      return internal::AxisParallelLineBoxDistanceSquared(2, segment_center,
                                                          box_half_lengths);
    }
    if (abs_direction[2] < kParallelEpsilon) {
      // Parallel to y-axis.
      return internal::AxisParallelLineBoxDistanceSquared(1, segment_center,
                                                          box_half_lengths);
    }
    // Parallel to yz-face.
    return internal::FaceParallelLineBoxDistanceSquared(
        0, direction, segment_center, box_half_lengths);
  }
  if (abs_direction[1] < kParallelEpsilon) {
    if (abs_direction[2] < kParallelEpsilon) {
      // Parallel to x-axis.
      return internal::AxisParallelLineBoxDistanceSquared(0, segment_center,
                                                          box_half_lengths);
    }
    // Parallel xz-face.
    return internal::FaceParallelLineBoxDistanceSquared(
        1, direction, segment_center, box_half_lengths);
  }
  if (abs_direction[2] < kParallelEpsilon) {
    // Parallel to xy-face.
    return internal::FaceParallelLineBoxDistanceSquared(
        2, direction, segment_center, box_half_lengths);
  }

  // All direction components are non-zero and strictly positive.
  int face_index;
  // Determine which face is closest by looking at the sign of the angle
  // between the direction and the line connecting the face center and the
  // positive box corner.
  const Vector3 cross = direction.cross(segment_center - box_half_lengths);
  if (cross[2] < Scalar{0}) {
    if (cross[1] > Scalar{0}) {
      // line intersects x = box_half_length[0] plane
      face_index = 0;
    } else {
      // line intersects z = box_half_length[2] plane
      face_index = 2;
    }
  } else {
    if (cross[0] < Scalar{0}) {
      // line intersects y = box_half_length[1] plane
      face_index = 1;
    } else {
      // line intersects z =box_half_length[2] plane
      face_index = 2;
    }
  }

  return internal::LineFaceDistanceSquared(face_index, direction,
                                           segment_center, box_half_lengths);
}
}  // namespace internal

template <typename Scalar, unsigned kDebugOptions>
CC_INLINE SegmentBoxResult<Scalar> DistanceSquared(
    const Segment<Scalar>& segment, const Box<Scalar>& box) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating point type.");
  if constexpr (kDebugOptions &
                DebugOptions::kDebugOptionsPerformExpensiveInputChecks) {
    CC_CHECK_LT(std::abs(segment.direction.squaredNorm() - Scalar{1}),
                       std::numeric_limits<Scalar>::epsilon() * 10);
    CC_CHECK_GE(segment.half_length, Scalar{0});
    BLUE_CUDA_KERNEL_CHECK((box.half_lengths.array(), Scalar{0}).all());
    CC_CHECK_LT(
        (box.box_rotation_world * box.box_rotation_world.transpose() -
         Matrix3<Scalar>::Identity())
            .norm(),
        std::numeric_limits<Scalar>::epsilon() * 10);
  }
  // This solves the minimum distance problem:
  // (box.center+box.box_rotation_world^T*[u;v;w] -
  //  segment.center-segment.direction*s)^2  -> min,
  //  s.t. |[u;v;w]|_i <= box.half_lengths[i],
  //  and |s| <= segment.half_length.
  // Segment parameters in box frame.
  const Vector3<Scalar> segment_center =
      box.box_rotation_world * (segment.center - box.center);
  const Vector3<Scalar> direction = box.box_rotation_world * segment.direction;

  // In box-centered coordinates, the problem is symmetric, so we can always
  // solve a problem where all segment direction components are positive.
  // This limits the number of geometric constellations that must be considered.
  const Vector3<Scalar> signs(
      direction[0] >= 0 ? Scalar{1} : Scalar{-1},
      direction[1] >= 0 ? Scalar{1} : Scalar{-1},
      direction[2] >= 0 ? Scalar{1} : Scalar{-1});
  Vector3<Scalar> positive_direction = direction.cwiseAbs();
  Vector3<Scalar> positive_center = segment_center.cwiseProduct(signs);

  auto [line_distance_squared, line_segment_parameter, line_closest_on_box] =
      internal::LineBoxDistanceSquared<Scalar>(
          positive_direction, positive_center, box.half_lengths);
  // Mirror the closest point on the box, if necessary.
  line_closest_on_box = line_closest_on_box.cwiseProduct(signs);

  // If the segment parameter is out of range, the closest point one of the
  // segment end points.
  if (line_segment_parameter > segment.half_length) {
    const Vector3<Scalar> segment_point =
        segment_center + segment.half_length * direction;
    const auto [endpoint_distance_squared, endpoint_closest_on_box] =
        internal::PointBoxDistanceSquared(segment_point, box.half_lengths);
    return {endpoint_distance_squared, segment.half_length,
            endpoint_closest_on_box};
  } else if (line_segment_parameter < -segment.half_length) {
    const Vector3<Scalar> segment_point =
        segment_center - segment.half_length * direction;
    const auto [endpoint_distance_squared, endpoint_closest_on_box] =
        internal::PointBoxDistanceSquared(segment_point, box.half_lengths);
    return {endpoint_distance_squared, -segment.half_length,
            endpoint_closest_on_box};
  } else {
    return {line_distance_squared, line_segment_parameter, line_closest_on_box};
  }
}
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_DISTANCE_SEGMENT_BOX_H_
