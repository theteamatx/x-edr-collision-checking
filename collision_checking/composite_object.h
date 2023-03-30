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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COMPOSITE_OBJECT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COMPOSITE_OBJECT_H_

#include "collision_checking/distance_box_box.h"
#include "collision_checking/distance_point_box.h"
#include "collision_checking/distance_point_point.h"
#include "collision_checking/distance_point_segment.h"
#include "collision_checking/distance_segment_box.h"
#include "collision_checking/distance_segment_segment.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/inlining.h"
#include "collision_checking/normalize_and_maybe_log.h"
#include "collision_checking/object_id.h"
#include "collision_checking/options.h"
#include "collision_checking/vector.h"

// This file contains classes and functions for collision models
// containing groups of objects and objects composed of multiple
// geometric primitives.

namespace collision_checking {

// A count of the number of supported primitives, e.g., in a compsite shape.
struct PrimitivesCount {
  size_t num_spheres = 0;
  size_t num_capsules = 0;
  size_t num_boxes = 0;
};

// A composite object grouping a number of geometric primitives, as well as
// collision flags and an axis-aligned bounding box.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
struct CompositeObject : public ParametrizedNewDelete<AllocatorTraits> {
  void ResizeBuffers(const PrimitivesCount& primitives);

  // Assign `other` to `this` after transforming it according to the provided
  // rotation matrix and translation.
  // Asserts that object sizes match.
  CC_INLINE
  void AssignTransformedShape(
      const Vector3<Scalar>& world_translation_object,
      const Matrix3<Scalar>& world_rotation_object,
      const CompositeObject<Scalar, AllocatorTraits>& other);

  ObjectFlags flags;
  AlignedBox<Scalar> aabb;
  Vector<Sphere<Scalar>, AllocatorTraits> spheres;
  Vector<Capsule<Scalar>, AllocatorTraits> capsules;
  Vector<Box<Scalar>, AllocatorTraits> boxes;
};

// A list of (composite) collision objects.
// This is usually the collision geometry of a robot/assembly.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
struct CollisionObjects : public ParametrizedNewDelete<AllocatorTraits> {
  void ResizeBuffers(absl::Span<const PrimitivesCount> primitives) {
    objects.resize(primitives.size());
    for (int i = 0; i < primitives.size(); i++) {
      objects[i].ResizeBuffers(primitives[i]);
    }
  }

  AlignedBox<Scalar> aabb;
  Vector<CompositeObject<Scalar, AllocatorTraits>, AllocatorTraits> objects;
};

// Update CompositeObject bounding box.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
CC_INLINE void UpdateAlignedBox(
    Scalar padding, CompositeObject<Scalar, AllocatorTraits>& updated_object) {
  SetEmptySize(&updated_object.aabb);
  for (const auto& sphere : updated_object.spheres) {
    GrowAlignedBoxAround(sphere, &updated_object.aabb);
  }
  for (const auto& capsule : updated_object.capsules) {
    GrowAlignedBoxAround(capsule, &updated_object.aabb);
  }
  for (const auto& box : updated_object.boxes) {
    GrowAlignedBoxAround(box, &updated_object.aabb);
  }
  updated_object.aabb.low.array() -= padding;
  updated_object.aabb.high.array() += padding;
}

// Updates all AlignedBoxes in objects, adding padding to all
// bounding box sizes.
// The overall bounding box for `moving_objects` excludes objects that
// do not include the obstacles (kVoxelMapIdSet) in the
// inclusion bitmask.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
CC_INLINE void UpdateAlignedBoxes(
    Scalar padding,
    CollisionObjects<Scalar, AllocatorTraits>& updated_moving_objects) {
  SetEmptySize(&updated_moving_objects.aabb);
  for (auto& object : updated_moving_objects.objects) {
    if (object.flags.inclusion_set.Empty()) {
      continue;
    }
    UpdateAlignedBox(padding, object);
    // Only grow overall bounding box if the object might be checked against
    // obstacles.
    if (object.flags.inclusion_set.Overlaps(kVoxelMapIdSet))
      GrowAlignedBoxAround(object.aabb, &updated_moving_objects.aabb);
  }
}

// Result for a call to CompositeObjectDistance.
template <typename Scalar>
struct CompositObjectDistanceResult {
  // Distance between composite object a and b.
  Scalar distance;
  // Point on object a for distance.
  Vector3<Scalar> contact_a;
  // Point on object b for distance.
  Vector3<Scalar> contact_b;
  // Normal from object a to object b for distance.
  Vector3<Scalar> a_normal_b;
};

// Returns the minimum distance between two composite objects, as well as the
// corresponding contact points and normal from object_a to object_b.
// The contact points and normal are only valid if options.GetType() >=
// kComputeContactPoints.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
CC_INLINE CompositObjectDistanceResult<Scalar> CompositeObjectDistance(
    const CompositeObject<Scalar, AllocatorTraits>& object_a,
    const CompositeObject<Scalar, AllocatorTraits>& object_b,
    const QueryOptions& options);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

template <typename Scalar, typename AllocatorTraits>
void CompositeObject<Scalar, AllocatorTraits>::ResizeBuffers(
    const PrimitivesCount& primitives) {
  spheres.resize(primitives.num_spheres);
  capsules.resize(primitives.num_capsules);
  boxes.resize(primitives.num_boxes);
}

template <typename Scalar, typename AllocatorTraits>
CC_INLINE void CompositeObject<Scalar, AllocatorTraits>::AssignTransformedShape(
    const Vector3<Scalar>& world_translation_object,
    const Matrix3<Scalar>& world_rotation_object,
    const CompositeObject<Scalar, AllocatorTraits>& other) {
  for (int sidx = 0; sidx < spheres.size(); ++sidx) {
    spheres[sidx].center = world_translation_object +
                           world_rotation_object * other.spheres[sidx].center;
  }
  for (int cidx = 0; cidx < capsules.size(); ++cidx) {
    capsules[cidx].center = world_translation_object +
                            world_rotation_object * other.capsules[cidx].center;
    capsules[cidx].direction =
        world_rotation_object * other.capsules[cidx].direction;
  }
  for (int bidx = 0; bidx < boxes.size(); ++bidx) {
    boxes[bidx].center = world_translation_object +
                         world_rotation_object * other.boxes[bidx].center;
    boxes[bidx].box_rotation_world = other.boxes[bidx].box_rotation_world *
                                     world_rotation_object.transpose();
  }
}

// Returns the minimum distance between two composite objects, as well as the
// corresponding contact points and normal from object_a to object_b.
// The contact points and normal are only valid if options.GetType() >=
// kComputeContactPoints.
template <typename Scalar, typename AllocatorTraits>
CC_INLINE CompositObjectDistanceResult<Scalar> CompositeObjectDistance(
    const CompositeObject<Scalar, AllocatorTraits>& object_a,
    const CompositeObject<Scalar, AllocatorTraits>& object_b,
    const QueryOptions& options) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating point type.");
  Scalar min_distance = std::numeric_limits<Scalar>::infinity();

  enum class ContactCase {
    kNone,
    kSphereSphere,
    kSphereCapsule,
    kSphereBox,
    kCapsuleSphere,
    kCapsuleCapsule,
    kCapsuleBox,
    kBoxSphere,
    kBoxCapsule,
    kBoxBox,
  };
  ContactCase min_contact_case = ContactCase::kNone;
  const Sphere<Scalar>* min_sphere_a = nullptr;
  const Sphere<Scalar>* min_sphere_b = nullptr;
  const Capsule<Scalar>* min_capsule_a = nullptr;
  const Capsule<Scalar>* min_capsule_b = nullptr;
  const Box<Scalar>* min_box_a = nullptr;
  const Box<Scalar>* min_box_b = nullptr;
  Scalar min_param_a = 0;
  Scalar min_param_b = 0;
  Vector3<Scalar> min_point_a = Vector3<Scalar>::Zero();
  Vector3<Scalar> min_point_b = Vector3<Scalar>::Zero();

  const bool compute_contact_points =
      options.GetType() == QueryOptions::kComputeContactPoints;

  for (const auto& sphere_a : object_a.spheres) {
    // Sphere object_a / sphere object_b distances.
    for (const auto& sphere_b : object_b.spheres) {
      const Scalar ra_plus_rb = sphere_a.radius + sphere_b.radius;
      const auto center_distance_result = DistanceSquared(sphere_a, sphere_b);
      const Scalar distance =
          std::sqrt(center_distance_result.distance_squared) - ra_plus_rb;
      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }

      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_sphere_a = &sphere_a;
          min_sphere_b = &sphere_b;
          min_contact_case = ContactCase::kSphereSphere;
        }
      }
    }
  }

  // Sphere object_a / Capsule object_b distances.
  for (const auto& sphere_a : object_a.spheres) {
    for (const auto& capsule : object_b.capsules) {
      const Scalar ra_plus_rb = capsule.radius + sphere_a.radius;
      const auto [center_distance_squared, min_segment_param] =
          DistanceSquared(sphere_a, capsule);
      const Scalar distance = std::sqrt(center_distance_squared) - ra_plus_rb;
      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }
      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_param_b = min_segment_param;
          min_sphere_a = &sphere_a;
          min_capsule_b = &capsule;
          min_contact_case = ContactCase::kSphereCapsule;
        }
      }
    }
  }

  // Sphere object_b / Capsule object_a distances.
  for (const auto& sphere_b : object_b.spheres) {
    for (const auto& capsule : object_a.capsules) {
      const Scalar ra_plus_rb = capsule.radius + sphere_b.radius;
      const auto [center_distance_squared, min_segment_param] =
          DistanceSquared(sphere_b, capsule);
      const Scalar distance = std::sqrt(center_distance_squared) - ra_plus_rb;

      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }

      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_param_a = min_segment_param;
          min_capsule_a = &capsule;
          min_sphere_b = &sphere_b;
          min_contact_case = ContactCase::kCapsuleSphere;
        }
      }
    }
  }

  // Capsule / Capsule distances.
  for (const auto& capsule_a : object_a.capsules) {
    for (const auto& capsule_b : object_b.capsules) {
      const Scalar ra_plus_rb = capsule_a.radius + capsule_b.radius;
      const auto [center_distance_squared, min_segment_param_a,
                  min_segment_param_b] =
          SegmentSegmentDistanceSquared(capsule_a, capsule_b);
      const Scalar distance = std::sqrt(center_distance_squared) - ra_plus_rb;

      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }

      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_param_a = min_segment_param_a;
          min_param_b = min_segment_param_b;
          min_capsule_a = &capsule_a;
          min_capsule_b = &capsule_b;
          min_contact_case = ContactCase::kCapsuleCapsule;
        }
      }
    }
  }

  for (const auto& box_a : object_a.boxes) {
    // Sphere object_b / Box object_a distances.
    for (const auto& sphere_b : object_b.spheres) {
      const auto [center_distance_squared, point_on_box] =
          DistanceSquared(sphere_b, box_a);
      const Scalar distance =
          std::sqrt(center_distance_squared) - sphere_b.radius;
      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }
      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_sphere_b = &sphere_b;
          min_box_a = &box_a;
          min_point_a = point_on_box;
          min_contact_case = ContactCase::kBoxSphere;
        }
      }
    }

    // Box / Capsule distances.
    for (const auto& capsule : object_b.capsules) {
      const auto [center_distance_squared, segment_param, point_on_box] =
          DistanceSquared(capsule, box_a);
      const Scalar distance =
          std::sqrt(center_distance_squared) - capsule.radius;

      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }

      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_capsule_b = &capsule;
          min_box_a = &box_a;
          min_point_a = point_on_box;
          min_param_b = segment_param;
          min_contact_case = ContactCase::kBoxCapsule;
        }
      }
    }
    // Box / Box
    for (const auto& box_b : object_b.boxes) {
      const auto [distance_squared, point_a, point_b] =
          DistanceSquared(box_a, box_b);
      const Scalar distance = std::sqrt(distance_squared);

      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }
      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_box_a = &box_a;
          min_box_b = &box_b;
          min_point_a = point_a;
          min_point_b = point_b;
          min_contact_case = ContactCase::kBoxBox;
        }
      }
    }
  }

  for (const auto& box_b : object_b.boxes) {
    // Sphere object_a / Box object_b distances.
    for (const auto& sphere_a : object_a.spheres) {
      const auto [center_distance_squared, point_on_box] =
          DistanceSquared(sphere_a, box_b);
      const Scalar distance =
          std::sqrt(center_distance_squared) - sphere_a.radius;
      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }
      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_sphere_a = &sphere_a;
          min_box_b = &box_b;
          min_point_b = point_on_box;
          min_contact_case = ContactCase::kSphereBox;
        }
      }
    }
    // Capsule / Box distances.
    for (const auto& capsule_a : object_a.capsules) {
      const auto [center_distance_squared, segment_param, point_on_box] =
          DistanceSquared(capsule_a, box_b);
      const Scalar distance =
          std::sqrt(center_distance_squared) - capsule_a.radius;

      // Terminate early if options allow it.
      if (options.GetType() == QueryOptions::kIsCollisionFree &&
          distance <= Scalar{0}) {
        return {.distance = distance};
      }

      if (distance < min_distance) {
        min_distance = distance;
        if (compute_contact_points) {
          min_capsule_a = &capsule_a;
          min_box_b = &box_b;
          min_point_b = point_on_box;
          min_param_a = segment_param;
          min_contact_case = ContactCase::kCapsuleBox;
        }
      }
    }
  }
  if (!compute_contact_points) {
    return {.distance = min_distance};
  }

  // If contact point computation was requested, redo the minimum distance
  // computation for the minimum contact pair with contact point computation.
  switch (min_contact_case) {
    case ContactCase::kNone: {
      return {.distance = min_distance};
    };
    case ContactCase::kSphereSphere: {
      CC_CHECK_NE(min_sphere_a, nullptr);
      CC_CHECK_NE(min_sphere_b, nullptr);
      const Vector3<Scalar> a_normal_b =
          (min_sphere_b->center - min_sphere_a->center).normalized();
      return {
          .distance = min_distance,
          .contact_a = min_sphere_a->center + a_normal_b * min_sphere_a->radius,
          .contact_b = min_sphere_b->center - a_normal_b * min_sphere_b->radius,
          .a_normal_b = a_normal_b};
    }
    case ContactCase::kSphereCapsule: {
      CC_CHECK_NE(min_sphere_a, nullptr);
      CC_CHECK_NE(min_capsule_b, nullptr);
      const Vector3<Scalar> contact_b =
          min_capsule_b->center + min_capsule_b->direction * min_param_b;
      const Vector3<Scalar> a_normal_b =
          (contact_b - min_sphere_a->center).normalized();
      return {
          .distance = min_distance,
          .contact_a = min_sphere_a->center + a_normal_b * min_sphere_a->radius,
          .contact_b = contact_b - a_normal_b * min_capsule_b->radius,
          .a_normal_b = a_normal_b};
    }
    case ContactCase::kSphereBox: {
      CC_CHECK_NE(min_sphere_a, nullptr);
      CC_CHECK_NE(min_box_b, nullptr);
      // Note: if the sphere center is exactly on the box boundary, the normal
      // computation fails. Consider computing normal from box face instead in
      // these cases.
      const Vector3<Scalar> contact_b =
          min_box_b->box_rotation_world.transpose() * min_point_b +
          min_box_b->center;
      const Vector3<Scalar> a_normal_b =
          internal::NormalizeAndMaybeLog(contact_b - min_sphere_a->center);

      return {
          .distance = min_distance,
          .contact_a = min_sphere_a->center + a_normal_b * min_sphere_a->radius,
          .contact_b = contact_b,
          .a_normal_b = a_normal_b};
    }
    case ContactCase::kCapsuleSphere: {
      CC_CHECK_NE(min_sphere_b, nullptr);
      CC_CHECK_NE(min_capsule_a, nullptr);
      const Vector3<Scalar> contact_a =
          min_capsule_a->center + min_capsule_a->direction * min_param_a;
      const Vector3<Scalar> a_normal_b =
          (min_sphere_b->center - contact_a).normalized();
      return {
          .distance = min_distance,
          .contact_a = contact_a + a_normal_b * min_capsule_a->radius,
          .contact_b = min_sphere_b->center - a_normal_b * min_sphere_b->radius,
          .a_normal_b = a_normal_b};
    }
    case ContactCase::kCapsuleCapsule: {
      CC_CHECK_NE(min_capsule_a, nullptr);
      CC_CHECK_NE(min_capsule_b, nullptr);
      const Vector3<Scalar> contact_a =
          min_capsule_a->center + min_capsule_a->direction * min_param_a;
      const Vector3<Scalar> contact_b =
          min_capsule_b->center + min_capsule_b->direction * min_param_b;
      const Vector3<Scalar> a_normal_b = (contact_b - contact_a).normalized();
      return {.distance = min_distance,
              .contact_a = contact_a + a_normal_b * min_capsule_a->radius,
              .contact_b = contact_b - a_normal_b * min_capsule_b->radius,
              .a_normal_b = a_normal_b};
    }
    case ContactCase::kCapsuleBox: {
      CC_CHECK_NE(min_capsule_a, nullptr);
      CC_CHECK_NE(min_box_b, nullptr);
      const Vector3<Scalar> contact_a =
          min_capsule_a->center + min_capsule_a->direction * min_param_a;
      const Vector3<Scalar> contact_b =
          min_box_b->box_rotation_world.transpose() * min_point_b +
          min_box_b->center;
      const Vector3<Scalar> a_normal_b = (contact_b - contact_a).normalized();
      return {.distance = min_distance,
              .contact_a = contact_a + a_normal_b * min_capsule_a->radius,
              .contact_b = contact_b,
              .a_normal_b = a_normal_b};
    }
    case ContactCase::kBoxSphere: {
      CC_CHECK_NE(min_sphere_b, nullptr);
      CC_CHECK_NE(min_box_a, nullptr);
      const Vector3<Scalar> contact_a =
          min_box_a->box_rotation_world.transpose() * min_point_a +
          min_box_a->center;
      // Note: if the sphere center is exactly on the box boundary, the normal
      // computation fails. Consider computing normal from box face instead in
      // these cases.
      const Vector3<Scalar> a_normal_b =
          internal::NormalizeAndMaybeLog(min_sphere_b->center - contact_a);
      return {
          .distance = min_distance,
          .contact_a = contact_a,
          .contact_b = min_sphere_b->center - a_normal_b * min_sphere_b->radius,
          .a_normal_b = a_normal_b};
    }
    case ContactCase::kBoxCapsule: {
      CC_CHECK_NE(min_capsule_b, nullptr);
      CC_CHECK_NE(min_box_a, nullptr);
      const Vector3<Scalar> contact_a =
          min_box_a->box_rotation_world.transpose() * min_point_a +
          min_box_a->center;
      const Vector3<Scalar> contact_b =
          min_capsule_b->center + min_capsule_b->direction * min_param_b;
      const Vector3<Scalar> a_normal_b = (contact_b - contact_a).normalized();
      return {.distance = min_distance,
              .contact_a = contact_a,
              .contact_b = contact_b - a_normal_b * min_capsule_b->radius,
              .a_normal_b = a_normal_b};
    }
    case ContactCase::kBoxBox: {
      CC_CHECK_NE(min_box_a, nullptr);
      CC_CHECK_NE(min_box_b, nullptr);
      const Vector3<Scalar> a_normal_b =
          (min_point_b - min_point_a).normalized();
      return {.distance = min_distance,
              .contact_a = min_point_a,
              .contact_b = min_point_b,
              .a_normal_b = a_normal_b};
    }
      // No default case, to catch any additions to contact combinations.
  }
  CC_PANIC(
      "Should not be able to reach this line: Missing return in switch/case "
      "statement?");
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COMPOSITE_OBJECT_H_
