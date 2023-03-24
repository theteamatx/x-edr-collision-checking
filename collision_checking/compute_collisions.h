#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COMPUTE_COLLISIONS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COMPUTE_COLLISIONS_H_

#include "collision_checking/collision_result.h"
#include "collision_checking/composite_object.h"
#include "collision_checking/normalize_and_maybe_log.h"
#include "collision_checking/options.h"
#include "collision_checking/status.h"
#include "collision_checking/voxel_map_object.h"

// This file contains a template for performing collision queries.

namespace collision_checking {

// Perform one collision check according to the specified options.
// The result object must be resized to match obstacles.
// Suitable for real-time use.
template <typename Scalar, typename AllocatorTraits>
Status ComputeCollisions(
    const VoxelMapObject<Scalar>& obstacles,
    const CollisionObjects<Scalar, AllocatorTraits>& moving,
    const QueryOptions& options,
    CollisionResult<Scalar, AllocatorTraits>& result);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
template <typename Scalar, typename AllocatorTraits>
Status ComputeCollisions(
    const VoxelMapObject<Scalar>& obstacles,
    const CollisionObjects<Scalar, AllocatorTraits>& moving,
    const QueryOptions& options,
    CollisionResult<Scalar, AllocatorTraits>& result) {
  static_assert(std::is_floating_point_v<Scalar>,
                "Scalar must be a floating point type.");

  const int num_moving = moving.objects.size();
  if (result.GetObjectCount() != num_moving) {
    return Status(absl::StatusCode::kInvalidArgument, Status::kResultSizeWrong);
  }

  if (!result.MemoryIsAllocatedFor(options)) {
    return Status(absl::StatusCode::kInvalidArgument,
                  Status::kResultOptionsWrong);
  }

  result.Reset();

  // Loop every every possible object / object collision pair and perform
  // collision checking on the ones that are not disabled and whose bounding
  // boxes overlap.
  for (int first = 0; first < num_moving; first++) {
    const auto& first_object = moving.objects[first];
    result.GetObjectIdSet(first) = first_object.flags.id_set;
    if (options.GetExclusionSet().Overlaps(first_object.flags.id_set)) {
      continue;
    }
    for (int second = first + 1; second < num_moving; second++) {
      const auto& second_object = moving.objects[second];
      if (options.GetExclusionSet().Overlaps(second_object.flags.id_set)) {
        continue;
      }
      if (!ShouldCheckPair(first_object.flags, second_object.flags)) {
        continue;
      }
      if (!DoOverlap(first_object.aabb, second_object.aabb)) {
        continue;
      }
      const auto [distance, contact_a, contact_b, a_normal_b] =
          CompositeObjectDistance(first_object, second_object, options);
      if (distance <= Scalar{0}) {
        result.GetObjectHits(first).Insert(second_object.flags.id_set);
        result.GetObjectHits(second).Insert(first_object.flags.id_set);
        result.SetHasCollisions(true);

        // Terminate early if options allow it.
        if (options.GetType() == QueryOptions::kIsCollisionFree) {
          return OkStatus();
        }
      }
      if (options.GetType() < QueryOptions::kComputeObjectDistances) {
        continue;
      }

      if (distance < result.GetMinimumDistance(first)) {
        result.GetMinimumDistance(first) = distance;
        if (options.GetType() >= QueryOptions::kComputeContactPoints) {
          result.GetContactPointInfo(first) = {contact_a, contact_b, a_normal_b,
                                               second_object.flags.id_set};
        }
      }
      if (distance < result.GetMinimumDistance(second)) {
        result.GetMinimumDistance(second) = distance;
        if (options.GetType() >= QueryOptions::kComputeContactPoints) {
          result.GetContactPointInfo(second) = {
              contact_b, contact_a, -a_normal_b, first_object.flags.id_set};
        }
      }

      result.SetMinimumDistance(std::min({result.GetMinimumDistance(),
                                          result.GetMinimumDistance(first),
                                          result.GetMinimumDistance(second)}));
    }
  }

  // Compute collisions with static obstacles by computing the distance to
  // moving objects, unless disabled or outside the aabb.
  // Note: This doesn't reuse CompositeObjectDistance(), as it is
  // significantly slower for a large number of obstacle spheres, e.g., from a
  // voxel map. The reason is the missing bounding box check of individual
  // spheres against the robot links.
  if (options.GetExclusionSet().Overlaps(kVoxelMapIdSet)) {
    return OkStatus();
  }

  for (const auto& voxel : obstacles.GetInBoxVoxelRange(moving.aabb)) {
    for (int object_index = 0; object_index < num_moving; object_index++) {
      const auto& object = moving.objects[object_index];
      if (options.GetExclusionSet().Overlaps(object.flags.id_set)) {
        continue;
      }
      if (!ShouldCheckPair(object.flags, obstacles.flags()) ||
          !(object.flags.inclusion_set.Contains(voxel.object_id)) ||
          !DoOverlap(object.aabb, voxel.sphere)) {
        continue;
      }

      Scalar& object_distance = result.GetMinimumDistance(object_index);
      typename CollisionResult<Scalar, AllocatorTraits>::ContactPointInfo*
          object_contact_point_info =
              options.GetType() >= QueryOptions::kComputeContactPoints
                  ? &result.GetContactPointInfo(object_index)
                  : nullptr;
      ObjectIdSet& object_hits = result.GetObjectHits(object_index);
      // Check object spheres vs. obstacle sphere.
      for (const auto& sphere : object.spheres) {
        const auto center_distance_result =
            DistanceSquared(sphere, voxel.sphere);
        const Scalar distance =
            std::sqrt(center_distance_result.distance_squared) - sphere.radius -
            voxel.sphere.radius;
        if (distance <= Scalar{0}) {
          result.SetHasCollisions(true);
          object_hits.Insert(voxel.object_id);
          if (options.GetType() == QueryOptions::kIsCollisionFree) {
            return OkStatus();
          }
        }
        if (options.GetType() < QueryOptions::kComputeObjectDistances) {
          continue;
        }
        if (distance >= object_distance) {
          continue;
        }
        object_distance = distance;
        result.SetMinimumDistance(
            std::min({result.GetMinimumDistance(), object_distance}));
        result.SetMinimumObstacleInfo(voxel.sphere.center, voxel.index);
        if (options.GetType() >= QueryOptions::kComputeContactPoints) {
          const Vector3<Scalar> a_normal_b =
              (voxel.sphere.center - sphere.center).normalized();
          CC_CHECK_NE(object_contact_point_info, nullptr);
          *object_contact_point_info = {
              sphere.center + a_normal_b * sphere.radius,
              voxel.sphere.center - a_normal_b * voxel.sphere.radius,
              a_normal_b, ObjectIdSet(voxel.object_id)};
        }

        object_distance = std::min(object_distance, distance);
        result.SetMinimumDistance(
            std::min({result.GetMinimumDistance(), object_distance}));
      }

      // Check object capsules vs. obstacle sphere.
      for (const auto& capsule : object.capsules) {
        const auto [center_distance_squared, min_segment_param] =
            DistanceSquared(voxel.sphere, capsule);
        const Scalar distance = std::sqrt(center_distance_squared) -
                                capsule.radius - voxel.sphere.radius;
        if (distance <= Scalar{0}) {
          result.SetHasCollisions(true);
          object_hits.Insert(voxel.object_id);
          if (options.GetType() == QueryOptions::kIsCollisionFree) {
            return OkStatus();
          }
        }
        if (options.GetType() < QueryOptions::kComputeObjectDistances) {
          continue;
        }
        if (distance >= object_distance) {
          continue;
        }
        object_distance = distance;
        result.SetMinimumDistance(
            std::min({result.GetMinimumDistance(), object_distance}));
        result.SetMinimumObstacleInfo(voxel.sphere.center, voxel.index);
        if (options.GetType() >= QueryOptions::kComputeContactPoints) {
          const Vector3<Scalar> contact =
              capsule.center + capsule.direction * min_segment_param;
          const Vector3<Scalar> a_normal_b =
              (voxel.sphere.center - contact).normalized();
          CC_CHECK_NE(object_contact_point_info, nullptr);
          *object_contact_point_info = {
              capsule.center + a_normal_b * capsule.radius,
              voxel.sphere.center - a_normal_b * voxel.sphere.radius,
              a_normal_b, ObjectIdSet(voxel.object_id)};
        }
      }
      // Check object boxes vs. obstacle sphere.
      for (const auto& box : object.boxes) {
        const auto [center_distance_squared, point_on_box] =
            DistanceSquared(voxel.sphere, box);
        const Scalar distance =
            std::sqrt(center_distance_squared) - voxel.sphere.radius;

        if (distance <= Scalar{0}) {
          result.SetHasCollisions(true);
          object_hits.Insert(voxel.object_id);
          if (options.GetType() == QueryOptions::kIsCollisionFree) {
            return OkStatus();
          }
        }
        if (options.GetType() < QueryOptions::kComputeObjectDistances) {
          continue;
        }
        if (distance >= object_distance) {
          continue;
        }
        object_distance = distance;
        result.SetMinimumDistance(
            std::min({result.GetMinimumDistance(), object_distance}));
        result.SetMinimumObstacleInfo(voxel.sphere.center, voxel.index);
        if (options.GetType() >= QueryOptions::kComputeContactPoints) {
          const Vector3<Scalar> contact =
              box.box_rotation_world.transpose() * point_on_box + box.center;
          // Note: if the sphere center is exactly on the box boundary, the
          // normal computation fails. Consider computing normal from box face
          // instead in these cases.
          const Vector3<Scalar> normal =
              internal::NormalizeAndMaybeLog(voxel.sphere.center - contact);
          CC_CHECK_NE(object_contact_point_info, nullptr);
          *object_contact_point_info = {
              contact, voxel.sphere.center - normal * voxel.sphere.radius,
              normal, ObjectIdSet(voxel.object_id)};
        }
      }
    }
  }

  return OkStatus();
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COMPUTE_COLLISIONS_H_
