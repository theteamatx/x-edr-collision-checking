// Utility functions for generating protobuf messages.

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_PROTO_UTILS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_PROTO_UTILS_H_

#include <array>
#include <string>

#include "experimental/users/buschmann/collision_checking/bounding_box.proto.h"
#include "experimental/users/buschmann/collision_checking/collision_result.h"
#include "experimental/users/buschmann/collision_checking/collision_state.proto.h"
#include "experimental/users/buschmann/collision_checking/composite_object.h"
#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "experimental/users/buschmann/collision_checking/geometry.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/eigen_proto_utils.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/geometry_shapes.proto.h"
#include "experimental/users/buschmann/collision_checking/object_id.h"
#include "experimental/users/buschmann/collision_checking/voxel_map_object.h"
#include "experimental/users/buschmann/collision_checking/voxel_map_object_name.h"
#include "third_party/absl/functional/function_ref.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"

namespace collision_checking {

// Returns a CollisionStateProto or a Status if an error occurred.
// This function is not real-time safe.
template <typename Scalar, typename AllocatorTraits>
absl::StatusOr<CollisionStateProto> MakeCollisionStateProto(
    absl::FunctionRef<absl::string_view(const ObjectIdSet set)> id_set_to_name,
    const VoxelMapObject<Scalar>& obstacles,
    const CollisionObjects<Scalar, AllocatorTraits>& objects,
    const CollisionResult<Scalar, AllocatorTraits>& result);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

template <typename Scalar>
geometry_shapes::proto::Marker ToMarkerProto(const Sphere<Scalar>& sphere) {
  geometry_shapes::proto::Marker marker;
  auto* pose = marker.mutable_pose();
  auto* shape = marker.mutable_shape();

  shape->mutable_sphere()->set_radius(sphere.radius);
  ProtoFromPose(
      Pose3d(Vector3d(sphere.center.x(), sphere.center.y(), sphere.center.z())),
      pose);
  return marker;
}

namespace proto_utils_internal {
template <typename Scalar>
Pose3d CapsulePose(const Capsule<Scalar>& capsule) {
  // The geometry_shapes::proto::Capsule is always aligned with the z-axis.
  // Determine a rotation s.t. the capsule is aligned with the z-direction.
  const Vector3d direction_normalized =
      capsule.direction.template cast<double>().normalized();
  std::array<Vector3d, 2> vw = ExtendToOrthonormalBasis(direction_normalized);
  Matrix3d rotation = Matrix3d::Identity();
  rotation.col(0) = vw[0];
  rotation.col(1) = vw[1];
  rotation.col(2) = direction_normalized;
  Quaterniond quaternion(rotation);
  return Pose3d(quaternion, capsule.center.template cast<double>());
}
}  // namespace proto_utils_internal

template <typename Scalar>
geometry_shapes::proto::Marker ToMarkerProto(const Capsule<Scalar>& capsule) {
  geometry_shapes::proto::Marker marker;
  auto* pose = marker.mutable_pose();
  auto* shape = marker.mutable_shape();

  shape->mutable_capsule()->set_radius(capsule.radius);
  shape->mutable_capsule()->set_length(2.0 * capsule.half_length);

  ProtoFromPose(proto_utils_internal::CapsulePose(capsule), pose);
  return marker;
}

template <typename Scalar>
AxisAlignedBoundingBox3dProto ToBoundingBoxProto(
    const AlignedBox<Scalar>& box) {
  AxisAlignedBoundingBox3dProto box_proto;
  box_proto.mutable_min_corner()->mutable_vec()->Reserve(3);
  box_proto.mutable_max_corner()->mutable_vec()->Reserve(3);

  for (int i = 0; i < 3; i++) {
    box_proto.mutable_min_corner()->add_vec(box.low[i]);
    box_proto.mutable_max_corner()->add_vec(box.high[i]);
  }
  return box_proto;
}

template <typename Scalar>
geometry_shapes::proto::Marker ToMarkerProto(const Box<Scalar>& box) {
  geometry_shapes::proto::Marker marker;
  auto* pose = marker.mutable_pose();
  auto* shape = marker.mutable_shape();

  *shape->mutable_box()->mutable_size() = ProtoFromEigenVector(
      Vector3d(box.half_lengths[0] * 2, box.half_lengths[1] * 2,
               box.half_lengths[2] * 2));

  ProtoFromPose(
      Pose3d(box.box_rotation_world.transpose().template cast<double>(),
             box.center.template cast<double>()),
      pose);
  return marker;
}

template <typename Scalar, typename AllocatorTraits>
absl::StatusOr<CollisionStateProto> MakeCollisionStateProto(
    absl::FunctionRef<absl::string_view(const ObjectIdSet set)> id_set_to_name,
    const VoxelMapObject<Scalar>& obstacles,
    const CollisionObjects<Scalar, AllocatorTraits>& objects,
    const CollisionResult<Scalar, AllocatorTraits>& result) {
  if (result.GetObjectCount() != objects.objects.size()) {
    return absl::InvalidArgumentError("`result` doesn't match `objects`.");
  }

  CollisionStateProto proto;

  proto.mutable_collision_objects()->Reserve(result.GetObjectCount());
  for (int idx = 0; idx < result.GetObjectCount(); idx++) {
    auto* object_info = proto.mutable_collision_objects()->Add();
    object_info->mutable_id_info()->set_id_set(
        objects.objects[idx].flags.id_set.AsSetType());
    object_info->mutable_id_info()->set_inclusion_set(
        objects.objects[idx].flags.inclusion_set.AsSetType());
    object_info->set_colliding(!result.GetObjectHits(idx).Empty());
    auto* marker = object_info->mutable_marker();
    marker->set_name(id_set_to_name(objects.objects[idx].flags.id_set));
    // Composite marker pose is identity by definition.
    // Pose is contained in the primitives.
    ProtoFromPose(Pose3d(), marker->mutable_pose());

    const auto& composite_object = objects.objects[idx];
    for (const auto& sphere : composite_object.spheres) {
      *marker->add_markers() = ToMarkerProto(sphere);
    }
    for (const auto& capsule : composite_object.capsules) {
      *marker->add_markers() = ToMarkerProto(capsule);
    }
    for (const auto& box : composite_object.boxes) {
      *marker->add_markers() = ToMarkerProto(box);
    }

    *object_info->mutable_bounding_box() =
        ToBoundingBoxProto(objects.objects[idx].aabb);
  }

  *proto.mutable_collision_objects_bounding_box() =
      ToBoundingBoxProto(objects.aabb);

  if (obstacles.size() > 0) {
    auto& map_info = *proto.mutable_environment_map_object_info();
    map_info.mutable_id_info()->set_id_set(
        obstacles.flags().id_set.AsSetType());
    map_info.mutable_id_info()->set_inclusion_set(
        obstacles.flags().inclusion_set.AsSetType());
    auto& spheres = *map_info.mutable_spheres();
    auto& sphere_ids = *map_info.mutable_sphere_ids();
    spheres.set_name(std::string(kVoxelMapObjectName));
    // Pose is identity by definition.
    ProtoFromPose(Pose3d(), spheres.mutable_pose());

    spheres.mutable_markers()->Reserve(obstacles.size());
    sphere_ids.Reserve(obstacles.size());

    for (const auto& voxel : obstacles.GetVoxelRange()) {
      *spheres.add_markers() = ToMarkerProto(voxel.sphere);
      *sphere_ids.Add() = ObjectIdSet(voxel.object_id).AsSetType();
    }
  }

  auto& object_names = *proto.mutable_object_names();
  object_names.Reserve(object_names.size() + kVoxelMapIds.size());
  for (int idx = 0; idx < kVoxelMapIds.size(); idx++) {
    auto& map_name = *object_names.Add();
    map_name.set_id_set(ObjectIdSet(kVoxelMapIds[idx]).AsSetType());
    map_name.set_name(absl::StrCat(kVoxelMapObjectName, "_", idx));
  }
  for (int idx = 0; idx < result.GetObjectCount(); idx++) {
    auto& object_name = *object_names.Add();
    object_name.set_id_set(objects.objects[idx].flags.id_set.AsSetType());
    object_name.set_name(id_set_to_name(objects.objects[idx].flags.id_set));
  }

  return proto;
}
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_PROTO_UTILS_H_
