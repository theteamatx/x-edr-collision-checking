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

// Functions for converting from blue::geometry_shapes::XYZ to
// structs from ./geometry.h.

#ifndef COLLISION_CHECKING_GEOMETRY_SHAPE_CONVERSION_H_
#define COLLISION_CHECKING_GEOMETRY_SHAPE_CONVERSION_H_
#include <type_traits>

#include "absl/status/status.h"
#include "collision_checking/composite_object.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/geometry_shapes/box.h"
#include "collision_checking/geometry_shapes/capsule.h"
#include "collision_checking/geometry_shapes/composite_shape.h"
#include "collision_checking/geometry_shapes/shape_base.h"
#include "collision_checking/geometry_shapes/sphere.h"
#include "collision_checking/geometry_shapes/spheres.h"
#include "collision_checking/voxel_map_object.h"

namespace collision_checking {

// Convert a geometry_shapes::Sphere to a collision::Sphere, with additional
// `padding` applied.
// Asserts that shape is the expected type.
template <typename Scalar>
Sphere<Scalar> GeometryShapesSphereToSphere(
    const Pose3d& world_pose_shape, const geometry_shapes::ShapeBase& shape,
    Scalar padding);

// Convert a geometry_shapes::Capsule to a collision::Capsule, with additional
// `padding` applied.
// Asserts that shape is the expected type.
template <typename Scalar>
Capsule<Scalar> GeometryShapesCapsuleToCapsule(
    const Pose3d& world_pose_shape, const geometry_shapes::ShapeBase& shape,
    Scalar padding);

// Convert a geometry_shapes::Box to a collision::Box.
// Asserts that shape is the expected type.
template <typename Scalar>
Box<Scalar> GeometryShapesBoxToBox(const Pose3d& world_pose_shape,
                                   const geometry_shapes::ShapeBase& shape,
                                   Scalar padding);

// Increments values in `primitives_count` according to the contents of `shape`.
// This function is recursive if `shape` is a composite shape.
absl::Status AddGeometryShapesToPrimitivesCount(
    const geometry_shapes::ShapeBase& shape, PrimitivesCount& primitives_count);

// Adds geometry from `shape` to `objects`, increasing the size according to
// `margin`.
// Primitives are added to `objects` starting starting at `indices`, and members
// of `indices` are incremented.
// This function is recursive if `shape` is a composite shape.
// Vectors in `objects` must be resized appropriately before calling this
// function.
template <typename Scalar, typename CompositeObject>
absl::Status AddGeometryShapesToCompositeObject(
    const Pose3d& world_pose_shape, const geometry_shapes::ShapeBase& shape,
    Scalar margin, CompositeObject& objects, PrimitivesCount& indices);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

// Convert a geometry_shapes::Sphere to a collision::Sphere, with additional
// `padding` applied.
// Asserts that shape is the expected type.
template <typename Scalar>
Sphere<Scalar> GeometryShapesSphereToSphere(
    const Pose3d& world_pose_shape, const geometry_shapes::ShapeBase& shape,
    const Scalar padding) {
  CC_CHECK_EQ(shape.GetType(), geometry_shapes::ShapeType::SPHERE);
  const auto& derived_shape = shape.Get<geometry_shapes::Sphere>();
  Sphere<Scalar> sphere;
  sphere.radius = derived_shape.GetRadius() + padding;
  const Pose3d world_pose_local = world_pose_shape * shape.GetLocalTransform();
  sphere.center = world_pose_local.translation().cast<Scalar>();
  return sphere;
}

// Convert a geometry_shapes::Capsule to a collision::Capsule.
// Asserts that shape is the expected type.
template <typename Scalar>
Capsule<Scalar> GeometryShapesCapsuleToCapsule(
    const Pose3d& world_pose_shape, const geometry_shapes::ShapeBase& shape,
    const Scalar padding) {
  CC_CHECK_EQ(shape.GetType(), geometry_shapes::ShapeType::CAPSULE);
  const auto& derived_shape = shape.Get<geometry_shapes::Capsule>();
  Capsule<Scalar> capsule;
  capsule.radius = derived_shape.GetRadius() + padding;
  const Pose3d world_pose_local = world_pose_shape * shape.GetLocalTransform();
  capsule.center = world_pose_local.translation().cast<Scalar>();
  capsule.direction =
      world_pose_local.quaternion().cast<Scalar>() * Vector3<Scalar>::UnitZ();
  capsule.half_length = derived_shape.GetLength() * 0.5;
  return capsule;
}

template <typename Scalar>
Box<Scalar> GeometryShapesBoxToBox(const Pose3d& world_pose_shape,
                                   const geometry_shapes::ShapeBase& shape,
                                   Scalar padding) {
  CC_CHECK_EQ(shape.GetType(), geometry_shapes::ShapeType::BOX);
  const auto& derived_shape = shape.Get<geometry_shapes::Box>();
  Box<Scalar> box;
  const Pose3d world_pose_local = world_pose_shape * shape.GetLocalTransform();
  box.center = world_pose_local.translation().template cast<Scalar>();
  box.box_rotation_world =
      world_pose_local.rotationMatrix().transpose().template cast<Scalar>();
  // Padding just increases the box size, i.e., doesn't truly create a padded
  // box with rounded corners.
  box.half_lengths = derived_shape.GetSize().template cast<Scalar>() * 0.5 +
                     Vector3<Scalar>::Constant(padding);
  return box;
}

absl::Status AddToSphereCount(const geometry_shapes::ShapeBase& shape,
                              size_t& sphere_count);

template <typename Scalar, typename CompositeObject>
absl::Status AddGeometryShapesToCompositeObject(
    const Pose3d& world_pose_shape, const geometry_shapes::ShapeBase& shape,
    Scalar margin, CompositeObject& object, PrimitivesCount& indices) {
  switch (shape.GetType()) {
    case geometry_shapes::ShapeType::SPHERE:
      object.spheres[indices.num_spheres++] =
          GeometryShapesSphereToSphere<Scalar>(world_pose_shape, shape, margin);
      break;
    case geometry_shapes::ShapeType::SPHERES: {
      const geometry_shapes::Spheres& geometry_spheres =
          shape.Get<geometry_shapes::Spheres>();
      for (const geometry_shapes::Sphere& sphere :
           geometry_spheres.GetSpheres()) {
        object.spheres[indices.num_spheres++] =
            GeometryShapesSphereToSphere<Scalar>(world_pose_shape, sphere,
                                                 margin);
      }
    } break;

    case geometry_shapes::ShapeType::COMPOSITE_SHAPE: {
      for (auto& sub_shape :
           shape.Get<geometry_shapes::CompositeShape>().GetShapes()) {
        if (const auto status = AddGeometryShapesToCompositeObject(
                world_pose_shape * shape.GetLocalTransform(), *sub_shape,
                margin, object, indices);
            !status.ok()) {
          return status;
        }
      }
    } break;
    case geometry_shapes::ShapeType::CAPSULE:
      object.capsules[indices.num_capsules++] =
          GeometryShapesCapsuleToCapsule<Scalar>(world_pose_shape, shape,
                                                 margin);
      break;
    case geometry_shapes::ShapeType::BOX:
      object.boxes[indices.num_boxes++] =
          GeometryShapesBoxToBox<Scalar>(world_pose_shape, shape, margin);
      break;
    default:
      return absl::UnimplementedError(
          absl::StrCat("Unsupported geometry type '",
                       geometry_shapes::ToString(shape.GetType()), "'."));
  }
  return absl::OkStatus();
}

template <typename Scalar>
absl::Status AddSpheresToVoxelMapObject(const geometry_shapes::ShapeBase& shape,
                                        Scalar margin,
                                        VoxelMapObject<Scalar>& object) {
  switch (shape.GetType()) {
    case geometry_shapes::ShapeType::SPHERE:
      object.AddSphere(GeometryShapesSphereToSphere<Scalar>(Pose3d::Identity(),
                                                            shape, margin),
                       -1);
      break;
    case geometry_shapes::ShapeType::SPHERES: {
      const geometry_shapes::Spheres& geometry_spheres =
          shape.Get<geometry_shapes::Spheres>();
      for (const geometry_shapes::Sphere& sphere :
           geometry_spheres.GetSpheres()) {
        object.AddSphere(GeometryShapesSphereToSphere<Scalar>(
                             Pose3d::Identity(), sphere, margin),
                         -1);
      }
    } break;

    case geometry_shapes::ShapeType::COMPOSITE_SHAPE: {
      for (auto& sub_shape :
           shape.Get<geometry_shapes::CompositeShape>().GetShapes()) {
        RETURN_IF_ERROR(AddSpheresToVoxelMapObject(*sub_shape, margin, object));
      }
    } break;
    default: {
      return absl::UnimplementedError(
          absl::StrCat("Unsupported geometry type '",
                       geometry_shapes::ToString(shape.GetType()), "'."));
    }
  }
  return absl::OkStatus();
}

}  // namespace collision_checking

#endif  // COLLISION_CHECKING_GEOMETRY_SHAPE_CONVERSION_H_
