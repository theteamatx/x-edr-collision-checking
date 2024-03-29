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

#ifndef COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_UTILS_IMPL_H_
#define COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_UTILS_IMPL_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry_shapes/box.h"
#include "collision_checking/geometry_shapes/capsule.h"
#include "collision_checking/geometry_shapes/composite_shape.h"
#include "collision_checking/geometry_shapes/eigen_proto_utils.h"
#include "collision_checking/geometry_shapes/geometry_shapes.pb.h"
#include "collision_checking/geometry_shapes/shape_base.h"
#include "collision_checking/geometry_shapes/sphere.h"

namespace collision_checking {
namespace geometry_shapes {
namespace utils_impl {

template <typename Marker>
void MakeMarkerCommon(const Pose3d& pose, const Vector4d& rgba, Marker* out) {
  CC_CHECK_NE(out, nullptr);
  ProtoFromPose(pose, out->mutable_pose());
  out->mutable_color()->set_r(rgba[0]);
  out->mutable_color()->set_g(rgba[1]);
  out->mutable_color()->set_b(rgba[2]);
  out->mutable_color()->set_a(rgba[3]);
}

template <typename Shape>
void MakeBoxShape(const Vector3d& size, Shape* shape) {
  ProtoFromVector(size, shape->mutable_box()->mutable_size());
}

template <typename Marker>
void MakeBox(const Pose3d& pose, const Vector4d& rgba, const Vector3d& size,
             Marker* out) {
  MakeMarkerCommon(pose, rgba, out);
  MakeBoxShape(size, out->mutable_shape());
}

template <typename Shape>
void MakeCapsuleShape(double length, double radius, Shape* shape) {
  shape->mutable_capsule()->set_length(length);
  shape->mutable_capsule()->set_radius(radius);
}

template <typename Marker>
void MakeCapsule(const Pose3d& pose, const Vector4d& rgba, double length,
                 double radius, Marker* out) {
  MakeMarkerCommon(pose, rgba, out);
  MakeCapsuleShape(length, radius, out->mutable_shape());
}

template <typename Shape>
void MakeSphereShape(double radius, Shape* shape) {
  shape->mutable_sphere()->set_radius(radius);
}

template <typename Marker>
void MakeSphere(const Pose3d& pose, const Vector4d& rgba, double radius,
                Marker* out) {
  MakeMarkerCommon(pose, rgba, out);
  MakeSphereShape(radius, out->mutable_shape());
}

template <typename ShapePrimitive>
absl::StatusOr<std::unique_ptr<ShapeBase>> FromProto(
    const ShapePrimitive& primitive) {
  if (primitive.has_box()) {
    // Ensure that all dimensions are positive.
    const Vector3d dimensions = EigenVectorFromProto(primitive.box().size());
    if (dimensions.minCoeff() <= 0.0) {
      return absl::InvalidArgumentError("Box dimensions must be positive.");
    }
    return std::make_unique<Box>(dimensions);
  } else if (primitive.has_capsule()) {
    if (primitive.capsule().radius() <= 0.0 ||
        primitive.capsule().length() <= 0.0) {
      return absl::InvalidArgumentError(
          "Capsule length and radius must be positive.");
    }
    return std::make_unique<Capsule>(primitive.capsule().length(),
                                     primitive.capsule().radius());
  } else if (primitive.has_sphere()) {
    if (primitive.sphere().radius() <= 0.0) {
      return absl::InvalidArgumentError("Sphere radius must be positive.");
    }
    return std::make_unique<Sphere>(primitive.sphere().radius());
  } else if (!primitive.sub_shapes().empty()) {
    std::vector<std::unique_ptr<ShapeBase>> sub_shapes;
    for (const auto& sub_shape_proto : primitive.sub_shapes()) {
      auto sub_shape = FromProto(sub_shape_proto.shape());
      if (!sub_shape.ok()) {
        return sub_shape.status();
      }
      sub_shapes.emplace_back(std::move(*sub_shape));
      sub_shapes.back()->SetLocalTransform(
          PoseFromProto(sub_shape_proto.pose()));
    }
    return std::make_unique<CompositeShape>(std::move(sub_shapes));
  }

  return absl::InvalidArgumentError("Unsupported or empty shape in the proto.");
}

// Converts a Shape into a ShapePrimitiveProto.
template <typename ShapePrimitive>
absl::Status ToShapePrimitiveProto(const ShapeBase& shape,
                                   ShapePrimitive* shape_primitive) {
  switch (shape.GetType()) {
    case ShapeType::BOX: {
      const Box& box = shape.Get<Box>();
      MakeBoxShape(box.GetSize(), shape_primitive);
      break;
    }
    case ShapeType::CAPSULE: {
      const Capsule& capsule = shape.Get<Capsule>();
      MakeCapsuleShape(capsule.GetLength(), capsule.GetRadius(),
                       shape_primitive);
      break;
    }
    case ShapeType::SPHERE: {
      const Sphere& sphere = shape.Get<Sphere>();
      MakeSphereShape(sphere.GetRadius(), shape_primitive);
      break;
    }
    case ShapeType::COMPOSITE_SHAPE: {
      const auto& composite_shape = shape.Get<CompositeShape>();
      for (auto& sub_shape : composite_shape.GetShapes()) {
        auto* sub_shape_proto = shape_primitive->add_sub_shapes();
        ProtoFromPose(sub_shape->GetLocalTransform(),
                      sub_shape_proto->mutable_pose());
        if (const auto status = ToShapePrimitiveProto(
                *sub_shape, sub_shape_proto->mutable_shape());
            !status.ok()) {
          return status;
        }
      }
      break;
    }
    default:
      return absl::InvalidArgumentError(absl::StrCat(
          "Unsupported shape with type ", static_cast<int>(shape.GetType())));
  }
  return absl::OkStatus();
}

template <typename Marker>
absl::Status MakeMarker(const ShapeBase& shape, const Vector4d& rgba,
                        Marker* out) {
  MakeMarkerCommon(shape.GetLocalTransform(), rgba, out);
  RETURN_IF_ERROR(ToShapePrimitiveProto(shape, out->mutable_shape()));
  return absl::OkStatus();
}

}  // namespace utils_impl
}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_UTILS_IMPL_H_
