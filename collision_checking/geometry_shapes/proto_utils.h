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

#ifndef COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_PROTO_UTILS_H_
#define COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_PROTO_UTILS_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry_shapes/geometry_shapes.pb.h"
#include "collision_checking/geometry_shapes/shape_base.h"

namespace collision_checking {
namespace geometry_shapes {
namespace proto {

void MakeBox(const Pose3d& pose, const Vector4d& rgba, const Vector3d& size,
             Marker* out);

void MakeCylinder(const Pose3d& pose, const Vector4d& rgba, double length,
                  double radius, Marker* out);

void MakeCapsule(const Pose3d& pose, const Vector4d& rgba, double length,
                 double radius, Marker* out);

void MakeSphere(const Pose3d& pose, const Vector4d& rgba, double radius,
                Marker* out);

absl::Status MakeMarker(const geometry_shapes::ShapeBase& shape,
                        const Vector4d& rgba, Marker* out);

// Converts a ShapePrimitive proto into a ShapeBase pointer.
absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>> FromProto(
    const ShapePrimitive& primitive);

// Converts a Shape into a ShapePrimitiveProto.
absl::Status ToProto(const geometry_shapes::ShapeBase& shape,
                     ShapePrimitive* proto);

}  // namespace proto
}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_PROTO_UTILS_H_
