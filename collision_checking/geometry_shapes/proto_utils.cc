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

#include "collision_checking/geometry_shapes/proto_utils.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "collision_checking/geometry_shapes/geometry_shapes.pb.h"
#include "collision_checking/geometry_shapes/utils_impl.h"

namespace collision_checking {
namespace geometry_shapes {
namespace proto {

void MakeBox(const Pose3d& pose, const Vector4d& rgba, const Vector3d& size,
             Marker* out) {
  utils_impl::MakeBox(pose, rgba, size, out);
}

void MakeCapsule(const Pose3d& pose, const Vector4d& rgba, double length,
                 double radius, Marker* out) {
  utils_impl::MakeCapsule(pose, rgba, length, radius, out);
}

void MakeSphere(const Pose3d& pose, const Vector4d& rgba, double radius,
                Marker* out) {
  utils_impl::MakeSphere(pose, rgba, radius, out);
}

absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>> FromProto(
    const ShapePrimitive& primitive) {
  return utils_impl::FromProto(primitive);
}

absl::Status ToProto(const geometry_shapes::ShapeBase& shape,
                     ShapePrimitive* proto) {
  return utils_impl::ToShapePrimitiveProto(shape, proto);
}

}  // namespace proto
}  // namespace geometry_shapes
}  // namespace collision_checking
