#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_PROTO_UTILS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_PROTO_UTILS_H_

#include <memory>

#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/geometry_shapes.proto.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/shape_base.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"

namespace collision_checking {
namespace geometry_shapes {
namespace proto {

void MakeBox(const Pose3d& pose, const Vector4d& rgba,
             const Vector3d& size, Marker* out);

void MakeCylinder(const Pose3d& pose,
                  const Vector4d& rgba, double length, double radius,
                  Marker* out);

void MakeCapsule(const Pose3d& pose, const Vector4d& rgba,
                 double length, double radius, Marker* out);

void MakeSphere(const Pose3d& pose, const Vector4d& rgba,
                double radius, Marker* out);

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

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_PROTO_PROTO_UTILS_H_
