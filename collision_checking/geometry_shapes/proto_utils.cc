#include "experimental/users/buschmann/collision_checking/geometry_shapes/proto_utils.h"

#include <memory>
#include <string>
#include <vector>

#include "experimental/users/buschmann/collision_checking/geometry_shapes/geometry_shapes.proto.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/utils_impl.h"
#include "third_party/absl/status/statusor.h"

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
