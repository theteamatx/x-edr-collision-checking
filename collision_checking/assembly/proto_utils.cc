#include "collision_checking/assembly/proto_utils.h"

#include <memory>
#include "collision_checking/assembly/assembly.proto.h"
#include "collision_checking/assembly/link.h"
#include "collision_checking/assembly/utils_impl.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"

namespace collision_checking {

// Creates an Assembly object from the given proto.
absl::StatusOr<Assembly> FromProto(const proto::Assembly& assembly_proto,
                                   bool finalize) {
  return utils_impl::FromProto(assembly_proto, finalize);
}

absl::StatusOr<proto::Assembly> ToProto(const Assembly& assembly) {
  proto::Assembly assembly_proto;
  const auto status = utils_impl::ToProto(assembly, &assembly_proto);
  if (!status.ok()) {
    return status;
  }
  return assembly_proto;
}

// Populates an Assembly proto with the given assembly's links and joints.
absl::Status ToProto(const Assembly& assembly,
                     proto::Assembly* assembly_proto) {
  return utils_impl::ToProto(assembly, assembly_proto);
}

absl::StatusOr<proto::Link> ToProto(const Link& link) {
  proto::Link link_proto;
  const auto status = utils_impl::details::ToProto(link, &link_proto);
  if (!status.ok()) {
    return status;
  }
  return link_proto;
}

absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>> MakeShapeFromProto(
    const proto::ShapeInfo& shape_proto) {
  return utils_impl::MakeShapeFromProto(shape_proto);
}

}  // namespace collision_checking
