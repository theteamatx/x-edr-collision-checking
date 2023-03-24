#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_PROTO_UTILS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_PROTO_UTILS_H_

#include <memory>

#include "collision_checking/assembly/assembly.h"
#include "collision_checking/assembly/assembly.proto.h"
#include "third_party/absl/status/statusor.h"

namespace collision_checking {

using AssemblyProto = proto::Assembly;
using ShapeInfoProto = proto::ShapeInfo;

// Creates an Assembly object from the given proto.
absl::StatusOr<Assembly> FromProto(const proto::Assembly& assembly_proto,
                                   bool finalize = true);

// Populates an Assembly proto with the given assembly's links and joints.
absl::Status ToProto(const Assembly& assembly, proto::Assembly* assembly_proto);
absl::StatusOr<proto::Assembly> ToProto(const Assembly& assembly);

absl::StatusOr<proto::Link> ToProto(const Link& link);

absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>> MakeShapeFromProto(
    const proto::ShapeInfo& shape_proto);

absl::Status ToProto(const geometry_shapes::ShapeBase& shape,
                     proto::ShapeInfo* shape_proto);

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_PROTO_UTILS_H_
