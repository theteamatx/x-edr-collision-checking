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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_PROTO_UTILS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_PROTO_UTILS_H_

#include <memory>

#include "absl/status/statusor.h"
#include "collision_checking/assembly/assembly.h"
#include "collision_checking/assembly/assembly.pb.h"

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
